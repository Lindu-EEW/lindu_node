#include <Arduino.h>
#include <sys/time.h>
#include <esp_attr.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "ConfigManager.h"
#include "NetworkManager.h"
#include "SensorManager.h"
#include "OTAUpdater.h"
#include "StorageManager.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include "esp_ota_ops.h"
#include <Preferences.h>

#include <Adafruit_NeoPixel.h>

// --- RESCUE MODE (SAFE MODE) VARIABLES ---
// RTC memory bertahan saat ESP32 crash atau reboot
RTC_DATA_ATTR int boot_crash_count = 0;
bool is_rescue_mode = false;

// Pin & aktuator berbeda per varian board.
// ARDUINO_USB_CDC_ON_BOOT selalu terdefinisi (fallback 0 dari HardwareSerial.h
// core Arduino-ESP32) sehingga harus dicek NILAINYA, bukan defined().
// Bernilai 1 hanya pada env:esp32s3 (lihat platformio.ini).
#if ARDUINO_USB_CDC_ON_BOOT
    // ESP32-S3 DevKitC (hardware asli/lapangan): TIDAK ada perubahan apa pun
    // di jalur ini dibanding main branch - aktuator valve tetap pakai servo.
    #define USE_SERVO_VALVE 1
    #define RGB_PIN 48
    #define SERVO_PIN 5
    #define BUZZER_PIN 6
    // Buzzer S3 (hardware asli/lapangan): active-LOW
    #define BUZZER_ON  LOW
    #define BUZZER_OFF HIGH
    #define BUZZER_ON_DUTY  128
    #define BUZZER_OFF_DUTY 255
    #include <ESP32Servo.h>
    Servo myServo;
#else
    // ESP32 classic / WROOM (unit pengujian): valve & door-lock pakai relay 2-channel
    #define USE_SERVO_VALVE 0
    // LED RGB modul unit pengujian adalah LED RGB analog biasa (katoda umum,
    // 3 kaki PWM terpisah R/G/B), BUKAN NeoPixel/WS2812 satu-pin seperti di S3.
    #define RGB_R_PIN 4
    #define RGB_G_PIN 16
    #define RGB_B_PIN 17
    #define BUZZER_PIN 14
    #define RELAY_DOOR_PIN 25   // Relay CH1 -> Solenoid Door Lock 12V
    #define RELAY_VALVE_PIN 26  // Relay CH2 -> Solenoid Water/Gas Valve 12V
    #define PIR_PIN 19          // Sensor PIR (HC-SR501/sejenis) - OUTPUT sensor ke GPIO (HIGH = ada gerakan), INPUT_PULLDOWN agar aman saat sensor belum terpasang. Dipindah dari GPIO27 ke GPIO19 (uji coba sensor LD2410C sebelumnya, kembali pakai PIR).
    #define OCCUPANCY_WINDOW_MS 600000UL // 10 menit - jika ADA MINIMAL 1 deteksi PIR dalam window ini, pintu auto-unlock
    // Modul relay 2-channel (active LOW): LOW = relay ON (energized), HIGH = relay OFF
    #define RELAY_ON  LOW
    #define RELAY_OFF HIGH
    // Buzzer unit pengujian (classic/WROOM): terbukti active-HIGH secara fisik
    // (dites 2026-09-17: mencabut sinyal GPIO14 membuat buzzer diam, artinya
    // GPIO HIGH = nyala), KEBALIKAN dari asumsi lama "active-LOW".
    #define BUZZER_ON  HIGH
    #define BUZZER_OFF LOW
    #define BUZZER_ON_DUTY  128
    #define BUZZER_OFF_DUTY 0
#endif

#define BOOT_BUTTON_PIN 0

ConfigManager configMgr;
NetworkManager networkMgr;
SensorManager sensorMgr;

#if ARDUINO_USB_CDC_ON_BOOT
Adafruit_NeoPixel pixels(1, RGB_PIN, NEO_GRB + NEO_KHZ800);
#else
// Shim minimal yang meniru API Adafruit_NeoPixel (begin/Color/setPixelColor/show)
// agar kode animasi LED di bawah tidak perlu diubah, tapi outputnya lewat
// 3 pin analogWrite/PWM biasa ke LED RGB modul analog (katoda umum).
class AnalogRGB {
public:
    AnalogRGB(uint8_t rPin, uint8_t gPin, uint8_t bPin) : _r(rPin), _g(gPin), _b(bPin) {}
    void begin() {
        pinMode(_r, OUTPUT);
        pinMode(_g, OUTPUT);
        pinMode(_b, OUTPUT);
    }
    static uint32_t Color(uint8_t r, uint8_t g, uint8_t b) {
        return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
    }
    void setPixelColor(uint8_t /*index*/, uint32_t color) { _color = color; }
    void show() {
        analogWrite(_r, (_color >> 16) & 0xFF);
        analogWrite(_g, (_color >> 8) & 0xFF);
        analogWrite(_b, _color & 0xFF);
    }
private:
    uint8_t _r, _g, _b;
    uint32_t _color = 0;
};
AnalogRGB pixels(RGB_R_PIN, RGB_G_PIN, RGB_B_PIN);
#endif

QueueHandle_t eventQueue;
unsigned long global_alarm_until = 0;
bool is_valve_locked = false;
Preferences actPrefs;
#if !ARDUINO_USB_CDC_ON_BOOT
bool is_door_locked = true; // Default: pintu terkunci (khusus unit ESP32 classic)
#endif
bool is_motion_detected = false; // Khusus unit ESP32 classic (sensor PIR belum dipasang di S3), selalu false di S3
unsigned long last_motion_ms = 0; // Timestamp deteksi PIR terakhir (khusus classic), dipakai untuk window occupancy 10 menit
float local_latest_pga = 0.0;
unsigned long local_alarm_until = 0;
unsigned long identify_until = 0;
bool bedroom_stealth_mode = false;
TaskHandle_t networkTaskHandle;

// --- P2P LOCAL LAN INTER-NODE CONFIRMATION (UDP Port 5055) ---
WiFiUDP p2pUdp;
const uint16_t P2P_UDP_PORT = 5055;

struct PeerTrigger {
    char node_id[32];
    char group_id[32];
    float pga;
    unsigned long received_ms;
};
#define MAX_PEER_RECORDS 16
PeerTrigger peerTriggers[MAX_PEER_RECORDS];
int peerCount = 0;
unsigned long last_self_trigger_ms = 0;

// Helper: Ambil Waktu Epoch NTP
double getEpochTime() {
    struct timeval tv;
    if (gettimeofday(&tv, NULL) != 0) return 0.0;
    return (double)tv.tv_sec + (double)tv.tv_usec / 1000000.0;
}

// TASK CORE 0: Mengurus Wi-Fi, MQTT, JSON, Edge Computing, dan Animasi LED
void networkTaskCode(void* parameter) {
    networkMgr.begin(&configMgr);
    
    // Sinkronisasi Waktu NTP untuk Timestamp
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    
    pixels.begin();
    p2pUdp.begin(P2P_UDP_PORT);
    Serial.printf("[P2P LAN] UDP Listener aktif di port %u (Grup: %s)\n", P2P_UDP_PORT, configMgr.config.group_id);
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, BUZZER_OFF); // MATI (lihat definisi BUZZER_OFF per varian board)

#if USE_SERVO_VALVE
    // ESP32-S3 PWM Timer Allocation untuk Servo (Hanya alokasi, tidak di-enable)
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    myServo.setPeriodHertz(50);
    // Servo sengaja TIDAK di-attach di sini agar tidak auto-enable saat alat menyala
#else
    pinMode(RELAY_DOOR_PIN, OUTPUT);
    pinMode(RELAY_VALVE_PIN, OUTPUT);
    digitalWrite(RELAY_DOOR_PIN, is_door_locked ? RELAY_OFF : RELAY_ON);
    digitalWrite(RELAY_VALVE_PIN, is_valve_locked ? RELAY_OFF : RELAY_ON);
    // INPUT_PULLDOWN (bukan INPUT polos): kalau sensor PIR belum/tidak terpasang, GPIO
    // mengambang bisa kebaca HIGH terus oleh noise, membuat occupancy logic mengira
    // ada gerakan terus-menerus dan pintu tidak pernah terkunci. Pull-down memaksa
    // default LOW (tidak ada gerakan) saat tidak ada sensor yang aktif men-drive pin.
    pinMode(PIR_PIN, INPUT_PULLDOWN);
#endif

    float breathAngle = 0;

    SensorEvent ev;
    
    while(1) {
        networkMgr.loop(); // Handle rutin MQTT
        
        // ========================================================
        // P2P LOCAL LAN INTER-NODE CONFIRMATION (UDP Port 5055)
        // ========================================================
        if (WiFi.status() == WL_CONNECTED) {
            int pSize = p2pUdp.parsePacket();
            if (pSize > 0) {
                char pBuf[256];
                int pLen = p2pUdp.read(pBuf, sizeof(pBuf) - 1);
                if (pLen > 0) {
                    pBuf[pLen] = '\0';
                    StaticJsonDocument<256> pDoc;
                    DeserializationError pErr = deserializeJson(pDoc, pBuf);
                    if (!pErr) {
                        const char* pType = pDoc["type"];
                        const char* pNode = pDoc["node"];
                        const char* pGroup = pDoc["group"];
                        float pPga = pDoc["pga"] | 0.0f;
                        
                        // Hanya proses jika paket bertipe QUAKE, dari grup yang sama, dan bukan diri sendiri
                        if (pType && strcmp(pType, "QUAKE") == 0 &&
                            pGroup && strcmp(pGroup, configMgr.config.group_id) == 0 &&
                            pNode && strcmp(pNode, configMgr.config.node_id) != 0) {
                            
                            unsigned long now_ms = millis();
                            bool found = false;
                            for (int i = 0; i < peerCount; i++) {
                                if (strcmp(peerTriggers[i].node_id, pNode) == 0) {
                                    peerTriggers[i].pga = pPga;
                                    peerTriggers[i].received_ms = now_ms;
                                    found = true;
                                    break;
                                }
                            }
                            if (!found && peerCount < MAX_PEER_RECORDS) {
                                strlcpy(peerTriggers[peerCount].node_id, pNode, sizeof(peerTriggers[peerCount].node_id));
                                strlcpy(peerTriggers[peerCount].group_id, pGroup, sizeof(peerTriggers[peerCount].group_id));
                                peerTriggers[peerCount].pga = pPga;
                                peerTriggers[peerCount].received_ms = now_ms;
                                peerCount++;
                            }
                            Serial.printf("[P2P LAN] Konfirmasi getaran dari %s (PGA: %.4fG) dalam grup '%s'!\n",
                                          pNode, pPga, pGroup);
                        }
                    }
                }
            }
            
            // Evaluasi Quorum LAN Lokal (Zero-Server / Edge Inter-Node Consensus)
            unsigned long now_ms = millis();
            int recent_peers = 0;
            float max_peer_pga = 0.0f;
            for (int i = 0; i < peerCount; i++) {
                if (now_ms - peerTriggers[i].received_ms <= 5000) {
                    recent_peers++;
                    if (peerTriggers[i].pga > max_peer_pga) max_peer_pga = peerTriggers[i].pga;
                }
            }
            bool self_active = (last_self_trigger_ms > 0 && (millis() - last_self_trigger_ms <= 5000));
            int total_group_active = recent_peers + (self_active ? 1 : 0);
            if (self_active && local_latest_pga > max_peer_pga) {
                max_peer_pga = local_latest_pga;
            }
            
            // KONSENSUS SEISMOLOGI (Arsitektur Lindu):
            // 1. Saat SERVER ONLINE (Normal): Seluruh event dikirim ke Server terlebih dahulu.
            //    Server konsensus memvalidasi gelombang P-Wave & Quorum 3 node.
            //    Node HANYA membunyikan sirine jika menerima perintah resmi 'trigger_siren' dari server.
            // 2. Saat SERVER OFFLINE (!networkMgr.isConnected()): P2P LAN Quorum aktif sebagai fail-safe darurat.
            if (!networkMgr.isConnected() && (global_alarm_until == 0 || millis() >= global_alarm_until)) {
                if (total_group_active >= 3 && max_peer_pga >= 0.05f) {
                    Serial.printf("\n🚨 [P2P OFFLINE FAIL-SAFE] %d Node di grup '%s' bergetar serentak (PGA %.4fG) saat server offline! Memicu alarm darurat & solenoid valve!\n",
                                  total_group_active, configMgr.config.group_id, max_peer_pga);
                    global_alarm_until = millis() + 15000; // Nyalakan sirine & LED merah 15 detik
                    is_valve_locked = true;                // Kunci valve keselamatan
                    actPrefs.putBool("valve_locked", true);
                }
            } else if (total_group_active >= 3) {
                // Online Advisory: Hanya catat info konsensus P2P tanpa mendahului perintah server
                static unsigned long last_p2p_adv = 0;
                if (millis() - last_p2p_adv >= 10000) {
                    last_p2p_adv = millis();
                    Serial.printf("\nℹ️ [P2P LAN INFO] %d Node di grup '%s' serentak aktif (PGA: %.4fG). Menunggu verifikasi master server...\n",
                                  total_group_active, configMgr.config.group_id, max_peer_pga);
                }
            }
        }
        
        // Heartbeat status setiap 3 detik (dipercepat agar status fleet dan PIR lebih responsif)
        static unsigned long last_status = 0;
        if (millis() - last_status >= 3000) {
            networkMgr.publishStatus("online", sensorMgr.sensor_ok, sensorMgr.getTiltAngle(), sensorMgr.getPose(), is_motion_detected);
            last_status = millis();
        }
        
        // Cek status alarm global dan lokal
        bool is_global_alarm = (millis() < global_alarm_until && global_alarm_until > 0);
        bool is_local_alarm = (millis() < local_alarm_until && local_alarm_until > 0);
        
        // Animasi LED Cerdas (Sesuai Status Sensor & WiFi)
        if (WiFi.status() == WL_CONNECTED && networkMgr.isConnected()) {
            otaUpdater.loop();
            
            // Mode Kamar Tidur (Bedroom Sleep Beat):
            // - Jika bedroom_stealth_mode aktif -> Lampu 100% MATI (Gelap gulita di kamar tidur)
            // - Jika normal -> Siklus denyut lambat 8.0 detik dengan 6.4 detik OFF-BEAT TOTAL
            //   (Hanya menyala sangat redup 1.6 detik maks 3/255, lalu 6.4 detik mati total)
            int brightness = 0;
            if (!bedroom_stealth_mode) {
                unsigned long cycle_ms = millis() % 8000;
                if (cycle_ms < 1600) {
                    float pulse_phase = (float)cycle_ms / 1600.0f;
                    brightness = (int)(sin(pulse_phase * PI) * 3.0f);
                }
            }
            
            bool hw611_ok = sensorMgr.bme_ok || sensorMgr.bmp_ok;
            
#if USE_SERVO_VALVE
            // LOGIKA VALVE MANUAL RESET (Hanya nyalakan motor saat diperintah sistem)
            static bool last_valve_state = false;
            if (is_valve_locked != last_valve_state) {
                // PRE-WRITE: Set target sudut SEBELUM motor dialiri listrik agar tidak melompat kaget
                int target_angle = is_valve_locked ? 90 : 0;
                myServo.write(target_angle); 
                
                // Beri jeda sangat kecil sebelum attach untuk stabilitas sinyal PWM FreeRTOS
                vTaskDelay(pdMS_TO_TICKS(50));
                
                // Nyalakan tenaga motor
                myServo.attach(SERVO_PIN, 500, 2400); 
                myServo.write(target_angle); // Tulis ulang untuk memastikan sinyal PWM terkirim

                // Beri waktu 2.5 detik (cukup panjang) agar motor yang membawa beban fisik berat
                // punya cukup waktu untuk sampai ke tujuan sebelum listriknya dicabut
                vTaskDelay(pdMS_TO_TICKS(2500)); 
                
                // Matikan aliran listrik (Zero-Torque Standby)
                myServo.detach();

                last_valve_state = is_valve_locked;
            }
#else
            // LOGIKA RELAY VALVE (Solenoid Water/Gas Valve 12V)
            // Fail-safe: valve tertutup (OFF) saat is_valve_locked true (gempa/gas bocor/manual)
            static bool last_valve_state = is_valve_locked;
            if (is_valve_locked != last_valve_state) {
                digitalWrite(RELAY_VALVE_PIN, is_valve_locked ? RELAY_OFF : RELAY_ON);
                last_valve_state = is_valve_locked;
            }

            // SENSOR PIR: Deteksi gerakan/orang di lokasi (HIGH = ada gerakan).
            // Dibaca duluan agar window occupancy di bawah selalu pakai data terbaru.
            is_motion_detected = digitalRead(PIR_PIN) == HIGH;
            if (is_motion_detected) {
                last_motion_ms = millis();
            }
            {
                static bool last_pir_debug = false;
                if (is_motion_detected != last_pir_debug) {
                    Serial.printf("[PIR DEBUG] motion=%s\n", is_motion_detected ? "TERDETEKSI" : "tidak ada");
                    last_pir_debug = is_motion_detected;
                }
            }

            // LOGIKA RELAY DOOR LOCK (Solenoid Door Lock 12V)
            // Default: ikut perintah manual (lock_door/unlock_door) atau cancel_alarm.
            // KHUSUS selama alarm gempa aktif (is_global_alarm): pintu HANYA unlock kalau
            // ADA MINIMAL 1 deteksi PIR dalam 10 menit terakhir (asumsi ada orang di lokasi
            // untuk evakuasi); kalau tidak ada yang terdeteksi, pintu tetap terkunci
            // walau alarm aktif. Dievaluasi ulang tiap tick SELAMA alarm masih berlangsung
            // (bukan cuma sekali di saat trigger) supaya tetap responsif kalau ada orang
            // baru terdeteksi di tengah-tengah alarm. Di luar alarm, occupancy PIR tidak
            // dicek sama sekali - status pintu murni ikut perintah manual/cancel_alarm.
            static bool last_door_state = is_door_locked;
            if (is_global_alarm) {
                bool occupancy_recent = (last_motion_ms > 0) && (millis() - last_motion_ms < OCCUPANCY_WINDOW_MS);
                is_door_locked = !occupancy_recent;
            }
            if (is_door_locked != last_door_state) {
                digitalWrite(RELAY_DOOR_PIN, is_door_locked ? RELAY_OFF : RELAY_ON);
                last_door_state = is_door_locked;
            }
#endif


            if (millis() < identify_until) {
                // IDENTIFY MODE: Berkedip Putih
                if ((millis() / 200) % 2 == 0) pixels.setPixelColor(0, pixels.Color(255, 255, 255));
                else pixels.setPixelColor(0, pixels.Color(0, 0, 0));
                // analogWrite (LEDC) dulu, baru digitalWrite - digitalWrite saja TIDAK memutus
                // channel PWM yang sudah dipakai analogWrite (mis. saat alarm sebelumnya), jadi
                // buzzer bisa nyangkut bunyi panjang terus-menerus kalau cuma digitalWrite.
                analogWrite(BUZZER_PIN, BUZZER_OFF_DUTY);
                digitalWrite(BUZZER_PIN, BUZZER_OFF);
            } else if (is_global_alarm) {
                // KONFIRMASI GEMPA (DARI SERVER): Berkedip Merah Cepat (Strobo) & Buzzer Menyala
                if ((millis() / 100) % 2 == 0) {
                    pixels.setPixelColor(0, pixels.Color(255, 0, 0));
                    analogWrite(BUZZER_PIN, BUZZER_ON_DUTY); // 50% Duty Cycle (Max Volume Tone untuk Speaker)
                } else {
                    pixels.setPixelColor(0, pixels.Color(0, 0, 0));
                    analogWrite(BUZZER_PIN, BUZZER_OFF_DUTY); // MATI
                }
            } else if (sensorMgr.gas_leak_detected) {
                // KEBOCORAN GAS TERDETEKSI (MQ-2): Berkedip Oranye & Buzzer Menyala.
                // Pola blink lebih lambat dari alarm gempa (300ms vs 100ms) supaya
                // operator bisa membedakan jenis alarm dari suara/kedipannya.
                if ((millis() / 300) % 2 == 0) {
                    pixels.setPixelColor(0, pixels.Color(255, 100, 0));
                    analogWrite(BUZZER_PIN, BUZZER_ON_DUTY);
                } else {
                    pixels.setPixelColor(0, pixels.Color(0, 0, 0));
                    analogWrite(BUZZER_PIN, BUZZER_OFF_DUTY);
                }
            } else if (is_local_alarm) {
                // DETEKSI GETARAN LOKAL (Menunggu Konfirmasi Node Lain): HANYA Berkedip Pink
                if ((millis() / 500) % 2 == 0) {
                    pixels.setPixelColor(0, pixels.Color(255, 20, 147)); // Hot Pink
                } else {
                    pixels.setPixelColor(0, pixels.Color(0, 0, 0));
                    // Wajib analogWrite juga: kalau transisi ini datang tepat setelah alarm
                    // gempa/gas (yang menyalakan buzzer via analogWrite) baru dibatalkan,
                    // digitalWrite saja tidak memutus PWM LEDC yang masih aktif di duty terakhir
                    // - itu penyebab buzzer nyangkut bunyi panjang terus-menerus.
                    analogWrite(BUZZER_PIN, BUZZER_OFF_DUTY);
                    digitalWrite(BUZZER_PIN, BUZZER_OFF);
                }
            } else if (otaUpdater.ota_status == "DOWNLOADING_FIRMWARE" || otaUpdater.ota_status == "CHECKING_GITHUB") {
                // MATIKAN BUZZER JIKA SEBELUMNYA NYALA
                analogWrite(BUZZER_PIN, BUZZER_OFF_DUTY); digitalWrite(BUZZER_PIN, BUZZER_OFF);
                
                // OTA SEDANG MENGUNDUH: Berkedip CYAN (Biru Tosca) sangat cepat layaknya loading
                if ((millis() / 80) % 2 == 0) pixels.setPixelColor(0, pixels.Color(0, 255, 255));
                else pixels.setPixelColor(0, pixels.Color(0, 0, 0));
            } else if (otaUpdater.ota_status == "UPDATE_SUCCESS_RESTARTING") {
                // OTA SELESAI & SIAP RESTART: Menyala Ungu Solid
                pixels.setPixelColor(0, pixels.Color(128, 0, 128));
            } else if (!sensorMgr.sensor_ok && !hw611_ok) {
                
                // Semua sensor mati: Berkedip Merah Cepat (Bahaya Fatal)
                if ((millis() / 200) % 2 == 0) pixels.setPixelColor(0, pixels.Color(50, 0, 0));
                else pixels.setPixelColor(0, pixels.Color(0, 0, 0));
            } else if (!sensorMgr.sensor_ok) {
                // Akselerometer mati: Bernafas Merah
                pixels.setPixelColor(0, pixels.Color(brightness, 0, 0));
            } else if (!hw611_ok) {
                // Suhu mati: Bernafas Kuning/Oranye
                pixels.setPixelColor(0, pixels.Color(brightness, brightness * 0.5, 0));
            } else {
                // Semua normal: Bernafas Hijau
                pixels.setPixelColor(0, pixels.Color(0, brightness, 0));
            }
        } else {
            // Berkedip Merah Pelan jika Wi-Fi putus
            if ((millis() / 500) % 2 == 0) pixels.setPixelColor(0, pixels.Color(30, 0, 0));
            else pixels.setPixelColor(0, pixels.Color(0, 0, 0));
        }
        
        // Matikan speaker sepenuhnya jika kondisi aman
        static bool is_buzzer_active = false;

        if (is_global_alarm || is_local_alarm || sensorMgr.gas_leak_detected) {
            is_buzzer_active = true;
        } else {
            if (is_buzzer_active) {
                // analogWrite dulu untuk memutus PWM LEDC (lihat catatan di cabang is_local_alarm
                // di atas), baru pinMode/digitalWrite untuk memastikan pin benar-benar diam.
                analogWrite(BUZZER_PIN, BUZZER_OFF_DUTY);
                pinMode(BUZZER_PIN, OUTPUT);
                digitalWrite(BUZZER_PIN, BUZZER_OFF); // mematikan arus sepenuhnya
                is_buzzer_active = false;
            }
        }
        
        pixels.show();

        // LOG EVENT AKTUATOR: kirim satu event tiap state berubah (valve, door, buzzer, led).
        // Hanya dikirim dari task ini (core 0) seperti publish lain. Cache state baru diperbarui
        // kalau publish berhasil, jadi state awal setelah boot tetap tercatat begitu MQTT tersambung.
        // Alasan (reason) adalah inferensi dari kondisi saat perubahan terdeteksi.
        {
            const char* alarm_mode = is_global_alarm ? "EARTHQUAKE"
                                   : sensorMgr.gas_leak_detected ? "GAS_LEAK"
                                   : is_local_alarm ? "LOCAL_VIBRATION"
                                   : "NORMAL";
            const char* reason = is_global_alarm ? "EARTHQUAKE_ALARM"
                               : sensorMgr.gas_leak_detected ? "GAS_LEAK"
                               : "MANUAL_OR_RESET";

            static int last_valve_log = -1;
            if (last_valve_log != (int)is_valve_locked) {
                if (networkMgr.publishActuator("valve", is_valve_locked ? "CLOSED" : "OPEN", reason, getEpochTime()))
                    last_valve_log = (int)is_valve_locked;
            }
#if !ARDUINO_USB_CDC_ON_BOOT
            static int last_door_log = -1;
            if (last_door_log != (int)is_door_locked) {
                if (networkMgr.publishActuator("door", is_door_locked ? "LOCKED" : "UNLOCKED",
                                               is_global_alarm ? "ALARM_OCCUPANCY_PIR" : "MANUAL_OR_RESET", getEpochTime()))
                    last_door_log = (int)is_door_locked;
            }
#endif
            bool siren_on = is_global_alarm || sensorMgr.gas_leak_detected;
            static int last_buzzer_log = -1;
            if (last_buzzer_log != (int)siren_on) {
                if (networkMgr.publishActuator("buzzer", siren_on ? "ON" : "OFF", alarm_mode, getEpochTime()))
                    last_buzzer_log = (int)siren_on;
            }
            static String last_led_mode = "";
            if (last_led_mode != alarm_mode) {
                if (networkMgr.publishActuator("led", alarm_mode, alarm_mode, getEpochTime()))
                    last_led_mode = alarm_mode;
            }
        }

        // Cek dan kirim antrean data sensor secara non-blocking
        while (xQueueReceive(eventQueue, &ev, 0) == pdTRUE) {
            local_latest_pga = ev.pga;
            
            // Filter Seismik Multi-Tier: Gempa bumi tektonik riil dominan pada frekuensi rendah (<= 15 Hz).
            // Hentakan langkah kaki keras, pintu dibanting, mengetik keyboard, atau meja tersenggol berfrekuensi tinggi (> 16-35 Hz)
            // difilter ketat agar tidak memicu alarm palsu.
            bool is_seismic_frequency = (ev.freq_hz == 0 || ev.freq_hz <= 15);

            // KATEGORI DETEKSI GEMPA DUA TINGKAT (MULTI-TIER):
            // 1. Gempa Kuat / Merusak (Shindo 4+, PGA >= 0.05g, STA/LTA >= 2.0, Freq <= 15Hz)
            // 2. Gempa Ringan / Terasa (Shindo 1-3, PGA >= 0.0040g, STA/LTA >= 1.50, Freq <= 15Hz)
            bool is_severe_earthquake = (ev.pga >= 0.05f && is_seismic_frequency && ev.ratio >= 2.0f) || (ev.ml_prob >= 0.65f);
            bool is_minor_earthquake = (ev.pga >= 0.0040f && is_seismic_frequency && ev.ratio >= 1.50f) || (ev.ml_prob >= 0.50f);
            bool is_significant_shake = (ev.pga >= 0.020f && ev.ratio >= 1.5f && is_seismic_frequency);
            bool is_offline = !networkMgr.isConnected();

            if (is_severe_earthquake || is_minor_earthquake) {
                last_self_trigger_ms = millis();
                
                // BROADCAST P2P UDP KE LAN (Hanya untuk getaran signifikan >= 0.02G):
                // Mengirim info getaran ke seluruh node di jaringan lokal dalam 1-3 ms
                if (WiFi.status() == WL_CONNECTED && ev.pga >= 0.02f) {
                    char udpBuf[160];
                    snprintf(udpBuf, sizeof(udpBuf), 
                             "{\"type\":\"QUAKE\",\"node\":\"%s\",\"group\":\"%s\",\"pga\":%.4f,\"ts\":%.2f}",
                             configMgr.config.node_id, configMgr.config.group_id, ev.pga, getEpochTime());
                    p2pUdp.beginPacket(IPAddress(255, 255, 255, 255), P2P_UDP_PORT);
                    p2pUdp.write((const uint8_t*)udpBuf, strlen(udpBuf));
                    p2pUdp.endPacket();
                }

                // ========== OFFLINE FAIL-SAFE (LONE WOLF MODE) ==========
                // Jika MQTT server mati DAN getaran AMAT SANGAT BRUTAL (PGA > 0.60G) dengan spektrum seismik valid
                if (!networkMgr.isConnected() && ev.pga > 0.60 && is_seismic_frequency) {
                    Serial.println("[!!!] LONE WOLF MODE: Server offline + PGA EKSTREM! Mengambil alih kendali!");
                    networkMgr.publishLog("LONE WOLF MODE DIINTIASI! Server Offline & PGA Ekstrem.");
                    global_alarm_until = millis() + 15000; // Sirine merah 15 detik
                    is_valve_locked = true;
                    actPrefs.putBool("valve_locked", true);
                }
            }

            // PEREKAM BLACKBOX GEMPA:
            // Rekam getaran gempa ke flash storage (LittleFS) saat offline maupun online.
            // Kini aktif untuk gempa ringan (Shindo 1-3, seperti Chiba M4.0) maupun gempa kuat.
            if (is_severe_earthquake || is_minor_earthquake || (is_offline && is_significant_shake)) {
                storageMgr.recordEarthquake(
                    getEpochTime(), ev.uptime_ms,
                    ev.pga, ev.ratio, ev.freq_hz, ev.rms,
                    ev.accel_x, ev.accel_y, ev.accel_z,
                    1.0f, 0.5f,
                    ev.ml_prob, ev.ml_agree,
                    is_severe_earthquake || is_minor_earthquake, is_offline
                );
            }

            networkMgr.publishEvent(
                ev.pga, ev.ratio, ev.freq_hz,
                ev.accel_x, ev.accel_y, ev.accel_z,
                ev.uptime_ms, getEpochTime(),
                configMgr.config.lat, configMgr.config.lon,
                ev.temperature, ev.pressure,
                ev.ml_prob, ev.ml_agree
            );
        }

        // SINKRONISASI OTOMATIS SAAT KEMBALI TERHUBUNG:
        // 1. Seketika saat transisi dari offline -> online terdeteksi (Immediate Reconnect Trigger)
        // 2. Berulang cepat setiap 5 detik selama masih ada antrean rekaman offline yang belum terkirim
        static bool last_net_connected = false;
        bool is_now_connected = networkMgr.isConnected();
        bool just_reconnected = (!last_net_connected && is_now_connected);
        last_net_connected = is_now_connected;

        static unsigned long last_sync_check = 0;
        if (is_now_connected && storageMgr.getUnsyncedCount() > 0) {
            if (just_reconnected || (millis() - last_sync_check >= 5000)) {
                last_sync_check = millis();
                int synced = storageMgr.syncOfflineRecords([](const char* payload) {
                    return networkMgr.publishOfflineRecord(payload);
                }, 25);
                if (synced > 0) {
                    char sync_log[96];
                    snprintf(sync_log, sizeof(sync_log), "[AUTO-SYNC] %d rekaman gempa offline berhasil dikirim ke server (%zu tersisa)",
                             synced, storageMgr.getUnsyncedCount());
                    networkMgr.publishLog(sync_log);
                }
            }
        }
        
        // Beri nafas untuk Watchdog Core 0 (delay 20ms juga membuat animasi breathing lebih smooth)
        vTaskDelay(pdMS_TO_TICKS(20)); 
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);
    delay(2000); // Tunggu Serial stabil
    
    Serial.println("\n\n==========================================");
    Serial.println(" Lindu.id - Life-Critical Node v2.0 ");
    Serial.println("==========================================");

    // 0. SEGERA tandai firmware sebagai VALID agar tidak di-rollback
    // Ini HARUS dilakukan sebelum WiFi agar OTA tidak loop
    esp_ota_img_states_t ota_state;
    if (esp_ota_get_state_partition(esp_ota_get_running_partition(), &ota_state) == ESP_OK) {
        if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
            Serial.println("[OTA] Firmware baru terdeteksi! Menandai sebagai VALID...");
            esp_ota_mark_app_valid_cancel_rollback();
            
            // Hapus blacklist karena berhasil boot
            Preferences tempPrefs;
            tempPrefs.begin("ota", false);
            tempPrefs.remove("failed_tag");
            tempPrefs.end();
        }
    }

    // 0.1 Inisialisasi NVS Preferences aktuator & preferensi lampu kamar tidur
    actPrefs.begin("act", false);
    is_valve_locked = actPrefs.getBool("valve_locked", false);
    bedroom_stealth_mode = actPrefs.getBool("stealth_led", false);

#if !ARDUINO_USB_CDC_ON_BOOT
    // Matikan paksa LED onboard GPIO 2 (LED biru DevKit WROOM) agar kamar tidur tidak silau
    pinMode(2, OUTPUT);
    digitalWrite(2, LOW);
#endif

    // 1. Muat Konfigurasi (EEPROM) & WiFi Captive Portal
    configMgr.begin();
    storageMgr.begin(); // Mount LittleFS untuk rekaman gempa offline & dataset ML
    
    // Nyalakan LED biru saat sedang setup WiFi
    pixels.begin();
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT); digitalWrite(BUZZER_PIN, BUZZER_OFF); // MATI (lihat definisi BUZZER_OFF per varian board)

#if USE_SERVO_VALVE
    // ESP32-S3 PWM Timer Allocation untuk Servo (Hanya alokasi, tidak di-enable)
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    myServo.setPeriodHertz(50);
    // Servo sengaja TIDAK di-attach di sini agar tidak auto-enable saat alat menyala
#else
    pinMode(RELAY_DOOR_PIN, OUTPUT);
    pinMode(RELAY_VALVE_PIN, OUTPUT);
    digitalWrite(RELAY_DOOR_PIN, is_door_locked ? RELAY_OFF : RELAY_ON);
    digitalWrite(RELAY_VALVE_PIN, is_valve_locked ? RELAY_OFF : RELAY_ON);
    // INPUT_PULLDOWN (bukan INPUT polos): kalau sensor PIR belum/tidak terpasang, GPIO
    // mengambang bisa kebaca HIGH terus oleh noise, membuat occupancy logic mengira
    // ada gerakan terus-menerus dan pintu tidak pernah terkunci. Pull-down memaksa
    // default LOW (tidak ada gerakan) saat tidak ada sensor yang aktif men-drive pin.
    pinMode(PIR_PIN, INPUT_PULLDOWN);
#endif

    pixels.setPixelColor(0, pixels.Color(0, 0, 40));
    pixels.show();
    
    Serial.println("[i] Memulai koneksi WiFi...");
    
    // RETRY LOGIC: Coba koneksi WiFi 3x sebelum masuk Captive Portal
    // Ini mengatasi masalah pasca-OTA reboot dimana router belum siap
    bool wifi_ok = false;
    for (int attempt = 1; attempt <= 3; attempt++) {
        Serial.printf("[WiFi] Percobaan %d/3...\n", attempt);
        WiFi.begin(); // Menggunakan memori NVS bawaan ESP32 // Gunakan kredensial dari EEPROM
        
        unsigned long start = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
            delay(500);
            Serial.print(".");
            pixels.setPixelColor(0, (millis() / 300) % 2 ? pixels.Color(0, 0, 40) : pixels.Color(0, 0, 0));
            pixels.show();
        }
        Serial.println();
        
        if (WiFi.status() == WL_CONNECTED) {
            wifi_ok = true;
            Serial.println("[OK] WiFi Terhubung via kredensial tersimpan!");
            networkMgr.publishLog("WiFi Terhubung (Kredensial Tersimpan).");
            break;
        }
        Serial.printf("[!] Gagal percobaan %d. Menunggu 3 detik...\n", attempt);
        delay(3000);
    }
    
    // Jika 3x retry gagal, baru buka Captive Portal sebagai fallback
    if (!wifi_ok) {
        Serial.println("[i] Retry habis. Membuka Captive Portal...");
        networkMgr.publishLog("Gagal WiFi 3x, Membuka Captive Portal.");
        if (!configMgr.startCaptivePortal()) {
            Serial.println("[!] Gagal connect WiFi. Alat akan restart...");
            delay(3000);
            ESP.restart();
        }
    }
    Serial.println("[OK] WiFi Terhubung.");

    // 2. Buat Antrean Pesan Lintas-Core (30 slot pesan @ ~48 byte)
    eventQueue = xQueueCreate(30, sizeof(SensorEvent));

    // 3. Pisahkan Tugas Berat ke Core 0
    xTaskCreatePinnedToCore(
        networkTaskCode,   // Fungsi
        "NetworkTask",     // Nama task
        12288,             // Stack 12KB
        NULL,              // Parameter
        1,                 // Prioritas 1
        &networkTaskHandle,// Handle
        0                  // Kunci ke Core 0
    );
    
    // 4. Inisialisasi Sensor di Core 1
    sensorMgr.begin(eventQueue);
    otaUpdater.begin();
}

void loop() {
    // --- TOMBOL BOOT (FACTORY RESET) ---
    static unsigned long boot_press_time = 0;
    if (digitalRead(BOOT_BUTTON_PIN) == LOW) {
        if (boot_press_time == 0) {
            boot_press_time = millis();
        } else if (millis() - boot_press_time > 5000) { // Tahan 5 detik
            Serial.println("\n[!] FACTORY RESET VIA TOMBOL BOOT DIMULAI!");
            digitalWrite(BUZZER_PIN, BUZZER_ON); // Bunyikan bel
            configMgr.resetConfig();
            Preferences prefs;
            prefs.begin("ota", false); prefs.clear(); prefs.end();
            delay(1000);
            ESP.restart();
        }
    } else {
        boot_press_time = 0;
    }

    // TASK CORE 1: Membaca Sensor (I2C) & Filter DSP (Real-time murni)
    sensorMgr.loop();  

    // Periksa perintah serial (misal untuk dump/export dataset gempa offline ke PC / Python ML)
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        cmd.toUpperCase();
        if (cmd == "DUMP" || cmd == "EXPORT" || cmd == "CSV") {
            storageMgr.dumpCSVToSerial();
        } else if (cmd == "JSON" || cmd == "JSONL") {
            storageMgr.dumpJSONToSerial();
        } else if (cmd == "STATUS" || cmd == "STORAGE") {
            Serial.printf("[STORAGE] Records: %zu (%zu belum sync), Sisa Flash: %u KB\n",
                          storageMgr.getRecordCount(), storageMgr.getUnsyncedCount(),
                          (unsigned int)(storageMgr.getFreeBytes() / 1024));
        } else if (cmd == "CLEAR" || cmd == "RESET_DATA") {
            storageMgr.clearRecords();
        }
    }
    
    // Beri sedikit nafas agar Watchdog Timer Core 1 tidak marah
    delay(1);
}
