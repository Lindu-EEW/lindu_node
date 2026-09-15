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
#include <WiFi.h>

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
    #include <ESP32Servo.h>
    Servo myServo;
#else
    // ESP32 classic / WROOM (unit pengujian): valve & door-lock pakai relay 2-channel
    #define USE_SERVO_VALVE 0
    #define RGB_PIN 4
    #define BUZZER_PIN 14
    #define RELAY_DOOR_PIN 25   // Relay CH1 -> Solenoid Door Lock 12V
    #define RELAY_VALVE_PIN 26  // Relay CH2 -> Solenoid Water/Gas Valve 12V
    // Modul relay 2-channel (active LOW): LOW = relay ON (energized), HIGH = relay OFF
    #define RELAY_ON  LOW
    #define RELAY_OFF HIGH
#endif

ConfigManager configMgr;
NetworkManager networkMgr;
SensorManager sensorMgr;
Adafruit_NeoPixel pixels(1, RGB_PIN, NEO_GRB + NEO_KHZ800);

QueueHandle_t eventQueue;
unsigned long global_alarm_until = 0;
bool is_valve_locked = false;
Preferences actPrefs;
#if !ARDUINO_USB_CDC_ON_BOOT
bool is_door_locked = true; // Default: pintu terkunci (khusus unit ESP32 classic)
#endif
float local_latest_pga = 0.0;
unsigned long local_alarm_until = 0;
unsigned long identify_until = 0;
TaskHandle_t networkTaskHandle;

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
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT); digitalWrite(BUZZER_PIN, HIGH); // ACTIVE LOW: HIGH artinya MATI

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
#endif

    float breathAngle = 0;

    SensorEvent ev;
    
    while(1) {
        networkMgr.loop(); // Handle rutin MQTT
        
        // Heartbeat status setiap 10 detik
        static unsigned long last_status = 0;
        if (millis() - last_status >= 10000) {
            networkMgr.publishStatus("online", sensorMgr.sensor_ok, sensorMgr.getTiltAngle(), sensorMgr.getPose());
            last_status = millis();
        }
        
        // Cek status alarm global dan lokal
        bool is_global_alarm = (millis() < global_alarm_until && global_alarm_until > 0);
        bool is_local_alarm = (millis() < local_alarm_until && local_alarm_until > 0);
        
        // Animasi LED Cerdas (Sesuai Status Sensor & WiFi)
        if (WiFi.status() == WL_CONNECTED && networkMgr.isConnected()) {
            otaUpdater.loop();
            breathAngle += 0.05;
            if (breathAngle > 2 * PI) breathAngle -= 2 * PI;
            int brightness = (sin(breathAngle) + 1.0) * 20.0; 
            
            bool hw611_ok = sensorMgr.bme_ok || sensorMgr.bmp_ok;
            
#if USE_SERVO_VALVE
            // LOGIKA VALVE MANUAL RESET (Hanya nyalakan motor saat diperintah sistem)
            static bool last_valve_state = false;
            if (is_valve_locked != last_valve_state) {
                myServo.attach(SERVO_PIN, 500, 2400); // Sistem meng-enable motor

                if (is_valve_locked) {
                    myServo.write(90); // Sistem menggerakkan katup ke posisi Tutup (90)
                } else {
                    myServo.write(0);  // Sistem mereset katup ke posisi Buka (0)
                }

                vTaskDelay(pdMS_TO_TICKS(1000)); // Beri waktu 1 detik agar motor selesai berputar fisik
                myServo.detach(); // Sistem mematikan/melepas motor kembali (Hemat baterai & tidak memaksa)

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

            // LOGIKA RELAY DOOR LOCK (Solenoid Door Lock 12V)
            // Auto-unlock saat alarm gempa terkonfirmasi, atau dikontrol manual via MQTT
            static bool last_door_state = is_door_locked;
            if (is_global_alarm) {
                is_door_locked = false; // Paksa buka pintu untuk evakuasi
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
                digitalWrite(BUZZER_PIN, HIGH);
            } else if (is_global_alarm) {
                // KONFIRMASI GEMPA (DARI SERVER): Berkedip Merah Cepat (Strobo) & Buzzer Menyala
                if ((millis() / 100) % 2 == 0) {
                    pixels.setPixelColor(0, pixels.Color(255, 0, 0));
                    analogWrite(BUZZER_PIN, 128); // 50% Duty Cycle (Max Volume Tone untuk Speaker)
                } else {
                    pixels.setPixelColor(0, pixels.Color(0, 0, 0));
                    analogWrite(BUZZER_PIN, 255); // ACTIVE LOW: 100% Duty Cycle (HIGH) artinya MATI
                }
            } else if (is_local_alarm) {
                // DETEKSI GETARAN LOKAL (Menunggu Konfirmasi Node Lain): HANYA Berkedip Pink
                if ((millis() / 500) % 2 == 0) {
                    pixels.setPixelColor(0, pixels.Color(255, 20, 147)); // Hot Pink
                } else {
                    pixels.setPixelColor(0, pixels.Color(0, 0, 0));
                    digitalWrite(BUZZER_PIN, HIGH);
                }} else if (!sensorMgr.sensor_ok && !hw611_ok) {
                
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
        
        if (is_global_alarm || is_local_alarm) {
            is_buzzer_active = true;
        } else {
            if (is_buzzer_active) {
                pinMode(BUZZER_PIN, OUTPUT);
                digitalWrite(BUZZER_PIN, HIGH); // ACTIVE LOW: HIGH mematikan arus sepenuhnya
                is_buzzer_active = false;
            }
        }
        
        pixels.show();
        
        // Cek apakah ada data sensor di antrean (Non-Blocking)
        if (xQueueReceive(eventQueue, &ev, 0) == pdTRUE) {
            local_latest_pga = ev.pga;
            if (ev.pga > 0.12) {
                local_alarm_until = millis() + 5000; // Tahan warna pink selama 5 detik
                
                // ========== OFFLINE FAIL-SAFE (LONE WOLF MODE) ==========
                // Jika MQTT server mati DAN getaran AMAT SANGAT BRUTAL (PGA > 0.60G),
                // ESP32 mengambil alih kekuasaan mutlak: langsung membunyikan sirine,
                // mengunci katup gas, dan membuka pintu untuk evakuasi.
                // Ini adalah garis pertahanan terakhir saat infrastruktur internet runtuh.
                if (!networkMgr.isConnected() && ev.pga > 0.60) {
                    Serial.println("[!!!] LONE WOLF MODE: Server offline + PGA EKSTREM! Mengambil alih kendali!");
                    global_alarm_until = millis() + 15000; // Sirine merah 15 detik
                    is_valve_locked = true;
                    actPrefs.putBool("valve_locked", true);
#if !ARDUINO_USB_CDC_ON_BOOT
                    is_door_locked = false; // Buka pintu untuk evakuasi (Classic only)
#endif
                }
                
                // TICK Dinamis: Volume/Intensitas diwakili oleh durasi (Haptic Feedback)
                static unsigned long last_tick_time = 0;
                if (!is_global_alarm && (millis() - last_tick_time > 500)) { 
                    last_tick_time = millis();
                    
                    // Semakin besar getaran (PGA), semakin lama durasi beep-nya
                    int beep_duration = (int)(ev.pga * 40.0);
                    if (beep_duration < 5) beep_duration = 5;     // Getaran pelan = 5ms (Tik kecil)
                    if (beep_duration > 100) beep_duration = 100; // Getaran keras = 100ms (Bip panjang/keras)
                    
                    digitalWrite(BUZZER_PIN, LOW); // Active-Low ON
                    vTaskDelay(pdMS_TO_TICKS(beep_duration));
                    digitalWrite(BUZZER_PIN, HIGH); // Active-Low OFF
                }
            }
            networkMgr.publishEvent(
                ev.pga, ev.ratio, ev.freq_hz,
                ev.accel_x, ev.accel_y, ev.accel_z,
                ev.uptime_ms, getEpochTime(),
                configMgr.config.lat, configMgr.config.lon,
                ev.temperature, ev.pressure
            );
        }
        
        // Beri nafas untuk Watchdog Core 0 (delay 20ms juga membuat animasi breathing lebih smooth)
        vTaskDelay(pdMS_TO_TICKS(20)); 
    }
}

void setup() {
    Serial.begin(115200);
    delay(2000); // Tunggu Serial stabil
    
    Serial.println("\n\n==========================================");
    Serial.println(" Lindu.id - Life-Critical Node v2.0 ");
    Serial.println("==========================================");

    // 1. Muat Konfigurasi (EEPROM) & WiFi Captive Portal
    configMgr.begin();
    
    // Nyalakan LED biru saat sedang setup WiFi
    pixels.begin();
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT); digitalWrite(BUZZER_PIN, HIGH); // ACTIVE LOW: HIGH artinya MATI

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
#endif

    pixels.setPixelColor(0, pixels.Color(0, 0, 40));
    pixels.show();
    
    Serial.println("[i] Memulai koneksi WiFi...");
    
    // RETRY LOGIC: Coba koneksi WiFi 3x sebelum masuk Captive Portal
    // Ini mengatasi masalah pasca-OTA reboot dimana router belum siap
    bool wifi_ok = false;
    for (int attempt = 1; attempt <= 3; attempt++) {
        Serial.printf("[WiFi] Percobaan %d/3...\n", attempt);
        WiFi.begin(); // Gunakan kredensial tersimpan
        
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
            break;
        }
        Serial.printf("[!] Gagal percobaan %d. Menunggu 3 detik...\n", attempt);
        delay(3000);
    }
    
    // Jika 3x retry gagal, baru buka Captive Portal sebagai fallback
    if (!wifi_ok) {
        Serial.println("[i] Retry habis. Membuka Captive Portal...");
        if (!configMgr.startCaptivePortal()) {
            Serial.println("[!] Gagal connect WiFi. Alat akan restart...");
            delay(3000);
            ESP.restart();
        }
    }
    Serial.println("[OK] WiFi Terhubung.");

    // 2. Buat Antrean Pesan Lintas-Core (10 slot pesan @ ~40 byte)
    eventQueue = xQueueCreate(10, sizeof(SensorEvent));

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
    // TASK CORE 1: Membaca Sensor (I2C) & Filter DSP (Real-time murni)
    sensorMgr.loop();  
    
    // Beri sedikit nafas agar Watchdog Timer Core 1 tidak marah
    delay(1);
}
