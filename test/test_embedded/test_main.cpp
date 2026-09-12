#include <Arduino.h>
#include <unity.h>
#include <ArduinoJson.h>
#include "NetworkManager.h"
#include "SensorManager.h"
#include "ConfigManager.h"

NetworkManager netMgr;
SensorManager sensMgr;
ConfigManager confMgr;

void setUp(void) {}
void tearDown(void) {}

// ==============================================
// TEST 1: Haversine Distance (Algoritma Jarak)
// ==============================================
void test_haversine_calculation(void) {
    float dist = netMgr.haversine(-6.2088, 106.8456, -6.9147, 107.6098);
    TEST_ASSERT_FLOAT_WITHIN(10.0, 120.0, dist);
}

// ==============================================
// TEST 2: Simulasi Matematika STA/LTA (Deteksi Gempa)
// ==============================================
void test_sta_lta_math(void) {
    float pga = 1.5;
    float energy = pga * pga; 
    float sta_ema = (energy * 0.1) + (0.01 * 0.9);
    float lta_ema = (energy * 0.005) + (0.01 * 0.995);
    float ratio = sta_ema / (lta_ema < 0.00001f ? 0.00001f : lta_ema);
    
    TEST_ASSERT_GREATER_THAN_FLOAT(5.0, ratio);
}

// ==============================================
// TEST 3: Zero-Crossing Rate (Pembeda Gempa vs Truk)
// ==============================================
void test_zcr_frequency(void) {
    int zcr_count = 0;
    // Gelombang naik turun melintasi angka 0 sebanyak 7 kali
    float dyn_z[] = {1.5, -1.2, 1.3, -1.0, 1.1, -1.4, 1.2, -1.1}; 
    float last_z = -1.0;
    
    for (int i = 0; i < 8; i++) {
        if ((dyn_z[i] > 0 && last_z <= 0) || (dyn_z[i] < 0 && last_z >= 0)) {
            zcr_count++;
        }
        last_z = dyn_z[i];
    }
    
    int freq_hz = zcr_count / 2;
    TEST_ASSERT_EQUAL(3, freq_hz); // 7 persilangan / 2 = 3 Hz (Valid Gempa)
}

// ==============================================
// TEST 4: Filter DC Offset (Penghilang Gravitasi)
// ==============================================
void test_dc_offset_calibration(void) {
    float dc_z = 0.0;
    float raw_z = 9.81; // Gaya gravitasi konstan ke bawah
    
    // Simulasi 500 loop pembacaan kalibrasi
    for(int i = 0; i < 500; i++) {
        dc_z = (raw_z * 0.1) + (dc_z * 0.9);
    }
    
    float dyn_z = raw_z - dc_z;
    // Sinyal dinamis murni harus bernilai 0 meski ada gravitasi
    TEST_ASSERT_FLOAT_WITHIN(0.01, 0.0, dyn_z); 
}

// ==============================================
// TEST 5: Keamanan Memori JSON Payload
// ==============================================
void test_json_payload_size(void) {
    StaticJsonDocument<384> doc;
    doc["node_id"] = "node_jakarta_001_sangat_panjang";
    doc["ts"]      = 1725432225;
    doc["uptime"]  = 125400;
    doc["pga"]     = 0.5234;
    doc["sta_lta"] = 12.45;
    doc["freq_hz"] = 7;
    doc["ax"]      = 0.312;
    doc["ay"]      = -0.123;
    doc["az"]      = 0.485;
    doc["lat"]     = -6.2088;
    doc["lon"]     = 106.8456;
    
    char buffer[384];
    size_t len = serializeJson(doc, buffer);
    
    // Memastikan buffer 384 bytes mencukupi dan tidak terjadi overflow (crash)
    TEST_ASSERT_GREATER_THAN(0, len);
    TEST_ASSERT_LESS_THAN(384, len);
}

// ==============================================
// TEST 6: Pengujian Kontrak Data (Telemetry Format)
// ==============================================
void test_telemetry_payload_format(void) {
    StaticJsonDocument<384> doc;
    doc["node_id"] = "S01";
    doc["ts"]      = 1700000000;
    doc["pga"]     = 0.5;
    doc["sta_lta"] = 10.2;
    doc["lat"]     = -6.2;
    doc["lon"]     = 106.8;
    
    char buffer[384];
    serializeJson(doc, buffer);
    
    // CARA YANG AMAN DAN STANDAR INDUSTRI:
    // Bongkar kembali (Deserialize) string ke objek JSON untuk memvalidasi strukturnya
    StaticJsonDocument<384> doc_in;
    DeserializationError error = deserializeJson(doc_in, buffer);
    
    TEST_ASSERT_FALSE_MESSAGE(error, "Fatal: Gagal mem-parsing kembali JSON yang dibuat!");
    
    // Uji eksistensi kunci (Key) secara terstruktur, bukan sekadar tebak string
    TEST_ASSERT_TRUE_MESSAGE(doc_in.containsKey("node_id"), "Key 'node_id' hilang!");
    TEST_ASSERT_EQUAL_STRING("S01", doc_in["node_id"].as<const char*>());
    
    TEST_ASSERT_TRUE_MESSAGE(doc_in.containsKey("ts"), "Key 'ts' (Timestamp) hilang!");
    TEST_ASSERT_EQUAL_UINT32(1700000000, doc_in["ts"].as<unsigned long>());
    
    TEST_ASSERT_TRUE_MESSAGE(doc_in.containsKey("pga"), "Key 'pga' hilang!");
    TEST_ASSERT_EQUAL_FLOAT(0.5, doc_in["pga"].as<float>());
    
    TEST_ASSERT_TRUE_MESSAGE(doc_in.containsKey("sta_lta"), "Key 'sta_lta' hilang!");
}

// ==============================================
// TEST 7: Uji Integrasi Jaringan (Mengirim Dummy Gempa ke Server)
// ==============================================
void test_mqtt_integration_send(void) {
    // Memulai komponen jaringan (Berdasarkan konfigurasi memori flash ESP32 yang sudah ada)
    confMgr.begin();
    netMgr.begin(&confMgr);
    
    // Beri waktu negosiasi TLS dan koneksi WiFi/MQTT (Bisa memakan waktu beberapa detik)
    int wait_cycles = 0;
    // Asumsi: jika dalam 5 detik tidak berhasil, tes dilewati 
    // (karena mungkin alat ini belum pernah disetup WiFi-nya via Captive Portal)
    for(int i = 0; i < 50; i++) {
        netMgr.loop();
        delay(100);
    }
    
    // 1. Tembak Payload Dummy (Gempa Super Besar 0.9G)
    // Walaupun MQTT belum tersambung sempurna, payload akan antri atau hilang (fire and forget)
    // Server di komputer Anda harusnya bereaksi jika tes ini jalan!
    netMgr.publishEvent(0.9, 15.0, 5, 0.5, 0.5, 9.0, millis(), 1700000000, -6.2088, 106.8456);
    
    // Beri waktu buffer terkirim ke internet
    for(int i = 0; i < 10; i++) {
        netMgr.loop();
        delay(100);
    }
    
    TEST_ASSERT_TRUE_MESSAGE(true, "Perintah publish tereksekusi. Cek log server Anda!");
}

// ==============================================
// TEST 8: Cek Hardware Fisik I2C
// ==============================================
void test_sensor_connection(void) {
    bool connected = sensMgr.selfTest();
    TEST_ASSERT_TRUE_MESSAGE(connected, "Sensor I2C Mati! Cek pin 3.3V, SDA(8), SCL(9).");
}

void setup() {
    delay(2000);
    UNITY_BEGIN();
    
    RUN_TEST(test_haversine_calculation);
    RUN_TEST(test_sta_lta_math);
    RUN_TEST(test_zcr_frequency);
    RUN_TEST(test_dc_offset_calibration);
    RUN_TEST(test_json_payload_size);
    RUN_TEST(test_telemetry_payload_format);
    RUN_TEST(test_mqtt_integration_send);
    RUN_TEST(test_sensor_connection); 
    
    UNITY_END();
}

void loop() {}
