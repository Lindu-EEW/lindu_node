#pragma once

#include <Arduino.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <functional>

struct SeismicRecord {
    unsigned long sample_id;
    double epoch;
    unsigned long uptime_ms;
    float pga;
    float sta_lta;
    int freq_hz;
    float rms;
    float ax;
    float ay;
    float az;
    float duration_s;
    float p_to_s;
    float ml_prob;
    bool ml_agree;
    bool is_earthquake;
    bool is_offline;
    bool is_synced;
};

class StorageManager {
public:
    StorageManager();
    bool begin();

    // Rekam getaran gempa ke non-volatile flash (CSV & JSON Lines)
    bool recordEarthquake(
        double epoch, unsigned long uptime_ms,
        float pga, float sta_lta, int freq_hz, float rms,
        float ax, float ay, float az,
        float duration_s, float p_to_s,
        float ml_prob, bool ml_agree,
        bool is_earthquake, bool is_offline
    );

    // Ambil metrik penyimpanan
    size_t getRecordCount() const { return _record_count; }
    size_t getUnsyncedCount() const { return _unsynced_count; }
    size_t getTotalBytes();
    size_t getUsedBytes();
    size_t getFreeBytes();
    bool isReady() const { return _fs_ok; }

    // Export dataset langsung ke Serial (cocok untuk ditarik ke file CSV / Python ML)
    void dumpCSVToSerial();
    void dumpJSONToSerial();

    // Hapus dataset di flash
    bool clearRecords();

    // Sinkronisasi record offline ke server via MQTT saat koneksi pulih
    int syncOfflineRecords(std::function<bool(const char* json_payload)> publishCallback, int max_records = 20);

private:
    bool _fs_ok;
    size_t _record_count;
    size_t _unsynced_count;
    unsigned long _last_save_time;
    float _last_saved_pga;

    const char* _csv_path;
    const char* _jsonl_path;

    void ensureHeader();
    void countExistingRecords();
};

extern StorageManager storageMgr;
