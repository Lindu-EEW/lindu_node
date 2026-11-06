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
    int syncOfflineRecords(std::function<bool(const char* json_payload)> publishCallback, int max_records = 20, bool force_all = false);

private:
    bool _fs_ok;
    size_t _record_count;
    size_t _unsynced_count;
    unsigned long _last_save_time;
    float _last_saved_pga;

    const char* _csv_path;
    const char* _jsonl_path;

    // Batas ukuran file agar partisi LittleFS (128KB) tidak pernah penuh.
    // Anggaran terburuk: JSONL 32K + CSV 24K + salinan sementara <= 32K = 88K < 128K.
    static constexpr size_t JSONL_MAX_BYTES = 32 * 1024;
    static constexpr size_t CSV_MAX_BYTES   = 24 * 1024;
    // Sisa ruang minimum yang harus tersedia sebelum menulis / menyalin file.
    static constexpr size_t MIN_FREE_BYTES  = 8 * 1024;

    void ensureHeader();
    void countExistingRecords();

    // Jika file melebihi max_bytes, buang rekaman terlama (simpan separuh terbaru).
    void trimFile(const char* path, size_t max_bytes, bool keep_header);
};

extern StorageManager storageMgr;
