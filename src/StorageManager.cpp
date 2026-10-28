#include "StorageManager.h"

StorageManager storageMgr;

StorageManager::StorageManager()
    : _fs_ok(false),
      _record_count(0),
      _unsynced_count(0),
      _last_save_time(0),
      _last_saved_pga(0.0f),
      _csv_path("/earthquakes.csv"),
      _jsonl_path("/earthquakes.jsonl") {}

bool StorageManager::begin() {
    // Mount LittleFS partition (menggunakan partisi 'spiffs' di partition table)
    // formatOnFail = true -> otomatis format jika pertama kali dipakai atau corrupt
    if (!LittleFS.begin(true)) {
        Serial.println("[STORAGE] GAGAL mount LittleFS flash storage!");
        _fs_ok = false;
        return false;
    }

    _fs_ok = true;
    size_t total = LittleFS.totalBytes();
    size_t used = LittleFS.usedBytes();
    size_t free_b = total - used;

    Serial.printf("[STORAGE] LittleFS Aktif! Total: %u KB, Terpakai: %u KB, Sisa: %u KB\n",
                  (unsigned int)(total / 1024),
                  (unsigned int)(used / 1024),
                  (unsigned int)(free_b / 1024));

    ensureHeader();
    countExistingRecords();

    Serial.printf("[STORAGE] Riwayat gempa tersimpan di flash: %zu record (%zu belum sync ke server)\n",
                  _record_count, _unsynced_count);

    return true;
}

void StorageManager::ensureHeader() {
    if (!_fs_ok) return;

    if (!LittleFS.exists(_csv_path)) {
        File f = LittleFS.open(_csv_path, FILE_WRITE);
        if (f) {
            // Header kompatibel langsung dengan dataset ML Lindu & Pandas dataframe
            f.println("sample_id,epoch,uptime_ms,pga,sta_lta,freq_hz,rms,ax,ay,az,duration_s,p_to_s,ml_prob,ml_agree,is_earthquake,is_offline");
            f.close();
            Serial.println("[STORAGE] File dataset baru /earthquakes.csv dibuat.");
        }
    }
}

void StorageManager::countExistingRecords() {
    if (!_fs_ok) return;

    _record_count = 0;
    _unsynced_count = 0;

    if (LittleFS.exists(_csv_path)) {
        File f = LittleFS.open(_csv_path, FILE_READ);
        if (f) {
            bool is_first = true;
            while (f.available()) {
                String line = f.readStringUntil('\n');
                if (is_first) {
                    is_first = false; // Lewati baris header CSV
                    continue;
                }
                if (line.length() > 5) {
                    _record_count++;
                }
            }
            f.close();
        }
    }

    if (LittleFS.exists(_jsonl_path)) {
        File f = LittleFS.open(_jsonl_path, FILE_READ);
        if (f) {
            while (f.available()) {
                String line = f.readStringUntil('\n');
                if (line.indexOf("\"sync\":0") >= 0 || line.indexOf("\"sync\": 0") >= 0) {
                    _unsynced_count++;
                }
            }
            f.close();
        }
    }
}

bool StorageManager::recordEarthquake(
    double epoch, unsigned long uptime_ms,
    float pga, float sta_lta, int freq_hz, float rms,
    float ax, float ay, float az,
    float duration_s, float p_to_s,
    float ml_prob, bool ml_agree,
    bool is_earthquake, bool is_offline
) {
    if (!_fs_ok) {
        if (!begin()) return false;
    }

    // Rate-limit penulisan flash:
    // Selama getaran gempa berlangsung, simpan setiap 200ms (5 Hz resolusi tinggi)
    // ATAU setiap kali PGA melonjak membentuk puncak baru (+5% dari puncak sebelumnya).
    unsigned long now = millis();
    bool is_new_peak = (pga > _last_saved_pga * 1.05f && pga > 0.005f);
    if ((now - _last_save_time < 200) && !is_new_peak) {
        return false;
    }
    _last_save_time = now;
    _last_saved_pga = pga;

    _record_count++;
    // Seluruh rekaman gempa selalu dicatat untuk sinkronisasi otomatis agar server terjamin utuh
    _unsynced_count++;

    // 1. Simpan baris ke CSV (untuk ditarik langsung ke scikit-learn / TensorFlow / TinyML)
    File f_csv = LittleFS.open(_csv_path, FILE_APPEND);
    if (f_csv) {
        f_csv.printf("%zu,%.3f,%lu,%.5f,%.3f,%d,%.5f,%.4f,%.4f,%.4f,%.2f,%.2f,%.4f,%d,%d,%d\n",
                     _record_count,
                     epoch,
                     uptime_ms,
                     pga,
                     sta_lta,
                     freq_hz,
                     rms,
                     ax,
                     ay,
                     az,
                     duration_s,
                     p_to_s,
                     ml_prob,
                     ml_agree ? 1 : 0,
                     is_earthquake ? 1 : 0,
                     is_offline ? 1 : 0);
        f_csv.close();
    }

    // 2. Simpan baris ke JSONL (untuk sinkronisasi MQTT store-and-forward)
    File f_json = LittleFS.open(_jsonl_path, FILE_APPEND);
    if (f_json) {
        f_json.printf("{\"id\":%zu,\"ts\":%.3f,\"uptime\":%lu,\"pga\":%.5f,\"sta_lta\":%.3f,\"freq\":%d,\"rms\":%.5f,\"ax\":%.4f,\"ay\":%.4f,\"az\":%.4f,\"dur\":%.2f,\"p2s\":%.2f,\"ml\":%.4f,\"agree\":%d,\"quake\":%d,\"off\":%d,\"sync\":0}\n",
                      _record_count,
                      epoch,
                      uptime_ms,
                      pga,
                      sta_lta,
                      freq_hz,
                      rms,
                      ax,
                      ay,
                      az,
                      duration_s,
                      p_to_s,
                      ml_prob,
                      ml_agree ? 1 : 0,
                      is_earthquake ? 1 : 0,
                      is_offline ? 1 : 0);
        f_json.close();
    }

    Serial.printf("[STORAGE] Rekaman gempa disimpan di flash! [#%zu | PGA: %.4fG | STA/LTA: %.2f | Freq: %dHz | ML: %.1f%% | Offline: %s]\n",
                  _record_count,
                  pga,
                  sta_lta,
                  freq_hz,
                  ml_prob * 100.0f,
                  is_offline ? "YA (TERISOLIR)" : "TIDAK (ONLINE)");

    return true;
}

size_t StorageManager::getTotalBytes() {
    if (!_fs_ok) return 0;
    return LittleFS.totalBytes();
}

size_t StorageManager::getUsedBytes() {
    if (!_fs_ok) return 0;
    return LittleFS.usedBytes();
}

size_t StorageManager::getFreeBytes() {
    if (!_fs_ok) return 0;
    size_t total = LittleFS.totalBytes();
    size_t used = LittleFS.usedBytes();
    return (total > used) ? (total - used) : 0;
}

void StorageManager::dumpCSVToSerial() {
    if (!_fs_ok) {
        Serial.println("[STORAGE ERROR] LittleFS tidak siap!");
        return;
    }

    if (!LittleFS.exists(_csv_path)) {
        Serial.println("[STORAGE] Belum ada rekaman gempa di /earthquakes.csv");
        return;
    }

    File f = LittleFS.open(_csv_path, FILE_READ);
    if (!f) {
        Serial.println("[STORAGE ERROR] Gagal membuka /earthquakes.csv");
        return;
    }

    Serial.println("\n========== [BEGIN EARTHQUAKE DATASET CSV] ==========");
    uint8_t buf[256];
    while (f.available()) {
        size_t len = f.read(buf, sizeof(buf));
        Serial.write(buf, len);
    }
    f.close();
    Serial.println("\n========== [END EARTHQUAKE DATASET CSV] ==========\n");
}

void StorageManager::dumpJSONToSerial() {
    if (!_fs_ok) {
        Serial.println("[STORAGE ERROR] LittleFS tidak siap!");
        return;
    }

    if (!LittleFS.exists(_jsonl_path)) {
        Serial.println("[STORAGE] Belum ada rekaman gempa di /earthquakes.jsonl");
        return;
    }

    File f = LittleFS.open(_jsonl_path, FILE_READ);
    if (!f) {
        Serial.println("[STORAGE ERROR] Gagal membuka /earthquakes.jsonl");
        return;
    }

    Serial.println("\n========== [BEGIN EARTHQUAKE DATASET JSONL] ==========");
    uint8_t buf[256];
    while (f.available()) {
        size_t len = f.read(buf, sizeof(buf));
        Serial.write(buf, len);
    }
    f.close();
    Serial.println("\n========== [END EARTHQUAKE DATASET JSONL] ==========\n");
}

bool StorageManager::clearRecords() {
    if (!_fs_ok) return false;

    if (LittleFS.exists(_csv_path)) LittleFS.remove(_csv_path);
    if (LittleFS.exists(_jsonl_path)) LittleFS.remove(_jsonl_path);

    _record_count = 0;
    _unsynced_count = 0;
    _last_save_time = 0;
    _last_saved_pga = 0.0f;

    ensureHeader();
    Serial.println("[STORAGE] Semua rekaman gempa di flash telah dibersihkan!");
    return true;
}

int StorageManager::syncOfflineRecords(std::function<bool(const char* json_payload)> publishCallback, int max_records, bool force_all) {
    if (!_fs_ok) return 0;
    if (!force_all && _unsynced_count == 0) return 0;
    if (!LittleFS.exists(_jsonl_path)) return 0;

    File f_in = LittleFS.open(_jsonl_path, FILE_READ);
    if (!f_in) return 0;

    String temp_path = "/earthquakes.tmp";
    File f_out = LittleFS.open(temp_path.c_str(), FILE_WRITE);
    if (!f_out) {
        f_in.close();
        return 0;
    }

    int synced_now = 0;

    while (f_in.available()) {
        String line = f_in.readStringUntil('\n');
        line.trim();
        if (line.length() < 5) continue;

        if (synced_now < max_records && (force_all || line.indexOf("\"sync\":0") >= 0 || line.indexOf("\"sync\": 0") >= 0)) {
            // Coba kirim via MQTT
            if (publishCallback(line.c_str())) {
                line.replace("\"sync\":0", "\"sync\":1");
                line.replace("\"sync\": 0", "\"sync\":1");
                synced_now++;
                if (_unsynced_count > 0) _unsynced_count--;
            }
        }
        f_out.println(line);
    }

    f_in.close();
    f_out.close();

    // Ganti file JSONL lama dengan yang sudah ter-update status sync-nya
    LittleFS.remove(_jsonl_path);
    LittleFS.rename(temp_path.c_str(), _jsonl_path);

    if (synced_now > 0) {
        Serial.printf("[STORAGE SYNC] Berhasil mengirim %d rekaman offline ke server. (Tersisa belum sync: %zu)\n",
                      synced_now, _unsynced_count);
    }

    return synced_now;
}
