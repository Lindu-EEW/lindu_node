#include "OTAUpdater.h"
#include <Adafruit_NeoPixel.h>
extern Adafruit_NeoPixel pixels;

OTAUpdater otaUpdater;

// GitHub API root cert (DigiCert High Assurance EV Root CA)
const char* rootCACertificate = \
"-----BEGIN CERTIFICATE-----\n" \
"MIIDxTCCAq2gAwIBAgIQAqxcJmoLQJuPC3nyrkYldzANBgkqhkiG9w0BAQUFADBs\n" \
"MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3\n" \
"d3cuZGlnaWNlcnQuY29tMSswKQYDVQQDEyJEaWdpQ2VydCBIaWdoIEFzc3VyYW5j\n" \
"ZSBFViBSb290IENBMB4XDTA2MTExMDAwMDAwMFoXDTMxMTExMDAwMDAwMFowbDEL\n" \
"MAkGA1UEBhMCVVMxFTATBgNVBAoTDERpZ2lDZXJ0IEluYzEZMBcGA1UECxMQd3d3\n" \
"LmRpZ2ljZXJ0LmNvbTErMCkGA1UEAxMiRGlnaUNlcnQgSGlnaCBBc3N1cmFuY2Ug\n" \
"RVYgUm9vdCBDQTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBAMbM5XPm\n" \
"+9S75S0tMqbf5YE/yc0lSbZxKsPVlDRnogocsF9ppkCxxLeyj9CYpKlBWTrT3JTW\n" \
"PNt0OKRKzE0lgvdKpVMSOO7zSW1xkX5jtqumX8OkhPhPYlG++MXs2ziS4wblCJEM\n" \
"xChBVfvLWokVfnHoNb9Ncgk9vjo4UFt3MRuNs8ckRZqnrG0AFFoEt7oT61EKmEFB\n" \
"Ik5lYYeBQVCmeVyJ3hlKV9Uu5l0cUyx+mM0aBhakaHPQNAQTXKFx01p8VdteZOE3\n" \
"hzBWBOURtCmAEvF5OYiiAhF8J2a3iLd48soKqDirCmTCv2ZdlYTBoSUeh10aUAsg\n" \
"EsxBu24LUTi4S8sCAwEAAaNjMGEwDgYDVR0PAQH/BAQDAgGGMA8GA1UdEwEB/wQF\n" \
"MAMBAf8wHQYDVR0OBBYEFLE+w2kD+L9HAdSYJhoIAu9jZCvDMB8GA1UdIwQYMBaA\n" \
"FLE+w2kD+L9HAdSYJhoIAu9jZCvDMA0GCSqGSIb3DQEBBQUAA4IBAQAcGgaX3Nec\n" \
"nzyIZgYIVyHbIUf4KmeqvxgydkAQV8GK83rZEWWONfqe/EW1ntlMMUu4kehDLI6z\n" \
"eM7b41N5cdblIZQB2lWHmiRk9opmzN6cN82oNLFpmyPInngiK3BD41VHMWEZ71jF\n" \
"hS9OMPagMRYjyOfiZRYzy78aG6A9+MpeizGLYAiJlQwGWQFJQ41HRxGLGv/cREcw\n" \
"nZAm50ydk+28n4RDrWjBfXnL2l9F8c8qZ4L1W4e/4s1+rQ8wT8E2sY0f+J1s3mK3\n" \
"T2h0jZf1N84D1KAnT1Z4u82u3pG/vI0Fp4X1zD8yI2b5a5C4Yh1+J3hD0l+X5H\n" \
"-----END CERTIFICATE-----\n";

void OTAUpdater::begin() {
    _prefs.begin("ota", false);
    _failed_tag = _prefs.getString("failed_tag", "");
    
    // Check if we just booted from a rollback
    esp_ota_img_states_t ota_state;
    if (esp_ota_get_state_partition(esp_ota_get_running_partition(), &ota_state) == ESP_OK) {
        if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
            Serial.println("[OTA] Firmware baru sukses di-boot! Menandai sebagai valid...");
            esp_ota_mark_app_valid_cancel_rollback();
            _prefs.remove("failed_tag"); // Hapus blacklist karena berhasil boot
        }
    }
}

void OTAUpdater::confirmWorking() {
    // If the app runs long enough without crashing, mark it valid.
    // In our case, begin() already marked it valid.
}

void OTAUpdater::loop() {
    // Check update every 24 hours (or at boot + 30s)
    if (_last_check == 0 && millis() > 30000) {
        checkForUpdate();
        _last_check = millis();
    } else if (millis() - _last_check > 43200000) { // 12 Jam
        checkForUpdate();
        _last_check = millis();
    }
}

void OTAUpdater::checkForUpdate() {
    if (WiFi.status() != WL_CONNECTED) return;
    
    ota_status = "CHECKING_GITHUB";
    Serial.println("[OTA] Mengecek versi terbaru di GitHub...");
    
    WiFiClientSecure client;
    client.setInsecure();
    HTTPClient http;
    
    String url = String("https://api.github.com/repos/") + REPO_OWNER + "/" + REPO_NAME + "/releases/latest";
    http.begin(client, url);
    int httpCode = http.GET();
    
    if (httpCode == 200) {
        String payload = http.getString();
        DynamicJsonDocument doc(8192);
        DeserializationError error = deserializeJson(doc, payload);
        
        if (!error) {
            String latest_tag = doc["tag_name"].as<String>();
            Serial.println("[OTA] Versi saat ini: " CURRENT_VERSION);
            Serial.println("[OTA] Versi terbaru: " + latest_tag);
            
            if (latest_tag != CURRENT_VERSION && latest_tag != "") {
                if (latest_tag == _failed_tag) {
                    Serial.println("[OTA] SKIPPED! Versi ini (" + latest_tag + ") pernah membuat sistem crash sebelumnya (Blacklisted).");
                    return;
                }
                
                String bin_url = "";
                JsonArray assets = doc["assets"];
                for (JsonVariant asset : assets) {
                    String name = asset["name"].as<String>();
                    if (name.endsWith(".bin")) {
                        bin_url = asset["browser_download_url"].as<String>();
                        break;
                    }
                }
                
                if (bin_url != "") {
                    Serial.println("[OTA] Ditemukan file .bin! Mengunduh dari: " + bin_url);
                    
                    if (performUpdate(bin_url.c_str(), latest_tag.c_str())) {
                        // Kita akan menandai failed_tag SETELAH download sukses tapi SEBELUM reboot. 
                        // Jika firmware baru gagal boot (crash), dia akan rollback dan blacklist ini tetap ada.
                        // Jika sukses boot, firmware baru akan menghapus blacklist ini di begin().
                        _prefs.putString("failed_tag", latest_tag);
                        ota_status = "UPDATE_SUCCESS_RESTARTING";
                        Serial.println("[OTA] Update selesai! Restarting...");
                        delay(1000);
                        ESP.restart();
                    } else {
                        ota_status = "ERROR_UPDATE_FAILED";
                        Serial.println("[OTA] Update gagal!");
                    }
                }
            } else {
                ota_status = "UP_TO_DATE";
                Serial.println("[OTA] Anda sudah menggunakan versi terbaru atau sama.");
            }
        } else {
            ota_status = "ERROR_JSON_PARSE";
            Serial.print("[OTA] JSON Parse Failed: ");
            Serial.println(error.c_str());
        }
    } else {
        ota_status = "ERROR_API_" + String(httpCode);
        Serial.printf("[OTA] Gagal menghubungi GitHub API. Kode: %d\n", httpCode);
    }
    http.end();
}

bool OTAUpdater::performUpdate(const char* url, const char* tag) {
    WiFiClientSecure client;
    client.setInsecure();
    HTTPClient http;
    
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    http.begin(client, url);
    int httpCode = http.GET();
    
    if (httpCode != 200) {
        ota_status = "ERROR_DOWNLOAD_" + String(httpCode);
        Serial.printf("[OTA] Gagal mengunduh firmware. Kode: %d\n", httpCode);
        return false;
    }
    
    int contentLength = http.getSize();
    bool canBegin = Update.begin(contentLength, U_FLASH);
    
    if (canBegin) {
        ota_status = "DOWNLOADING_v1.1.0";
        Serial.println("[OTA] Memulai penulisan ke memori Flash...");
        
        Update.onProgress([](size_t progress, size_t total) {
            static unsigned long last_blink = 0;
            if (millis() - last_blink > 100) { // Berkedip Cyan cepat tiap 100ms
                last_blink = millis();
                static bool toggle = false;
                toggle = !toggle;
                pixels.setPixelColor(0, toggle ? pixels.Color(255, 255, 0) : pixels.Color(0, 0, 0)); // Yellow = Updating
                pixels.show();
            }
            if (progress % (total / 10) == 0) {
                Serial.printf("[OTA] Progress: %u%%\n", (progress / (total / 100)));
            }
        });
        
        size_t written = Update.writeStream(http.getStream());
        if (written == contentLength) {
            Serial.println("[OTA] Penulisan selesai (100%).");
        } else {
            Serial.printf("[OTA] Penulisan gagal! Ditulis: %d/%d\n", written, contentLength);
            return false;
        }
        
        if (Update.end()) {
            if (Update.isFinished()) {
                Serial.println("[OTA] Update berhasil divalidasi!");
                
                // Konfigurasi Rollback
                esp_ota_set_boot_partition(esp_ota_get_next_update_partition(NULL));
                return true;
            }
        }
    }
    
    Serial.printf("[OTA] Error Update: %s\n", Update.errorString());
    return false;
}
