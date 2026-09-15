#include <Preferences.h>
#include "OTAUpdater.h"
#include "NetworkManager.h"

extern unsigned long global_alarm_until;
#include "SensorManager.h"
extern SensorManager sensorMgr;
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <math.h>

extern unsigned long identify_until;

WiFiClient espClient;
PubSubClient mqtt(espClient);
NetworkManager* instance = nullptr; 

void NetworkManager::begin(ConfigManager* configMgr) {
    _configMgr = configMgr;
    instance = this;
    
    // Ganti IP broker ke IP laptop user
    mqtt.setServer(_configMgr->config.mqtt_server, 1883);
    mqtt.setCallback(NetworkManager::mqttCallback);
}

void NetworkManager::loop() {
    if (!mqtt.connected()) {
        reconnectMQTT();
    }
    mqtt.loop();
}

void NetworkManager::reconnectMQTT() {
    if (millis() - _last_reconnect_attempt > 5000) {
        _last_reconnect_attempt = millis();
        
        String willTopic = "lindu/sensor/" + String(_configMgr->config.node_id) + "/status";
        String willPayload = "{\"status\":\"offline\",\"node_id\":\"" + String(_configMgr->config.node_id) + "\"}";
        
        if (mqtt.connect(_configMgr->config.node_id, willTopic.c_str(), 1, true, willPayload.c_str())) {
            Serial.println("TERHUBUNG KE MQTT!");
            
            String statusPayload = "{\"status\":\"online\",\"node_id\":\"" + String(_configMgr->config.node_id) + "\",\"lat\":" + String(_configMgr->config.lat, 4) + ",\"lon\":" + String(_configMgr->config.lon, 4) + ",\"pose\":\"" + sensorMgr.getPose() + "\",\"tilt_angle\":" + String(sensorMgr.getTiltAngle(), 1) + ",\"sensor_ok\":" + String(sensorMgr.sensor_ok ? "true" : "false") + "}";
            mqtt.publish(willTopic.c_str(), statusPayload.c_str(), true);
            
            mqtt.subscribe("lindu/actuator/cmd/all", 1);
        }
    }
}

void NetworkManager::publishEvent(float pga, float sta_lta, int freq_hz, float ax, float ay, float az, unsigned long uptime_ms, unsigned long epoch, float lat, float lon, float temp, float pres) {
    if (!mqtt.connected()) return;
    
    StaticJsonDocument<512> doc;
    doc["node_id"] = _configMgr->config.node_id;
    doc["ts"]      = epoch;       
    doc["lat"]     = lat;
    doc["lon"]     = lon;
    doc["pga"]     = pga;
    doc["sta_lta"] = sta_lta;
    doc["freq_hz"] = freq_hz;
    doc["ax"]      = ax;
    doc["ay"]      = ay;
    doc["az"]      = az;
    doc["valve_status"] = is_valve_locked ? "DISABLED" : "ENABLED";
    doc["door_status"] = is_door_locked ? "LOCKED" : "UNLOCKED";
    doc["gas_alert"] = sensorMgr.gas_leak_detected;
    doc["temperature"] = temp;
    doc["pressure"] = pres;
    
    // Data Cuaca (Hanya diisi jika valid)
    if (temp > 0.0) doc["temperature"] = temp;
    if (pres > 0.0) doc["pressure"] = pres;

    char buffer[512];
    serializeJson(doc, buffer);
    
    String topic = "lindu/sensor/" + String(_configMgr->config.node_id) + "/telemetry";
    mqtt.publish(topic.c_str(), buffer);
}

float NetworkManager::haversine(float lat1, float lon1, float lat2, float lon2) {
    float dLat = (lat2 - lat1) * PI / 180.0;
    float dLon = (lon2 - lon1) * PI / 180.0;
    lat1 = lat1 * PI / 180.0;
    lat2 = lat2 * PI / 180.0;
    float a = pow(sin(dLat / 2), 2) + pow(sin(dLon / 2), 2) * cos(lat1) * cos(lat2);
    float c = 2 * asin(sqrt(a));
    return 6371.0 * c;
}

void NetworkManager::mqttCallback(char* topic, byte* payload, unsigned int length) {
    String msg;
    for (int i = 0; i < length; i++) msg += (char)payload[i];
    
    if (String(topic) == "lindu/actuator/cmd/all") {
        StaticJsonDocument<256> doc;
        DeserializationError error = deserializeJson(doc, msg);
        if (error) return;
        
        if (doc["cmd"] == "trigger_siren") {
            float e_lat = doc["epicenter_lat"];
            float e_lon = doc["epicenter_lon"];
            
            float dist = instance->haversine(
                instance->_configMgr->config.lat, 
                instance->_configMgr->config.lon, 
                e_lat, e_lon
            );
            
            bool is_bypass = doc["bypass"] | false;
            
            if (dist < 50.0 || is_bypass) {
                Serial.println("[!] SIRINE MENYALA! Valve Dikunci Tutup, Pintu Dibuka untuk Evakuasi!");
                global_alarm_until = millis() + 15000;
                is_valve_locked = true;
                is_door_locked = false;
            } else {
                Serial.println("[i] Epicenter terlalu jauh. Abaikan.");
            }
        } else if (doc["cmd"] == "enable_valve") {
            String target = doc["target_node"] | "all";
            String my_id = String(instance->_configMgr->config.node_id);
            if (target == "all" || target == my_id) {
                Serial.println("[i] Perintah Sistem: Valve di-ENABLE (Aliran Dibuka).");
                is_valve_locked = false;
            } else {
                Serial.println("[i] Perintah Enable diabaikan (Bukan untuk Node ini).");
            }
        } else if (doc["cmd"] == "disable_valve") {
            String target = doc["target_node"] | "all";
            String my_id = String(instance->_configMgr->config.node_id);
            if (target == "all" || target == my_id) {
                Serial.println("[i] Perintah Sistem: Valve di-DISABLE (Aliran Ditutup).");
                is_valve_locked = true;
            } else {
                Serial.println("[i] Perintah Disable diabaikan (Bukan untuk Node ini).");
            }
        } else if (doc["cmd"] == "lock_door") {
            String target = doc["target_node"] | "all";
            String my_id = String(instance->_configMgr->config.node_id);
            if (target == "all" || target == my_id) {
                Serial.println("[i] Perintah Sistem: Pintu di-LOCK.");
                is_door_locked = true;
            } else {
                Serial.println("[i] Perintah Lock Door diabaikan (Bukan untuk Node ini).");
            }
        } else if (doc["cmd"] == "unlock_door") {
            String target = doc["target_node"] | "all";
            String my_id = String(instance->_configMgr->config.node_id);
            if (target == "all" || target == my_id) {
                Serial.println("[i] Perintah Sistem: Pintu di-UNLOCK.");
                is_door_locked = false;
            } else {
                Serial.println("[i] Perintah Unlock Door diabaikan (Bukan untuk Node ini).");
            }
        } else if (doc["cmd"] == "identify") {
            String target = doc["target_node"] | "all";
            String my_id = String(instance->_configMgr->config.node_id);
            if (target == "all" || target == my_id) {
                identify_until = millis() + 10000; // 10 detik
                Serial.println("[i] Perintah Sistem: IDENTIFY. Lampu berkedip putih.");
            }
        } else if (doc["cmd"] == "set_location") {
            String target = doc["target_node"] | "all";
            String my_id = String(instance->_configMgr->config.node_id);
            if (target == "all" || target == my_id) {
                if (doc.containsKey("lat") && doc.containsKey("lon")) {
                    instance->_configMgr->config.lat = doc["lat"];
                    instance->_configMgr->config.lon = doc["lon"];
                    instance->_configMgr->saveConfig();
                    Serial.println("[i] Perintah Sistem: SET LOCATION. Koordinat diperbarui!");
                    instance->forcePublishStatus(); // Segera kirim update ke dashboard
                }
            }
        } else if (doc["cmd"] == "factory_reset") {
            String target = doc["target_node"] | "all";
            String my_id = String(instance->_configMgr->config.node_id);
            if (target == "all" || target == my_id) {
                Serial.println("[!] Perintah Sistem: FACTORY RESET. Menghapus semua memori dan Restart...");
                instance->_configMgr->resetConfig(); // Hapus WiFi & Koordinat
                Preferences prefs;
                prefs.begin("ota", false);
                prefs.clear(); // Hapus blacklist OTA
                prefs.end();
                delay(1000);
                ESP.restart();
            }
        } else if (doc["cmd"] == "set_broker") {
            String target = doc["target_node"] | "all";
            String my_id = String(instance->_configMgr->config.node_id);
            if (target == "all" || target == my_id) {
                if (doc.containsKey("server")) {
                    strlcpy(instance->_configMgr->config.mqtt_server, (const char*)doc["server"], 64);
                    instance->_configMgr->saveConfig();
                    Serial.println("[i] Perintah Sistem: SET BROKER. Restarting ESP32...");
                    delay(1000);
                    ESP.restart();
                }
            }
        } else if (doc["cmd"] == "force_update" || doc["cmd"] == "reboot") {
            String target = doc["target_node"] | "all";
            String my_id = String(instance->_configMgr->config.node_id);
            if (target == "all" || target == my_id) {
                Serial.println("[i] Perintah Sistem: FORCE UPDATE. Menghapus Blacklist dan Restarting ESP32...");
                Preferences prefs;
                prefs.begin("ota", false);
                prefs.remove("failed_tag");
                prefs.end();
                delay(1000);
                ESP.restart();
            }
        }
    }
}

bool NetworkManager::isConnected() {
    return mqtt.connected();
}

void NetworkManager::forcePublishStatus() {
    publishStatus("online", true, 0.0, "FLAT");
}

void NetworkManager::publishStatus(String status, bool sensor_ok, float tilt_angle, String pose) {
    if (!mqtt.connected()) return;
    String willTopic = "lindu/sensor/" + String(_configMgr->config.node_id) + "/status";
    String statusPayload = "{\"status\":\"" + status + "\",\"node_id\":\"" + String(_configMgr->config.node_id) + "\",\"lat\":" + String(_configMgr->config.lat, 4) + ",\"lon\":" + String(_configMgr->config.lon, 4) + ",\"pose\":\"" + pose + "\",\"tilt_angle\":" + String(tilt_angle, 1) + ",\"sensor_ok\":" + String(sensor_ok ? "true" : "false") + ",\"fw_version\":\"" + String(CURRENT_VERSION) + "\",\"ota_status\":\"" + otaUpdater.ota_status + "\"}";
    mqtt.publish(willTopic.c_str(), statusPayload.c_str(), true);
}
