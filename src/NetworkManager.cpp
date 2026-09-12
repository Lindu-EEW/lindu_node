#include "NetworkManager.h"
#include "SensorManager.h"
extern SensorManager sensorMgr;
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <math.h>

WiFiClient espClient;
PubSubClient mqtt(espClient);
NetworkManager* instance = nullptr; 

void NetworkManager::begin(ConfigManager* configMgr) {
    _configMgr = configMgr;
    instance = this;
    
    // Ganti IP broker ke IP laptop user
    mqtt.setServer("Likos-MacBook-Air.local", 1883);
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
            
            if (dist < 50.0) {
                Serial.println("[!] SIRINE MENYALA! Epicenter berjarak < 50km.");
            }
        }
    }
}
