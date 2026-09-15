#include "ConfigManager.h"
#include <WiFiManager.h>
#include <Preferences.h>

Preferences prefs;

void ConfigManager::begin() {
    prefs.begin("lindu", false);
    loadConfig();
}

void ConfigManager::loadConfig() {
    // ponytail: fallback ke MAC address jika belum ada ID
    String defaultId = "node_" + String((uint32_t)(ESP.getEfuseMac() >> 16), HEX);
    String id = prefs.getString("node_id", defaultId);
    strlcpy(config.node_id, id.c_str(), sizeof(config.node_id));
    
    config.lat = prefs.getFloat("lat", 0.0);
    config.lon = prefs.getFloat("lon", 0.0);
    
    String srv = prefs.getString("mqtt_srv", "192.168.68.105");
    strlcpy(config.mqtt_server, srv.c_str(), sizeof(config.mqtt_server));
    config.is_provisioned = (config.lat != 0.0);
    String repo = prefs.getString("ota_repo", "Lindu-EEW/lindu_node");
    strlcpy(config.ota_repo, repo.c_str(), sizeof(config.ota_repo));

}

void ConfigManager::saveConfig() {
    prefs.putString("node_id", config.node_id);
    prefs.putFloat("lat", config.lat);
    prefs.putFloat("lon", config.lon);
    prefs.putString("mqtt_srv", config.mqtt_server);
    prefs.putString("ota_repo", config.ota_repo);
}

void ConfigManager::resetConfig() {
    prefs.clear();
    WiFi.disconnect(true, true);
    ESP.restart();
}

bool ConfigManager::startCaptivePortal() {
    WiFi.mode(WIFI_STA);
    WiFiManager wm;
    
    // Custom HTML params
    char latStr[16]; dtostrf(config.lat, 4, 6, latStr);
    char lonStr[16]; dtostrf(config.lon, 4, 6, lonStr);
    
    WiFiManagerParameter custom_repo("repo", "GitHub OTA Repo", config.ota_repo, 64);
    wm.addParameter(&custom_repo);
    WiFiManagerParameter custom_mqtt("mqtt", "MQTT Broker IP/Domain", config.mqtt_server, 64);
    wm.addParameter(&custom_mqtt);
    WiFiManagerParameter custom_lat("lat", "Latitude", latStr, 16);
    WiFiManagerParameter custom_lon("lon", "Longitude", lonStr, 16);
    
    wm.addParameter(&custom_lat);
    wm.addParameter(&custom_lon);

    // ponytail: blocking 3 menit, jika gagal biarkan Watchdog mereset
    wm.setConfigPortalTimeout(180); 

    if (!wm.autoConnect(config.node_id)) {
        return false;
    }

    // Save params jika diupdate via portal
    config.lat = atof(custom_lat.getValue());
    config.lon = atof(custom_lon.getValue());
    strlcpy(config.mqtt_server, custom_mqtt.getValue(), sizeof(config.mqtt_server));
    strlcpy(config.ota_repo, custom_repo.getValue(), sizeof(config.ota_repo));
    saveConfig();

    return true;
}
