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
    config.is_provisioned = (config.lat != 0.0);
}

void ConfigManager::saveConfig() {
    prefs.putString("node_id", config.node_id);
    prefs.putFloat("lat", config.lat);
    prefs.putFloat("lon", config.lon);
}

void ConfigManager::resetConfig() {
    prefs.clear();
    WiFi.disconnect(true, true);
    ESP.restart();
}

bool ConfigManager::startCaptivePortal() {
    WiFiManager wm;
    
    // Custom HTML params
    char latStr[16]; dtostrf(config.lat, 4, 6, latStr);
    char lonStr[16]; dtostrf(config.lon, 4, 6, lonStr);
    
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
    saveConfig();

    return true;
}
