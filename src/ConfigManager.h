#ifndef CONFIG_MANAGER_H
#define CONFIG_MANAGER_H

#include <Arduino.h>

// Struktur memori yang akan disimpan secara permanen di Flash NVS
struct LinduConfig {
    char mqtt_server[64];
    char ota_repo[64];
    char node_id[32];
    float lat;
    float lon;
    bool is_provisioned;
};

class ConfigManager {
public:
    LinduConfig config;
    
    void begin();
    void saveConfig();
    void loadConfig();
    void resetConfig(); // Hapus memori (Hard Reset)
    
    // Membuka halaman HTML Captive Portal di HP jika WiFi putus/pertama kali nyala
    bool startCaptivePortal(); 
};

#endif
