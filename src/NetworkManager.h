#ifndef NETWORK_MANAGER_H
#define NETWORK_MANAGER_H

#include <Arduino.h>
#include "ConfigManager.h"

class NetworkManager {
public:
    void begin(ConfigManager* configMgr);
    void loop(); 
    void publishEvent(float pga, float sta_lta, int freq_hz, float ax, float ay, float az, unsigned long uptime_ms, unsigned long epoch, float lat, float lon, float temp, float pres);
    
private:
    ConfigManager* _configMgr;
    
    void reconnectMQTT();
    
    unsigned long _last_reconnect_attempt = 0; 
    
    float haversine(float lat1, float lon1, float lat2, float lon2);
    static void mqttCallback(char* topic, byte* payload, unsigned int length);
};

#endif
