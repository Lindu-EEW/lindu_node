#ifndef OTA_UPDATER_H
#define OTA_UPDATER_H

#include <Arduino.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Update.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include "esp_ota_ops.h"
#include "esp_system.h"

#ifndef CURRENT_VERSION
#define CURRENT_VERSION "v1.2.0"
#endif

class OTAUpdater {
public:
    void begin();
    void loop();
    void checkForUpdate();
    void confirmWorking();
    String ota_status = "IDLE";

private:
    unsigned long _last_check;
    Preferences _prefs;
    String _failed_tag;

    bool performUpdate(const char* url, const char* tag);
};

extern OTAUpdater otaUpdater;
#endif
