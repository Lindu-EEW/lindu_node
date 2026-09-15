#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <Arduino.h>
#include <Adafruit_LSM6DS3.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BMP280.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

struct SensorEvent {
    float pga;
    float ratio;
    int   freq_hz;
    float rms;
    float accel_x;
    float accel_y;
    float accel_z;
    unsigned long uptime_ms;
    float temperature;
    float pressure;
};

#define GAS_LEAK_THRESHOLD 1800
#define GAS_WARMUP_MS 30000 // MQ-2 butuh waktu pemanasan sebelum pembacaan stabil

class SensorManager {
public:
    void begin(QueueHandle_t queue);
    void loop();

    bool selfTest();
    bool sensor_ok = false;
    bool bmp_ok = false;
    bool bme_ok = false;
    bool gas_leak_detected = false;
    bool gas_warming_up = true;
    int  gas_raw_value = 0;
    String getPose();
    float getTiltAngle();

private:
    void readGasSensor();
    unsigned long _last_gas_read = 0;
    unsigned long _gas_boot_time = 0;
    Adafruit_LSM6DS3 _lsm6ds3;
    Adafruit_BME280 _bme;
    Adafruit_BMP280* _bmp;
    
    QueueHandle_t _eventQueue;
    
    float _dc_x = 0.0;
    float _dc_y = 0.0;
    float _dc_z = 0.0;
    bool  _is_calibrated = false;
    
    int _zcr_count = 0;
    float _last_dyn_z = 0.0;
    unsigned long _zcr_timer = 0;
    int _current_hz = 0;

    float _sta_ema = 0.0;
    float _lta_ema = 0.0;
    
    unsigned long _last_read_time = 0;
    unsigned long _last_trigger_time = 0;

    bool  _telemetry_active = false;
    unsigned long _telemetry_last_send = 0;
    unsigned long _telemetry_calm_since = 0;
    static const unsigned long TELEMETRY_INTERVAL_MS = 1000;
    static const unsigned long TELEMETRY_COOLDOWN_MS = 10000;
};

#endif
