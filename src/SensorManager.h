#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <Arduino.h>
#include <Adafruit_LSM6DS3.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BMP280.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

enum AccelType {
    ACCEL_NONE,
    ACCEL_LSM6DS3,
    ACCEL_MPU6050,
    ACCEL_MPU_DIRECT
};

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
    float ml_prob;
    bool  ml_agree;
};

#define GAS_LEAK_THRESHOLD 2000
#define GAS_WARMUP_MS 30000 // MQ-2 butuh waktu pemanasan sebelum pembacaan stabil

class SensorManager {
public:
    void begin(QueueHandle_t queue);
    void loop();

    bool selfTest();
    bool sensor_ok = false;
    AccelType accel_type = ACCEL_NONE;
    bool bmp_ok = false;
    bool bme_ok = false;
    bool gas_leak_detected = false;
    bool gas_warming_up = true;
    int  gas_raw_value = 0;
    String getPose();
    float getTiltAngle();
    float getNoiseFloor() const { return _noise_floor_ema; }
    bool isCalibrated() const { return _is_calibrated; }

private:
    void readGasSensor();
    unsigned long _last_gas_read = 0;
    unsigned long _gas_boot_time = 0;
    Adafruit_LSM6DS3 _lsm6ds3;
    Adafruit_MPU6050 _mpu6050;
    uint8_t _mpu_addr = 0x68;
    Adafruit_BME280 _bme;
    Adafruit_BMP280* _bmp;
    
    QueueHandle_t _eventQueue;
    
    float _dc_x = 0.0;
    float _dc_y = 0.0;
    float _dc_z = 0.0;
    bool  _is_calibrated = false;
    
    // Kalibrasi bertahap multi-sampel saat boot (300 sampel @ 100Hz = 3.0 detik)
    static const int CALIBRATION_SAMPLE_TARGET = 300;
    int   _calib_samples_count = 0;
    float _calib_sum_x = 0.0f;
    float _calib_sum_y = 0.0f;
    float _calib_sum_z = 0.0f;
    float _noise_floor_ema = 0.005f;
    
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
