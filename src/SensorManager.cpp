#include "SensorManager.h"

extern bool is_valve_locked;

bool probeI2C(uint8_t address) {
  Wire1.setTimeOut(50); // Set timeout hanya untuk I2C Cuaca
  Wire1.beginTransmission(address);
  if (Wire1.endTransmission() == 0) {
    // Validasi ekstra: Coba baca 1 byte register ID (0xD0 untuk BMP/BME280)
    Wire1.beginTransmission(address);
    Wire1.write(0xD0);
    Wire1.endTransmission();
    if (Wire1.requestFrom(address, (uint8_t)1) == 1) {
      uint8_t id = Wire1.read();
      if (id == 0x58 || id == 0x60 || id == 0x56 || id == 0x57) { // ID valid BMP/BME
        return true;
      }
    }
  }
  return false;
}

void SensorManager::begin(QueueHandle_t queue) {
  _eventQueue = queue;

// Sensor gas MQ-2 sekarang diaktifkan untuk semua unit
  pinMode(PIN_GAS_MQ2, INPUT);
  _gas_boot_time = millis();

  Wire.begin(PIN_I2C_SEIS_SDA, PIN_I2C_SEIS_SCL);  // Akselerometer
  Wire1.begin(PIN_I2C_ATMO_SDA, PIN_I2C_ATMO_SCL); // Cuaca

  _bmp = new Adafruit_BMP280(&Wire1);

  sensor_ok = selfTest();
  if (sensor_ok) {
    _lsm6ds3.setAccelRange(LSM6DS_ACCEL_RANGE_8_G);
    _lsm6ds3.setAccelDataRate(LSM6DS_RATE_1_66K_HZ);
  }

  // PROBE I2C DULU UNTUK MENCEGAH HANG
  if (probeI2C(0x76) || probeI2C(0x77)) {
    bme_ok = _bme.begin(0x76, &Wire1) || _bme.begin(0x77, &Wire1);
    if (!bme_ok) {
      bmp_ok = _bmp->begin(0x76) || _bmp->begin(0x77);
    }
  } else {
    bme_ok = false;
    bmp_ok = false;
  }
}

bool SensorManager::selfTest() {
  return _lsm6ds3.begin_I2C(0x6A, &Wire) || _lsm6ds3.begin_I2C(0x6B, &Wire);
}

// Sensor gas MQ-2 diaktifkan lintas platform.
void SensorManager::readGasSensor() {
  if (millis() - _last_gas_read < 500) return;
  _last_gas_read = millis();

  gas_raw_value = analogRead(PIN_GAS_MQ2);

  gas_warming_up = (millis() - _gas_boot_time) < GAS_WARMUP_MS;
  if (gas_warming_up) {
    // Sensor belum stabil (heater MQ-2 belum panas) -> abaikan hasil baca sementara
    gas_leak_detected = false;
    return;
  }

  gas_leak_detected = gas_raw_value > GAS_LEAK_THRESHOLD;

  // FAIL-SAFE: Kebocoran gas terdeteksi -> paksa tutup valve, abaikan status manual
  if (gas_leak_detected) {
    is_valve_locked = true;
  }
}

void SensorManager::loop() {
readGasSensor();

  static unsigned long last_cuaca_check = 0;
  static int cuaca_retry_count = 0;

  if (!bme_ok && !bmp_ok && cuaca_retry_count < 3 && millis() - last_cuaca_check > 5000) {
    cuaca_retry_count++;
    if (probeI2C(0x76) || probeI2C(0x77)) {
      bme_ok = _bme.begin(0x76, &Wire1) || _bme.begin(0x77, &Wire1);
      if (!bme_ok) {
        bmp_ok = _bmp->begin(0x76) || _bmp->begin(0x77);
      }
    }
    last_cuaca_check = millis();
  }

  static unsigned long last_accel_check = 0;
  if (!sensor_ok && millis() - last_accel_check > 5000) {
    sensor_ok = selfTest();
    if (sensor_ok) {
      _lsm6ds3.setAccelRange(LSM6DS_ACCEL_RANGE_8_G);
      _lsm6ds3.setAccelDataRate(LSM6DS_RATE_1_66K_HZ);
    }
    last_accel_check = millis();
  }

  if (millis() - _last_read_time < 10)
    return;
  _last_read_time = millis();

  float dyn_x = 0;
  float dyn_y = 0;
  float dyn_z = 0;
  float pga = 0;
  float rms = 0;

  if (sensor_ok) {
    sensors_event_t accel, gyro, temp_accel;
    _lsm6ds3.getEvent(&accel, &gyro, &temp_accel);

    static bool is_first_read = true;
    if (is_first_read) {
      _dc_x = accel.acceleration.x;
      _dc_y = accel.acceleration.y;
      _dc_z = accel.acceleration.z;
      is_first_read = false;
      _is_calibrated = true;
    }

    float alpha_dc = 0.002;
    _dc_x = (accel.acceleration.x * alpha_dc) + (_dc_x * (1.0 - alpha_dc));
    _dc_y = (accel.acceleration.y * alpha_dc) + (_dc_y * (1.0 - alpha_dc));
    _dc_z = (accel.acceleration.z * alpha_dc) + (_dc_z * (1.0 - alpha_dc));

    if (_is_calibrated) {
      dyn_x = accel.acceleration.x - _dc_x;
      dyn_y = accel.acceleration.y - _dc_y;
      dyn_z = accel.acceleration.z - _dc_z;

      if ((dyn_z > 0 && _last_dyn_z <= 0) || (dyn_z < 0 && _last_dyn_z >= 0))
        _zcr_count++;
      _last_dyn_z = dyn_z;

      if (millis() - _zcr_timer >= 1000) {
        _current_hz = _zcr_count / 2;
        _zcr_count = 0;
        _zcr_timer = millis();
      }

      pga = sqrt(dyn_x * dyn_x + dyn_y * dyn_y + dyn_z * dyn_z) / 9.81;
      float energy = pga * pga;
      rms = sqrt((dyn_x * dyn_x + dyn_y * dyn_y + dyn_z * dyn_z) / 3.0) / 9.81;

      _sta_ema = (energy * 0.1) + (_sta_ema * 0.9);
      _lta_ema = (energy * 0.005) + (_lta_ema * 0.995);
    }
  }

  float safe_lta = _lta_ema < 0.00001f ? 0.00001f : _lta_ema;
  float ratio = _sta_ema / safe_lta;

  float temp = 0.0;
  float pres = 0.0;
  if (bme_ok) {
    temp = _bme.readTemperature();
    pres = _bme.readPressure() / 100.0F;
  } else if (bmp_ok) {
    temp = _bmp->readTemperature();
    pres = _bmp->readPressure() / 100.0F;
  }

  // SELALU KIRIM DATA SETIAP 1 DETIK
      // LOGIKA ZERO-DELAY
    bool is_earthquake_spike = (pga > 0.12); 
    
    if ( (is_earthquake_spike && millis() - _telemetry_last_send >= 100) || (millis() - _telemetry_last_send >= 1000) ) {
    SensorEvent ev = {pga,   ratio, _current_hz, rms,  dyn_x,
                      dyn_y, dyn_z, millis(),    temp, pres};
    if (is_earthquake_spike) {
            xQueueSendToFront(_eventQueue, &ev, 0);
        } else {
            xQueueSend(_eventQueue, &ev, 0);
        }
    _telemetry_last_send = millis();
  }
}

float SensorManager::getTiltAngle() {
  if (!_is_calibrated || !sensor_ok)
    return 0.0;
  float pitch = atan2(-_dc_x, sqrt(_dc_y * _dc_y + _dc_z * _dc_z)) * 180.0 / PI;
  return pitch;
}

String SensorManager::getPose() {
  if (!_is_calibrated || !sensor_ok)
    return "Calibrating";
  float pitch = abs(getTiltAngle());
  if (pitch < 15.0)
    return "Flat";
  if (pitch > 75.0 && pitch < 105.0)
    return "Wall";
  return "Tilted";
}
