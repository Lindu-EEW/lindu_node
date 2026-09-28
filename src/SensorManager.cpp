#include "SensorManager.h"
#include "NetworkManager.h"

extern NetworkManager networkMgr;
extern bool is_valve_locked;

static String scanI2CBus(TwoWire &bus) {
  String res = "";
  for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
    bus.beginTransmission(addr);
    if (bus.endTransmission() == 0) {
      if (res.length() > 0) res += ", ";
      char buf[8];
      snprintf(buf, sizeof(buf), "0x%02X", addr);
      res += buf;
    }
  }
  return res.length() > 0 ? res : "NONE";
}

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

  Wire.setPins(PIN_I2C_SEIS_SDA, PIN_I2C_SEIS_SCL);
  Wire.begin(PIN_I2C_SEIS_SDA, PIN_I2C_SEIS_SCL);  // Akselerometer
  Wire.setTimeOut(50);

  Wire1.setPins(PIN_I2C_ATMO_SDA, PIN_I2C_ATMO_SCL);
  Wire1.begin(PIN_I2C_ATMO_SDA, PIN_I2C_ATMO_SCL); // Cuaca
  Wire1.setTimeOut(50);

  _bmp = new Adafruit_BMP280(&Wire1);

  sensor_ok = selfTest();
  if (sensor_ok) {
    if (accel_type == ACCEL_LSM6DS3) {
      _lsm6ds3.setAccelRange(LSM6DS_ACCEL_RANGE_8_G);
      _lsm6ds3.setAccelDataRate(LSM6DS_RATE_1_66K_HZ);
    } else if (accel_type == ACCEL_MPU6050) {
      _mpu6050.setAccelerometerRange(MPU6050_RANGE_8_G);
      _mpu6050.setFilterBandwidth(MPU6050_BAND_44_HZ);
    }
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
  if (_lsm6ds3.begin_I2C(0x6A, &Wire) || _lsm6ds3.begin_I2C(0x6B, &Wire)) {
    accel_type = ACCEL_LSM6DS3;
    return true;
  }
  if (_mpu6050.begin(0x68, &Wire)) {
    _mpu_addr = 0x68;
    accel_type = ACCEL_MPU6050;
    return true;
  }
  if (_mpu6050.begin(0x69, &Wire)) {
    _mpu_addr = 0x69;
    accel_type = ACCEL_MPU6050;
    return true;
  }

  // Fallback: Direct I2C driver untuk MPU-6050 / MPU-6500 / MPU-9250 / clone
  // Mengatasi pembatasan WHO_AM_I == 0x68 bawaan Adafruit MPU6050 library
  for (uint8_t addr : { (uint8_t)0x68, (uint8_t)0x69 }) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Wire.beginTransmission(addr);
      Wire.write(0x75); // WHO_AM_I register
      if (Wire.endTransmission(false) == 0 && Wire.requestFrom((uint8_t)addr, (uint8_t)1) == 1) {
        uint8_t who = Wire.read();
        if (who != 0x00 && who != 0xFF) {
          _mpu_addr = addr;

          // Reset device (PWR_MGMT_1 = 0x80)
          Wire.beginTransmission(addr);
          Wire.write(0x6B);
          Wire.write(0x80);
          Wire.endTransmission();
          delay(50);

          // Wake up: clock source PLL with X Gyro (PWR_MGMT_1 = 0x01)
          Wire.beginTransmission(addr);
          Wire.write(0x6B);
          Wire.write(0x01);
          Wire.endTransmission();
          delay(10);

          // Sample rate divider = 0 (1 kHz)
          Wire.beginTransmission(addr);
          Wire.write(0x19);
          Wire.write(0x00);
          Wire.endTransmission();

          // CONFIG = 0x03 (DLPF ~44Hz)
          Wire.beginTransmission(addr);
          Wire.write(0x1A);
          Wire.write(0x03);
          Wire.endTransmission();

          // ACCEL_CONFIG = 0x10 (±8G range)
          Wire.beginTransmission(addr);
          Wire.write(0x1C);
          Wire.write(0x10);
          Wire.endTransmission();

          accel_type = ACCEL_MPU_DIRECT;
          return true;
        }
      }
    }
  }

  accel_type = ACCEL_NONE;
  return false;
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
  static int accel_retry_count = 0;
  if (!sensor_ok && millis() - last_accel_check > 5000) {
    accel_retry_count++;
    sensor_ok = selfTest();
    if (sensor_ok) {
      if (accel_type == ACCEL_LSM6DS3) {
        _lsm6ds3.setAccelRange(LSM6DS_ACCEL_RANGE_8_G);
        _lsm6ds3.setAccelDataRate(LSM6DS_RATE_1_66K_HZ);
      } else if (accel_type == ACCEL_MPU6050) {
        _mpu6050.setAccelerometerRange(MPU6050_RANGE_8_G);
        _mpu6050.setFilterBandwidth(MPU6050_BAND_44_HZ);
      }
      char ok_msg[64];
      snprintf(ok_msg, sizeof(ok_msg), "[I2C OK] Accel terdeteksi (type=%d, addr=0x%02X)", accel_type, _mpu_addr);
      networkMgr.publishLog(ok_msg);
    } else if (accel_retry_count <= 2 || accel_retry_count % 12 == 0) {
      String s_devs = scanI2CBus(Wire);
      String a_devs = scanI2CBus(Wire1);
      char scan_msg[160];
      snprintf(scan_msg, sizeof(scan_msg), "[I2C SCAN] Seis(SDA=%d,SCL=%d):[%s] Atmo(SDA=%d,SCL=%d):[%s]",
               PIN_I2C_SEIS_SDA, PIN_I2C_SEIS_SCL, s_devs.c_str(),
               PIN_I2C_ATMO_SDA, PIN_I2C_ATMO_SCL, a_devs.c_str());
      networkMgr.publishLog(scan_msg);
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
    if (accel_type == ACCEL_LSM6DS3) {
      _lsm6ds3.getEvent(&accel, &gyro, &temp_accel);
    } else if (accel_type == ACCEL_MPU6050) {
      _mpu6050.getEvent(&accel, &gyro, &temp_accel);
    } else if (accel_type == ACCEL_MPU_DIRECT) {
      Wire.beginTransmission(_mpu_addr);
      Wire.write(0x3B); // ACCEL_XOUT_H
      if (Wire.endTransmission(false) == 0 && Wire.requestFrom(_mpu_addr, (uint8_t)6) == 6) {
        int16_t rx = (int16_t)((Wire.read() << 8) | Wire.read());
        int16_t ry = (int16_t)((Wire.read() << 8) | Wire.read());
        int16_t rz = (int16_t)((Wire.read() << 8) | Wire.read());
        // Rentang ±8G = 4096 LSB/g, gravitasi = 9.80665 m/s^2
        accel.acceleration.x = ((float)rx / 4096.0f) * 9.80665f;
        accel.acceleration.y = ((float)ry / 4096.0f) * 9.80665f;
        accel.acceleration.z = ((float)rz / 4096.0f) * 9.80665f;
      }
    }

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
