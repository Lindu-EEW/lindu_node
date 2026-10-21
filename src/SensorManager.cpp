#include "SensorManager.h"
#include "NetworkManager.h"
#include "lindu_ml_model.h"

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
      _lsm6ds3.setAccelDataRate(LSM6DS_RATE_104_HZ); // Anti-aliasing filter matched to 100Hz sampling
    } else if (accel_type == ACCEL_MPU6050) {
      _mpu6050.setAccelerometerRange(MPU6050_RANGE_8_G);
      _mpu6050.setFilterBandwidth(MPU6050_BAND_10_HZ); // Hardware DLPF ~10Hz (Seismic bandwidth 0.5-8Hz)
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

          // CONFIG = 0x05 (DLPF ~10Hz, delay 13.8ms untuk eliminasi noise frekuensi tinggi)
          Wire.beginTransmission(addr);
          Wire.write(0x1A);
          Wire.write(0x05);
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
        _lsm6ds3.setAccelDataRate(LSM6DS_RATE_104_HZ); // Anti-aliasing 104Hz
      } else if (accel_type == ACCEL_MPU6050) {
        _mpu6050.setAccelerometerRange(MPU6050_RANGE_8_G);
        _mpu6050.setFilterBandwidth(MPU6050_BAND_10_HZ); // Hardware DLPF ~10Hz
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

    // Multi-sample calibration saat booting (300 sampel @ 100Hz = 3.0 detik)
    if (!_is_calibrated) {
      _calib_sum_x += accel.acceleration.x;
      _calib_sum_y += accel.acceleration.y;
      _calib_sum_z += accel.acceleration.z;
      _calib_samples_count++;

      if (_calib_samples_count >= CALIBRATION_SAMPLE_TARGET) {
        _dc_x = _calib_sum_x / (float)CALIBRATION_SAMPLE_TARGET;
        _dc_y = _calib_sum_y / (float)CALIBRATION_SAMPLE_TARGET;
        _dc_z = _calib_sum_z / (float)CALIBRATION_SAMPLE_TARGET;
        _is_calibrated = true;
        _prev_raw_x = accel.acceleration.x;
        _prev_raw_y = accel.acceleration.y;
        _prev_raw_z = accel.acceleration.z;
        _stationary_samples = 0;
        _sta_ema = 0.0f;
        _lta_ema = 0.0001f;
        _zcr_count = 0;
        Serial.printf("[SENSOR] Kalibrasi Selesai! Baseline DC: (%.3f, %.3f, %.3f) m/s^2 (%d sampel)\n",
                      _dc_x, _dc_y, _dc_z, CALIBRATION_SAMPLE_TARGET);
        char c_msg[128];
        snprintf(c_msg, sizeof(c_msg), "[CALIB OK] Baseline Tare Selesai: (%.3f, %.3f, %.3f) m/s^2, Tilt: %.1f° (%s)",
                 _dc_x, _dc_y, _dc_z, getTiltAngle(), getPose().c_str());
        networkMgr.publishLog(c_msg);
        networkMgr.forcePublishStatus();
      } else {
        // Tahan pengiriman data dan perhitungan PGA selama kalibrasi
        return;
      }
    }

    // Auto-Tare Kontinu (Zero-Tracking saat sensor diam):
    // Hitung deviasi dinamik instan relatif terhadap baseline
    float raw_dyn_x = accel.acceleration.x - _dc_x;
    float raw_dyn_y = accel.acceleration.y - _dc_y;
    float raw_dyn_z = accel.acceleration.z - _dc_z;
    float inst_pga = sqrt(raw_dyn_x * raw_dyn_x + raw_dyn_y * raw_dyn_y + raw_dyn_z * raw_dyn_z) / 9.80665f;

    // Saat kondisi tenang (inst_pga < 0.015g), perbarui baseline DC secara adaptif
    // Ini memastikan posisi sensor miring, vertikal di dinding, maupun tidur selalu ter-nol-kan (tare ke 0)
    if (inst_pga < 0.015f) {
      float alpha_dc = 0.005f;
      _dc_x = (accel.acceleration.x * alpha_dc) + (_dc_x * (1.0f - alpha_dc));
      _dc_y = (accel.acceleration.y * alpha_dc) + (_dc_y * (1.0f - alpha_dc));
      _dc_z = (accel.acceleration.z * alpha_dc) + (_dc_z * (1.0f - alpha_dc));
      _stationary_samples = 0;
    } else {
      // Jika terjadi pergeseran orientasi / node tumbang / dipindahkan (inst_pga >= 0.015g),
      // periksa apakah sensor sudah diam di posisi baru (bukan gempa yang dinamis/bergoyang):
      float total_g = sqrt(accel.acceleration.x * accel.acceleration.x +
                           accel.acceleration.y * accel.acceleration.y +
                           accel.acceleration.z * accel.acceleration.z) / 9.80665f;
      float sample_jitter = (abs(accel.acceleration.x - _prev_raw_x) +
                             abs(accel.acceleration.y - _prev_raw_y) +
                             abs(accel.acceleration.z - _prev_raw_z)) / 9.80665f;

      // Ciri sensor diam di posisi baru:
      // 1. Total magnitudo gravitasi bumi murni ~1.0g (antara 0.80g dan 1.20g)
      // 2. Jitter antar-sampel sangat kecil (< 0.025g)
      if (total_g >= 0.80f && total_g <= 1.20f && sample_jitter < 0.025f) {
        _stationary_samples++;
        // 250 sampel @ 100Hz = 2.5 detik diam stabil di posisi baru
        if (_stationary_samples >= 250) {
          Serial.printf("[SENSOR] Auto-Tare! Node stabil di orientasi baru (offset: %.3fG). Menginisiasi tare...\n", inst_pga);
          networkMgr.publishLog("[AUTO-TARE] Orientasi baru stabil terdeteksi (diam 2.5s). Menjalankan auto-tare...");
          tare();
          _stationary_samples = 0;
          return;
        }
      } else {
        // Jika sensor masih bergerak/bergoyang (atau sedang gempa asli), reset counter
        _stationary_samples = 0;
      }
    }

    _prev_raw_x = accel.acceleration.x;
    _prev_raw_y = accel.acceleration.y;
    _prev_raw_z = accel.acceleration.z;

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

    float raw_pga = sqrt(dyn_x * dyn_x + dyn_y * dyn_y + dyn_z * dyn_z) / 9.80665f;
    // Deadband peredam noise sensor miring:
    // Fluktuasi MEMS diam di bawah batas sensor diam (0.0030g) di-clamp bersih ke 0.000g
    if (raw_pga < 0.0030f) {
      pga = 0.0f;
    } else {
      pga = raw_pga;
    }
    float energy = pga * pga;
    rms = sqrt((dyn_x * dyn_x + dyn_y * dyn_y + dyn_z * dyn_z) / 3.0f) / 9.80665f;

    _sta_ema = (energy * 0.1f) + (_sta_ema * 0.9f);
    _lta_ema = (energy * 0.005f) + (_lta_ema * 0.995f);

    // Lacak noise floor saat kondisi tenang (pga < 0.04g)
    if (pga < 0.04f) {
      _noise_floor_ema = (pga * 0.005f) + (_noise_floor_ema * 0.995f);
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

  // TRACK 1: Algoritma Klasik (Heuristik Fisika Seismik)
  // Frekuensi seismik gempa tektonik <= 15 Hz; deteksi getaran Shindo 1-3 (PGA >= 0.0040g, STA/LTA >= 1.50)
  bool is_seismic_freq = (_current_hz == 0 || _current_hz <= 15);
  bool is_earthquake_spike = (pga >= 0.0040f && is_seismic_freq && ratio >= 1.50f) || (pga >= 0.05f && is_seismic_freq && ratio >= 2.0f); 

  // TRACK 2: TinyML Shadow Mode - Berjalan Paralel Menilai Probabilitas Gempa Tanpa Menyentuh Aktuator
  static unsigned long _vibe_start_time = 0;
  float duration_s = 1.0f;
  if (pga > 0.0035f) {
    if (_vibe_start_time == 0) _vibe_start_time = millis();
    duration_s = (millis() - _vibe_start_time) / 1000.0f;
    if (duration_s < 0.5f) duration_s = 0.5f;
  } else {
    _vibe_start_time = 0;
    duration_s = 1.0f;
  }

  // Hitung probabilitas TinyML (0.00 - 1.00)
  float ml_prob = 0.01f;
  if (pga >= 0.003f) {
    float effective_hz = ((float)_current_hz > 0.0f) ? (float)_current_hz : (pga >= 0.05f ? 2.5f : 10.0f);
    ml_prob = lindu_predict_earthquake(pga, ratio, effective_hz, rms, duration_s, 0.5f);
  } else {
    ml_prob = 0.01f + (pga / 0.003f) * 0.02f;
  }
  bool ml_detect = (ml_prob >= 0.35f);
  bool ml_agree = (ml_detect == is_earthquake_spike);

  if ( (is_earthquake_spike && millis() - _telemetry_last_send >= 100) || (millis() - _telemetry_last_send >= 1000) ) {
    SensorEvent ev = {pga,   ratio, _current_hz, rms,  dyn_x,
                      dyn_y, dyn_z, millis(),    temp, pres,
                      ml_prob, ml_agree};
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

void SensorManager::tare() {
  _calib_sum_x = 0.0f;
  _calib_sum_y = 0.0f;
  _calib_sum_z = 0.0f;
  _calib_samples_count = 0;
  _stationary_samples = 0;
  _is_calibrated = false;
  Serial.println("[SENSOR] Tare / Kalibrasi ulang diinisiasi.");
}
