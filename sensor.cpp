/* ============================================================================
 *  DriveSafe - sensor.cpp
 * ========================================================================== */
#include <Wire.h>
#include <math.h>
#include "sensor.h"

DriveSafeSensor Sensors;

static const float RAD2DEG = 57.2957795131f;

/* -------------------------------------------------------------------------- */
/*  Akses register dasar                                                      */
/* -------------------------------------------------------------------------- */
bool DriveSafeSensor::w8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(BNO055_I2C_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return (Wire.endTransmission() == 0);
}

bool DriveSafeSensor::r8(uint8_t reg, uint8_t &val) {
  return rN(reg, &val, 1);
}

bool DriveSafeSensor::rN(uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(BNO055_I2C_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;      /* repeated start */
  uint8_t got = Wire.requestFrom((uint8_t)BNO055_I2C_ADDR, (uint8_t)len);
  if (got != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

bool DriveSafeSensor::setMode(uint8_t mode) {
  if (!w8(BNO_REG_OPR_MODE, mode)) return false;
  /* Datasheet: CONFIG -> operasi = 7 ms, operasi -> CONFIG = 19 ms */
  delay(mode == BNO_MODE_CONFIG ? 25 : 20);
  _mode = mode;
  return true;
}

/* -------------------------------------------------------------------------- */
/*  Konfigurasi Low-Pass Filter (Tabel 4.1)                                   */
/*                                                                            */
/*  ACC_CONFIG  (page 1, 0x08): [7:5] pwr mode | [4:2] bandwidth | [1:0] range */
/*     range  : 00=+/-2g 01=+/-4g 10=+/-8g 11=+/-16g                          */
/*     bw     : 011 = 62,5 Hz  <-- nilai yang dipilih dokumen                 */
/*  GYR_CONFIG_0 (page 1, 0x0A): [5:3] bandwidth | [2:0] range                */
/*     range  : 000 = +/-2000 dps                                             */
/*     bw     : 010 = 116 Hz   <-- nilai yang dipilih dokumen                 */
/*                                                                            */
/*  CATATAN PENTING: pada mode fusion (NDOF) kedua register ini dikunci oleh   */
/*  firmware internal BNO055 (ACC +/-4g @62,5 Hz, GYR +/-2000dps @32 Hz),      */
/*  sehingga penulisan diabaikan sensor. Rentang +/-4 g membuat G-Force di     */
/*  atas 4 g ter-clipping dan estimasi severity (Pers. 4.23) tidak dapat       */
/*  membedakan Moderate dari Severe. Karena itu SENSOR_MODE default = AMG.     */
/* -------------------------------------------------------------------------- */
bool DriveSafeSensor::applyLpfConfig() {
  if (!setMode(BNO_MODE_CONFIG)) return false;
  if (!w8(BNO_REG_PAGE_ID, 0x01)) return false;

  /* +/-16 g, bandwidth 62,5 Hz, normal power  -> 0b000 011 11 = 0x0F */
  const uint8_t acc_cfg = (0x00 << 5) | (0x03 << 2) | 0x03;
  /* +/-2000 dps, bandwidth 116 Hz             -> 0b00 010 000 = 0x10 */
  const uint8_t gyr_cfg = (0x02 << 3) | 0x00;

  bool ok = w8(BNO_REG_ACC_CONFIG,   acc_cfg);
  ok     &= w8(BNO_REG_GYR_CONFIG_0, gyr_cfg);

  w8(BNO_REG_PAGE_ID, 0x00);
  return ok;
}

/* -------------------------------------------------------------------------- */
/*  Inisialisasi                                                              */
/* -------------------------------------------------------------------------- */
bool DriveSafeSensor::begin() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_CLOCK_HZ);
  Wire.setTimeOut(50);
  delay(700);                               /* BNO055 boot time ~650 ms      */

  uint8_t id = 0;
  for (uint8_t attempt = 0; attempt < 10; attempt++) {
    if (r8(BNO_REG_CHIP_ID, id) && id == BNO_CHIP_ID_VALUE) break;
    delay(100);
  }
  if (id != BNO_CHIP_ID_VALUE) {
    Serial.printf("[SENSOR] BNO055 tidak terdeteksi (CHIP_ID=0x%02X)\n", id);
    _present = false;
    return false;
  }

  setMode(BNO_MODE_CONFIG);
  w8(BNO_REG_PAGE_ID, 0x00);

  /* Reset perangkat lunak */
  w8(BNO_REG_SYS_TRIGGER, 0x20);
  delay(700);
  for (uint8_t i = 0; i < 10 && !(r8(BNO_REG_CHIP_ID, id) && id == BNO_CHIP_ID_VALUE); i++)
    delay(50);

  w8(BNO_REG_PWR_MODE, 0x00);               /* normal power                  */
  delay(10);
  w8(BNO_REG_SYS_TRIGGER, 0x00);            /* pakai osilator internal       */
  delay(10);

  /* UNIT_SEL = 0x00 : akselerasi m/s2, giroskop dps, euler derajat          */
  w8(BNO_REG_UNIT_SEL, 0x00);
  delay(10);

  /* Orientasi sumbu default P1: sumbu X sensor sejajar arah gerak kendaraan
     (lihat penanda panah pada enclosure, Bab 8.2.4)                         */
  w8(BNO_REG_AXIS_MAP_CFG,  0x24);
  w8(BNO_REG_AXIS_MAP_SIGN, 0x00);
  delay(10);

#if (SENSOR_MODE == SENSOR_MODE_AMG)
  applyLpfConfig();                         /* efektif hanya di mode non-fusion */
  setMode(BNO_MODE_AMG);
  Serial.println(F("[SENSOR] Mode AMG  (+/-16 g, ACC BW 62,5 Hz, GYR BW 116 Hz)"));
  Serial.println(F("[SENSOR] Orientasi dihitung ESP32 (Pers. 4.21/4.22 + comp. filter)"));
#else
  setMode(BNO_MODE_NDOF);
  Serial.println(F("[SENSOR] Mode NDOF (fusion internal, akselerometer dikunci +/-4 g)"));
  Serial.println(F("[SENSOR] PERINGATAN: G-Force > 4 g akan ter-clipping."));
#endif

  _present = true;
  Serial.printf("[SENSOR] BNO055 siap pada 0x%02X, target %.0f Hz\n",
                BNO055_I2C_ADDR, SAMPLE_RATE_HZ);
  return true;
}

/* -------------------------------------------------------------------------- */
/*  Kalibrasi awal - estimasi bias giroskop (Pers. 4.7)                       */
/* -------------------------------------------------------------------------- */
bool DriveSafeSensor::calibrateBias(uint16_t duration_ms) {
  if (!_present) return false;

  Serial.println(F("[SENSOR] Kalibrasi awal: jangan gerakkan perangkat..."));

  double sx = 0, sy = 0, sz = 0;
  uint32_t n = 0;
  float    max_dev = 0;
  uint32_t t0 = millis();

  while (millis() - t0 < duration_ms) {
    uint8_t buf[6];
    if (rN(BNO_REG_GYR_DATA, buf, 6)) {
      const float gx = (int16_t)(buf[0] | (buf[1] << 8)) / BNO_GYR_LSB_PER_DPS;
      const float gy = (int16_t)(buf[2] | (buf[3] << 8)) / BNO_GYR_LSB_PER_DPS;
      const float gz = (int16_t)(buf[4] | (buf[5] << 8)) / BNO_GYR_LSB_PER_DPS;
      sx += gx; sy += gy; sz += gz; n++;

      const float dev = fabsf(gx) + fabsf(gy) + fabsf(gz);
      if (dev > max_dev) max_dev = dev;
    }
    delay(SAMPLE_PERIOD_MS);
  }

  if (n == 0) return false;

  /* Bila perangkat jelas bergerak, bias tidak sahih */
  if (max_dev > 25.0f) {
    Serial.println(F("[SENSOR] Kalibrasi gagal: perangkat bergerak. Bias = 0."));
    _bias_gx = _bias_gy = _bias_gz = 0;
    return false;
  }

  _bias_gx = (float)(sx / n);
  _bias_gy = (float)(sy / n);
  _bias_gz = (float)(sz / n);

  Serial.printf("[SENSOR] Bias giroskop: %.3f / %.3f / %.3f deg/s (n=%lu)\n",
                _bias_gx, _bias_gy, _bias_gz, (unsigned long)n);

  resetOrientation();
  return true;
}

void DriveSafeSensor::resetOrientation() {
  _orientInit = false;
  _roll = _pitch = _yaw = 0;
}

/* -------------------------------------------------------------------------- */
/*  Akuisisi satu cuplikan                                                    */
/* -------------------------------------------------------------------------- */
bool DriveSafeSensor::read(SensorSample &out) {
  if (!_present) return false;

  uint8_t a[6], g[6];
  if (!rN(BNO_REG_ACC_DATA, a, 6)) return false;
  if (!rN(BNO_REG_GYR_DATA, g, 6)) return false;

  out.ax = (int16_t)(a[0] | (a[1] << 8)) / BNO_ACC_LSB_PER_MS2;
  out.ay = (int16_t)(a[2] | (a[3] << 8)) / BNO_ACC_LSB_PER_MS2;
  out.az = (int16_t)(a[4] | (a[5] << 8)) / BNO_ACC_LSB_PER_MS2;

  /* Pers. 4.7 : kurangi bias hasil kalibrasi awal */
  out.gx = (int16_t)(g[0] | (g[1] << 8)) / BNO_GYR_LSB_PER_DPS - _bias_gx;
  out.gy = (int16_t)(g[2] | (g[3] << 8)) / BNO_GYR_LSB_PER_DPS - _bias_gy;
  out.gz = (int16_t)(g[4] | (g[5] << 8)) / BNO_GYR_LSB_PER_DPS - _bias_gz;

  updateOrientation(out);
  return true;
}

/* -------------------------------------------------------------------------- */
/*  Estimasi orientasi                                                        */
/*                                                                            */
/*  Catatan teknis penting terhadap Pers. 4.22 dan Tabel 5.7:                 */
/*  pitch = arctan(ax / sqrt(ay^2+az^2)) secara matematis terbatas pada       */
/*  rentang [-90, +90] derajat, sehingga syarat |theta| > 90 pada Sub-bab      */
/*  4.5.3 TIDAK PERNAH dapat terpenuhi bila hanya memakai akselerometer.      */
/*  Firmware ini mengintegrasikan giroskop (complementary filter, Pers. 4.8)  */
/*  sehingga pitch dapat merentang [-180, +180] dan ambang 90 derajat menjadi  */
/*  dapat dicapai secara fisik saat kendaraan menukik/terguling ekstrem.       */
/* -------------------------------------------------------------------------- */
void DriveSafeSensor::updateOrientation(const SensorSample &s) {
#if (SENSOR_MODE == SENSOR_MODE_NDOF)
  uint8_t e[6];
  if (rN(BNO_REG_EUL_DATA, e, 6)) {
    _yaw   = (int16_t)(e[0] | (e[1] << 8)) / BNO_EUL_LSB_PER_DEG;
    _roll  = (int16_t)(e[2] | (e[3] << 8)) / BNO_EUL_LSB_PER_DEG;
    _pitch = (int16_t)(e[4] | (e[5] << 8)) / BNO_EUL_LSB_PER_DEG;
  }
#else
  /* Pers. 4.21 - 4.22 : orientasi dari akselerometer */
  const float roll_acc  = atan2f(s.ay, s.az) * RAD2DEG;
  const float pitch_acc = atan2f(s.ax, sqrtf(s.ay * s.ay + s.az * s.az)) * RAD2DEG;

  if (!_orientInit) {
    _roll = roll_acc; _pitch = pitch_acc; _yaw = 0;
    _orientInit = true;
    return;
  }

  /* Integrasi giroskop */
  _roll  += s.gx * TS_SECONDS;
  _pitch += s.gy * TS_SECONDS;
  _yaw   += s.gz * TS_SECONDS;

  /* Koreksi akselerometer hanya saat |a| mendekati 1 g. Selama benturan,
     akselerometer tidak lagi merepresentasikan arah gravitasi sehingga
     koreksi dimatikan dan orientasi murni mengandalkan giroskop.          */
  const float amag = sqrtf(s.ax * s.ax + s.ay * s.ay + s.az * s.az);
  const float gdev = fabsf(amag / GRAVITY_MS2 - 1.0f);

  if (gdev < 0.30f) {
    _roll  = COMP_FILTER_ALPHA * _roll  + (1.0f - COMP_FILTER_ALPHA) * roll_acc;
    /* hanya koreksi pitch bila belum melewati singularitas +/-90 deg */
    if (fabsf(_pitch) < 80.0f)
      _pitch = COMP_FILTER_ALPHA * _pitch + (1.0f - COMP_FILTER_ALPHA) * pitch_acc;
  }

  /* Normalisasi ke [-180, 180] */
  while (_roll  >  180.0f) _roll  -= 360.0f;
  while (_roll  < -180.0f) _roll  += 360.0f;
  while (_pitch >  180.0f) _pitch -= 360.0f;
  while (_pitch < -180.0f) _pitch += 360.0f;
  while (_yaw   >  360.0f) _yaw   -= 360.0f;
  while (_yaw   <    0.0f) _yaw   += 360.0f;
#endif
}

/* -------------------------------------------------------------------------- */
/*  Status kalibrasi & offset                                                 */
/* -------------------------------------------------------------------------- */
uint8_t DriveSafeSensor::calibStatus() {
  uint8_t v = 0;
  r8(BNO_REG_CALIB_STAT, v);
  return v;
}

bool DriveSafeSensor::fullyCalibrated() {
  uint8_t c = calibStatus();
  return ((c >> 6) & 0x03) == 3 &&   /* sys  */
         ((c >> 4) & 0x03) == 3 &&   /* gyr  */
         ((c >> 2) & 0x03) == 3;     /* acc  */
}

uint8_t DriveSafeSensor::systemError() {
  uint8_t v = 0;
  r8(BNO_REG_SYS_ERR, v);
  return v;
}

bool DriveSafeSensor::readCalibOffsets(uint8_t *buf22) {
  uint8_t prev = _mode;
  if (!setMode(BNO_MODE_CONFIG)) return false;
  bool ok = rN(BNO_REG_OFFSET_BASE, buf22, 22);
  setMode(prev);
  return ok;
}

bool DriveSafeSensor::writeCalibOffsets(const uint8_t *buf22) {
  uint8_t prev = _mode;
  if (!setMode(BNO_MODE_CONFIG)) return false;
  bool ok = true;
  for (uint8_t i = 0; i < 22; i++) ok &= w8(BNO_REG_OFFSET_BASE + i, buf22[i]);
  setMode(prev);
  return ok;
}
