/* ============================================================================
 *  DriveSafe - sensor.h
 *  Driver BNO055 mandiri (I2C/Wire) + kalibrasi awal + estimasi orientasi.
 *
 *  Mengimplementasikan:
 *    Bab 4.3    konfigurasi digital low-pass filter (Tabel 4.1)
 *    Pers. 4.7  kompensasi bias giroskop  w_gyro = w + b_g + eta_g
 *    Pers. 4.21 roll   = arctan(ay / az)
 *    Pers. 4.22 pitch  = arctan(ax / sqrt(ay^2 + az^2))
 *    Pers. 4.8  complementary filter orde-1 y[n]=a*x[n]+(1-a)*y[n-1]
 * ========================================================================== */
#ifndef DRIVESAFE_SENSOR_H
#define DRIVESAFE_SENSOR_H

#include "config.h"

/* ---- Peta register BNO055 (datasheet Bosch BST-BNO055-DS000) ------------- */
#define BNO_REG_PAGE_ID         0x07
#define BNO_REG_CHIP_ID         0x00
#define BNO_CHIP_ID_VALUE       0xA0
#define BNO_REG_ACC_DATA        0x08   /* page 0 : 6 byte X/Y/Z LSB-MSB      */
#define BNO_REG_GYR_DATA        0x14   /* page 0 : 6 byte                    */
#define BNO_REG_EUL_DATA        0x1A   /* page 0 : heading, roll, pitch      */
#define BNO_REG_LIA_DATA        0x28   /* page 0 : linear acceleration       */
#define BNO_REG_CALIB_STAT      0x35
#define BNO_REG_SYS_STATUS      0x39
#define BNO_REG_SYS_ERR         0x3A
#define BNO_REG_UNIT_SEL        0x3B
#define BNO_REG_OPR_MODE        0x3D
#define BNO_REG_PWR_MODE        0x3E
#define BNO_REG_SYS_TRIGGER     0x3F
#define BNO_REG_AXIS_MAP_CFG    0x41
#define BNO_REG_AXIS_MAP_SIGN   0x42
#define BNO_REG_OFFSET_BASE     0x55   /* 22 byte kalibrasi                  */

#define BNO_REG_ACC_CONFIG      0x08   /* page 1                             */
#define BNO_REG_GYR_CONFIG_0    0x0A   /* page 1                             */

#define BNO_MODE_CONFIG         0x00
#define BNO_MODE_ACCGYRO        0x05
#define BNO_MODE_AMG            0x07
#define BNO_MODE_NDOF           0x0C

/* Faktor skala unit standar: ACC 100 LSB/(m/s2), GYR 16 LSB/dps, EUL 16 LSB/deg */
#define BNO_ACC_LSB_PER_MS2     100.0f
#define BNO_GYR_LSB_PER_DPS     16.0f
#define BNO_EUL_LSB_PER_DEG     16.0f

class DriveSafeSensor {
public:
  bool     begin();
  bool     isPresent() const { return _present; }

  /* Kalibrasi awal: estimasi bias giroskop b_g saat kendaraan diam.
     Mengembalikan false bila perangkat bergerak selama proses. */
  bool     calibrateBias(uint16_t duration_ms = 2000);

  /* Satu siklus akuisisi 100 Hz. Mengisi sample dan memperbarui orientasi. */
  bool     read(SensorSample &out);

  /* Orientasi hasil complementary filter / fusion NDOF (derajat) */
  float    roll()  const { return _roll;  }
  float    pitch() const { return _pitch; }
  float    yaw()   const { return _yaw;   }
  void     resetOrientation();

  uint8_t  calibStatus();                 /* byte CALIB_STAT mentah          */
  bool     fullyCalibrated();
  uint8_t  systemError();

  /* Simpan / muat 22 byte offset kalibrasi BNO055 ke-/dari NVS */
  bool     readCalibOffsets(uint8_t *buf22);
  bool     writeCalibOffsets(const uint8_t *buf22);

  float    biasX() const { return _bias_gx; }
  float    biasY() const { return _bias_gy; }
  float    biasZ() const { return _bias_gz; }

private:
  bool     w8(uint8_t reg, uint8_t val);
  bool     r8(uint8_t reg, uint8_t &val);
  bool     rN(uint8_t reg, uint8_t *buf, uint8_t len);
  bool     setMode(uint8_t mode);
  bool     applyLpfConfig();              /* Tabel 4.1 : ACC 62,5 Hz / GYR 116 Hz */
  void     updateOrientation(const SensorSample &s);

  bool     _present   = false;
  uint8_t  _mode      = BNO_MODE_CONFIG;
  float    _bias_gx   = 0, _bias_gy = 0, _bias_gz = 0;
  float    _roll      = 0, _pitch   = 0, _yaw    = 0;
  bool     _orientInit = false;
};

extern DriveSafeSensor Sensors;

#endif /* DRIVESAFE_SENSOR_H */
