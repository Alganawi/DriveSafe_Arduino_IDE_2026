#ifndef CONFIG_H
#define CONFIG_H

// =====================================================================
// KONFIGURASI PIN - sesuaikan dengan wiring PCB/purwarupa kamu
// =====================================================================

// I2C - Sensor BNO055
#define PIN_SDA           21
#define PIN_SCL           22

// UART1 - Modul GSM SIM800L
#define PIN_SIM800L_RX    26   // ke TX SIM800L
#define PIN_SIM800L_TX    27   // ke RX SIM800L
#define SIM800L_BAUD      9600

// UART2 - Modul GPS NEO-6M / NEO-M8N
#define PIN_GPS_RX        16   // ke TX GPS
#define PIN_GPS_TX        17   // ke RX GPS
#define GPS_BAUD          9600

// Tombol fisik (pairing mode / batalkan false alarm)
#define PIN_BUTTON        33

// Buzzer indikator audio
#define PIN_BUZZER        25

// Pembacaan tegangan baterai (opsional, via voltage divider)
#define PIN_BATTERY_ADC   34

// =====================================================================
// PARAMETER SAMPLING & FILTER (Bab 4.3 laporan)
// =====================================================================
#define SAMPLE_RATE_HZ     100
#define SAMPLE_PERIOD_MS   (1000 / SAMPLE_RATE_HZ)
#define TS_SECONDS         (1.0f / SAMPLE_RATE_HZ)   // Ts pada rumus jerk/rotational jerk

// BNO055 - bandwidth default (Tabel 4.1): accel 62.5Hz, gyro 116Hz, delay ~8ms
// (BNO055_OPERATION_MODE_NDOF sudah pakai fusion internal, filter di firmware
//  BNO055 sendiri; nilai ini hanya sebagai referensi dokumentasi)

// =====================================================================
// PROFIL KENDARAAN & ADAPTIVE THRESHOLD (Tabel 4.2)
// =====================================================================
enum VehicleProfile {
  PROFILE_MOBIL = 0,
  PROFILE_MOTOR = 1,
  PROFILE_SEPEDA = 2
};

struct ThresholdConfig {
  float Gth_base;   // dalam satuan g
  float Rth_base;   // dalam derajat/s^2
  float Jth_base;   // dalam m/s^3
};

// Tabel 4.2 - Konfigurasi Adaptive Threshold berdasarkan Profil Kendaraan
static const ThresholdConfig VEHICLE_THRESHOLDS[3] = {
  /* PROFILE_MOBIL  */ { 3.0f,  500.0f, 50.0f },
  /* PROFILE_MOTOR  */ { 2.5f,  800.0f, 40.0f },
  /* PROFILE_SEPEDA */ { 2.0f,  600.0f, 30.0f }
};

// Parameter koreksi kspeed (Persamaan 4.10)
#define BETA_SENSITIVITY   0.15f
#define OMEGA_REF_DEG_S    100.0f

// =====================================================================
// PARAMETER DETEKSI ORIENTASI (Bab 4.5.3)
// =====================================================================
#define ROLL_THRESHOLD_DEG      60.0f
#define PITCH_THRESHOLD_DEG     90.0f
#define ROLLOVER_THRESHOLD_DEG  60.0f
#define ROLLOVER_PERSIST_MS     1500   // Troll = 1.5 detik

// =====================================================================
// PARAMETER VERIFIKASI & COUNTDOWN (Bab 4.5.2, 4.5.4)
// =====================================================================
#define VERIFICATION_WINDOW_MS   3000    // jendela cek "diam" pasca-spike
#define STILLNESS_G_MIN          0.9f    // ~1g -> kendaraan diam/hampir diam
#define STILLNESS_G_MAX          1.1f
#define STILLNESS_GYRO_MAX       10.0f   // deg/s, dianggap "diam" kalau di bawah ini

#define FALSE_ALARM_MIN_MS       3000    // tombol batal aktif mulai detik ke-3
#define FALSE_ALARM_COUNTDOWN_MS 30000   // total countdown 30 detik

// Severity (Persamaan 4.23)
#define SEVERITY_MODERATE_G   4.0f
#define SEVERITY_SEVERE_G     7.0f

// =====================================================================
// KONTAK DARURAT
// =====================================================================
#define MAX_EMERGENCY_CONTACTS 5
#define SMS_RETRY_MAX           2
#define MISSED_CALL_DURATION_MS 5000

#endif
