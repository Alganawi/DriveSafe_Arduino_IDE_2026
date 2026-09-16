/* ============================================================================
 *  DriveSafe  -  Dokumen C-251 (Capstone B-06 DTETI UGM)
 *  config.h : seluruh konstanta sistem (pin, threshold, timing, UUID)
 *
 *  Referensi dokumen:
 *    Tabel 4.1  konfigurasi LPF BNO055
 *    Tabel 4.2  adaptive threshold per profil kendaraan
 *    Tabel 5.6  hasil perhitungan awal threshold percepatan
 *    Tabel 5.7  threshold parameter deteksi insiden
 *    Pers. 4.9 - 4.12, 4.13 - 4.15, 4.16 - 4.17, 4.21 - 4.23
 *    Tabel 6.2  spesifikasi yang dijanjikan
 * ========================================================================== */
#ifndef DRIVESAFE_CONFIG_H
#define DRIVESAFE_CONFIG_H

#include <Arduino.h>

/* ---------------------------------------------------------------------------
 * 0. VERSI & IDENTITAS
 * ------------------------------------------------------------------------- */
#define FW_NAME                 "DriveSafe"
#define FW_VERSION              "1.0.0"
#define DOC_CODE                "C-251-B06"

/* ---------------------------------------------------------------------------
 * 1. PEMETAAN PIN ESP32 DevKit V1
 * ------------------------------------------------------------------------- */
/* I2C - Sensor inersial BNO055 (Bab 8.2.5) */
#define PIN_I2C_SDA             21
#define PIN_I2C_SCL             22
#define I2C_CLOCK_HZ            400000UL
#define BNO055_I2C_ADDR         0x28        /* 0x29 bila ADR di-pull HIGH   */

/* UART1 - Modul GPS NEO-M8N (Bab 8.3 poin 1) */
#define PIN_GPS_RX              16          /* ESP32 RX  <- GPS  TX         */
#define PIN_GPS_TX              17          /* ESP32 TX  -> GPS  RX         */
#define GPS_BAUD                9600UL

/* UART2 - Modul GSM SIM800L (Bab 8.3 poin 2-3) */
#define PIN_GSM_RX              26          /* ESP32 RX  <- SIM800L TX      */
#define PIN_GSM_TX              27          /* ESP32 TX  -> SIM800L RX      */
#define PIN_GSM_RST             14          /* opsional, -1 bila tak dipakai*/
#define GSM_BAUD                9600UL

/* Antarmuka pengguna */
#define PIN_BUZZER              25          /* buzzer aktif 100 dB          */
#define PIN_BUTTON              33          /* push button INTERRUPT, aktif-rendah */
#define PIN_LED_STATUS          2           /* LED onboard                  */

/* Monitoring baterai Li-ion 18650 melalui pembagi tegangan 100k/100k */
#define PIN_VBAT_ADC            34          /* GPIO input-only + ADC1       */
#define VBAT_DIVIDER_RATIO      2.0f
#define VBAT_ADC_REF            3.30f
#define VBAT_FULL               4.20f
#define VBAT_EMPTY              3.20f

/* ---------------------------------------------------------------------------
 * 2. AKUISISI DATA  (Tabel 6.2 no.2 : minimal 100 Hz)
 * ------------------------------------------------------------------------- */
#define SAMPLE_RATE_HZ          100.0f
#define SAMPLE_PERIOD_MS        10          /* Ts = 0,01 s                  */
#define TS_SECONDS              (1.0f / SAMPLE_RATE_HZ)
#define GRAVITY_MS2             9.81f       /* Pers. 5.3                    */

/* Mode operasi BNO055
 *   MODE_NDOF : sensor fusion internal (Bab 4.3). Orientasi absolut akurat,
 *               tetapi rentang akselerometer dikunci firmware pada +/-4 g
 *               sehingga G-Force > 4 g akan ter-clipping.
 *   MODE_AMG  : accel/mag/gyro mentah, rentang akselerometer dapat diset
 *               +/-16 g sehingga estimasi severity (Pers. 4.23) tetap valid
 *               sampai 7 g ke atas. Orientasi dihitung di ESP32 memakai
 *               Pers. 4.21-4.22 + complementary filter.
 * Ganti nilai di bawah ini untuk memilih mode. */
#define SENSOR_MODE_NDOF        0
#define SENSOR_MODE_AMG         1
#define SENSOR_MODE             SENSOR_MODE_AMG

/* Bandwidth LPF yang dipilih dokumen (Tabel 4.1, baris 62,5 Hz) */
#define LPF_ACC_BW_HZ           62.5f
#define LPF_GYR_BW_HZ           116.0f
#define LPF_TYPICAL_DELAY_MS    8.0f

/* Koefisien complementary filter untuk mode AMG (Pers. 4.8, y=a*x+(1-a)*y) */
#define COMP_FILTER_ALPHA       0.98f

/* ---------------------------------------------------------------------------
 * 3. PROFIL KENDARAAN & ADAPTIVE THRESHOLD
 *    Nilai G memakai hasil perhitungan matematis Tabel 5.6,
 *    nilai R dan J memakai Tabel 4.2.
 * ------------------------------------------------------------------------- */
enum VehicleProfile : uint8_t {
  PROFILE_SEPEDA = 0,
  PROFILE_MOTOR  = 1,
  PROFILE_MOBIL  = 2,
  PROFILE_COUNT  = 3
};

struct ProfileThreshold {
  const char *name;
  float g_th;      /* G-Force dasar            (g)     Tabel 5.6 */
  float r_th;      /* rotational jerk          (deg/s2) Tabel 4.2 */
  float j_th;      /* jerk linear              (m/s3)  Tabel 4.2 */
};

static const ProfileThreshold PROFILE_TABLE[PROFILE_COUNT] = {
  /* name      G_th    R_th     J_th  */
  { "sepeda",  2.0f,   600.0f,  30.0f },
  { "motor",   3.2f,   800.0f,  40.0f },
  { "mobil",   4.5f,   500.0f,  50.0f }
};

/* ---------------------------------------------------------------------------
 *  CATATAN KALIBRASI EMPIRIS (penting sebelum uji jalan)
 *
 *  J[n] pada Pers. 4.14 dihitung sebagai selisih magnitudo percepatan antar
 *  cuplikan dibagi Ts. Dengan Ts = 0,01 s, perubahan 0,4 m/s2 antar cuplikan
 *  saja sudah menghasilkan J = 40 m/s3, yaitu tepat nilai J_th profil motor
 *  pada Tabel 4.2. Pengukuran menunjukkan polisi tidur ringan menghasilkan
 *  J di kisaran 700 m/s3 dan benturan 4,5 g mencapai ~3.500 m/s3, sehingga
 *  nilai 30-50 m/s3 akan hampir selalu terpicu pada 100 Hz.
 *
 *  Nilai Tabel 4.2 dipertahankan sebagai default agar firmware konsisten
 *  dengan dokumen. Setelah menjalankan perintah 'baseline' pada uji manuver
 *  normal (Tabel 10.1 tahap 2), naikkan kedua pengali di bawah ini ke angka
 *  empiris yang diperoleh, lalu perbarui Tabel 4.2 pada dokumen.
 * ------------------------------------------------------------------------- */
#define JERK_THRESHOLD_SCALE    1.0f    /* pengali J_th  (1.0 = nilai Tabel 4.2) */
#define ROTJERK_THRESHOLD_SCALE 1.0f    /* pengali R_th  (1.0 = nilai Tabel 4.2) */

/* Faktor koreksi adaptif (Pers. 4.9 - 4.12) */
#define BETA_SENSITIVITY        0.15f       /* beta, default dokumen        */
#define OMEGA_REF_DPS           100.0f      /* omega_ref, default dokumen   */
#define K_SLOPE_MAX             2.00f       /* clamp 1/cos(pitch)           */
#define K_SPEED_MAX             2.00f       /* clamp faktor kecepatan sudut */

/* ---------------------------------------------------------------------------
 * 4. THRESHOLD DETEKSI INSIDEN  (Tabel 5.7 & Sub-bab 4.5.3)
 * ------------------------------------------------------------------------- */
#define SHOCK_AXIS_TH_MS2       12.0f       /* |ax| DAN |ay| > 12 m/s2      */
#define ROLL_ANGLE_TH_DEG       60.0f       /* |phi|   >= 60 deg            */
#define PITCH_ANGLE_TH_DEG      90.0f       /* |theta| >= 90 deg            */
#define OVERTURN_TH_DEG         60.0f       /* |phi| DAN |theta| >= 60 deg  */
#define ROLLOVER_PERSIST_MS     1500        /* T_roll = 1,5 s (Pers. 4.23)  */

/* Kondisi kendaraan diam pascabenturan (f2 pada Pers. 4.17) */
#define STILL_G_TOLERANCE       0.15f       /* |G - 1g| < 0,15 g            */
#define STILL_GYRO_TH_DPS       20.0f       /* |omega| < 20 deg/s           */

/* ---------------------------------------------------------------------------
 * 5. TIMING MESIN KEADAAN  (Sub-bab 5.10 & Tabel 6.2 no.3-4)
 * ------------------------------------------------------------------------- */
#define VERIFY_STILL_MS         3000        /* 3 s post-impact stillness    */
#define VERIFY_ORIENT_MS        2000        /* 2 s abnormal orientation     */
#define VERIFY_WINDOW_MS        5000        /* total fase verifikasi        */

/* t_spare = 10 (observasi) + 15 (pembatalan) + 5 (finalisasi) = 30 s */
#define SPARE_OBSERVE_MS        10000
#define SPARE_CANCEL_MS         15000
#define SPARE_FINALIZE_MS       5000
#define SPARE_TOTAL_MS          (SPARE_OBSERVE_MS + SPARE_CANCEL_MS + SPARE_FINALIZE_MS)

/* Jendela tombol pembatalan valid (Sub-bab 4.5.4) : 3 s s.d. 30 s */
#define CANCEL_WINDOW_MIN_MS    3000
#define CANCEL_WINDOW_MAX_MS    30000

/* Jeda sebelum sistem siap mendeteksi insiden berikutnya */
#define COOLDOWN_MS             20000
#define BUTTON_DEBOUNCE_MS      60
#define BUTTON_LONGPRESS_MS     2000        /* tekan lama = mode pairing    */

/* ---------------------------------------------------------------------------
 * 6. EVENT LOGGING  (Sub-bab 4.6, Pers. 4.27 - 4.30)
 * ------------------------------------------------------------------------- */
#define PRE_IMPACT_SECONDS      10
#define POST_IMPACT_SECONDS     5
#define PRE_SAMPLES             ((int)(PRE_IMPACT_SECONDS  * SAMPLE_RATE_HZ))  /* 1000 */
#define POST_SAMPLES            ((int)(POST_IMPACT_SECONDS * SAMPLE_RATE_HZ))  /*  500 */
#define TOTAL_SAMPLES           (PRE_SAMPLES + POST_SAMPLES)                   /* 1500 */
/* 6 sumbu x 1500 sampel x 4 byte = 36.000 byte ~ 35,2 KB (Pers. 4.27) */

#define ENABLE_FLASH_LOG        1           /* salin event log ke LittleFS  */
#define MAX_FLASH_EVENTS        5

/* ---------------------------------------------------------------------------
 * 7. KONTAK DARURAT & NVS  (Tabel 6.2 no.8, Pers. 4.31)
 * ------------------------------------------------------------------------- */
#define MAX_CONTACTS            5
#define CONTACT_NAME_LEN        16
#define CONTACT_NUMBER_LEN      16          /* 16 byte/nomor (Pers. 4.31)   */
#define OWNER_NAME_LEN          20
#define NVS_NAMESPACE           "drivesafe"

/* ---------------------------------------------------------------------------
 * 8. MODUL GSM  (Bab 8.3)
 * ------------------------------------------------------------------------- */
#define SMS_MAX_CHARS           160         /* GSM 7-bit single part        */
#define SMS_MAX_RETRY           2           /* retry otomatis maks 2x       */
#define GSM_AT_TIMEOUT_MS       3000
#define GSM_SMS_TIMEOUT_MS      15000
#define MISSED_CALL_DURATION_MS 5000        /* ATD 5 detik lalu ATH         */
#define GSM_BOOT_TIMEOUT_MS     10000

/* ---------------------------------------------------------------------------
 * 9. BLUETOOTH LOW ENERGY  (Bab 8.5, 10.2.1)
 * ------------------------------------------------------------------------- */
#define BLE_DEVICE_PREFIX       "DriveSafe-"
#define BLE_SVC_UUID            "4f1a0000-9c1e-4a2b-8d33-7b21c0de5afe"
#define BLE_CHR_CONFIG_UUID     "4f1a0001-9c1e-4a2b-8d33-7b21c0de5afe" /* WRITE  */
#define BLE_CHR_STATUS_UUID     "4f1a0002-9c1e-4a2b-8d33-7b21c0de5afe" /* NOTIFY */
#define BLE_CHR_EVENT_UUID      "4f1a0003-9c1e-4a2b-8d33-7b21c0de5afe" /* NOTIFY */
#define BLE_CHR_CMD_UUID        "4f1a0004-9c1e-4a2b-8d33-7b21c0de5afe" /* WRITE  */
#define BLE_STATUS_PERIOD_MS    1000
#define BLE_RX_BUFFER_LEN       768

/* ---------------------------------------------------------------------------
 * 10. TIPE DATA BERSAMA
 * ------------------------------------------------------------------------- */

/* Satu cuplikan sensor: 6 sumbu float32 (24 byte) */
struct SensorSample {
  float ax, ay, az;      /* m/s2   */
  float gx, gy, gz;      /* deg/s  */
};

/* Fitur turunan (Pers. 4.13 - 4.15) + orientasi (Pers. 4.21 - 4.22) */
struct SensorFeature {
  float g_force;         /* G[n]      (g)       */
  float a_mag;           /* |a|       (m/s2)    */
  float jerk;            /* J[n]      (m/s3)    */
  float rot_jerk;        /* R[n]      (deg/s2)  */
  float omega_mag;       /* |omega|   (deg/s)   */
  float roll;            /* phi       (deg)     */
  float pitch;           /* theta     (deg)     */
  float yaw;             /* psi       (deg)     */
  float g_threshold;     /* Gth[n] adaptif (g)  */
  float k_slope;
  float k_speed;
};

/* Tingkat keparahan (Pers. 4.23) */
enum CrashSeverity : uint8_t {
  SEVERITY_MINOR    = 0,   /* Gpeak < 4 g          */
  SEVERITY_MODERATE = 1,   /* 4 g <= Gpeak < 7 g   */
  SEVERITY_SEVERE   = 2    /* Gpeak >= 7 g         */
};

/* Klasifikasi tipe benturan (Bab 5.7) */
enum CrashType : uint8_t {
  CRASH_UNKNOWN   = 0,
  CRASH_FRONTAL   = 1,
  CRASH_REAR      = 2,
  CRASH_SIDE      = 3,
  CRASH_FALL      = 4,
  CRASH_ROLLOVER  = 5
};

/* Struktur event log (Tabel 4.4) */
struct EventLog {
  uint32_t timestamp;      /* epoch GPS (detik)            */
  float    g_peak;         /* G-Force tertinggi   (g)      */
  float    j_peak;         /* jerk tertinggi      (m/s3)   */
  float    r_peak;         /* rotational jerk     (deg/s2) */
  float    lat;
  float    lon;
  float    roll_peak;
  float    pitch_peak;
  uint8_t  severity;       /* 0 minor, 1 moderate, 2 severe*/
  uint8_t  crash_type;
  uint8_t  profile;
  uint8_t  trigger_flags;  /* bit0 G, bit1 J, bit2 R, bit3 shock, bit4 rollover */
};

#define TRIG_G        0x01
#define TRIG_J        0x02
#define TRIG_R        0x04
#define TRIG_SHOCK    0x08
#define TRIG_ROLLOVER 0x10

/* Mesin keadaan sistem (Gambar 4.2 & 4.3) */
enum SystemState : uint8_t {
  ST_BOOT = 0,
  ST_CALIBRATING,
  ST_MONITORING,
  ST_VERIFYING,
  ST_COUNTDOWN,
  ST_ALERTING,
  ST_CANCELLED,
  ST_COOLDOWN,
  ST_PAIRING
};

const char *stateName(SystemState s);
const char *severityName(uint8_t s);
const char *crashTypeName(uint8_t t);

#endif /* DRIVESAFE_CONFIG_H */
