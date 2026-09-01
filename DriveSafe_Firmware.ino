/*
  =====================================================================
  DriveSafe - Sistem Deteksi Kecelakaan Kendaraan dan Respons Darurat
  Cerdas Berbasis IoT dengan Perangkat Multisensor
  Capstone Project C-251 - DTETI FT UGM

  Hardware:
   - Mikrokontroler : ESP32 DevKit V1
   - IMU 9-DoF      : Bosch BNO055 (I2C)
   - GPS            : u-blox NEO-M8N (UART, NMEA, 9600 bps)
   - GSM/SMS        : SIMCom SIM800L (UART, AT Command)
   - Tombol pembatalan alarm (false alarm cancellation)
   - Buzzer + LED indikator

  Fitur firmware ini mengimplementasikan seluruh model pada Bab 3 & 4
  dokumen: kalibrasi awal sensor, low-pass filter, adaptive threshold
  berbasis profil kendaraan, algoritma deteksi multi-tahap (event
  detection -> post-impact stillness -> abnormal orientation),
  false alarm cancellation, estimasi crash severity, circular buffer
  event logging di RAM, serta pelaporan darurat via SMS + missed call.
  =====================================================================
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <TinyGPS++.h>
#include <Preferences.h>

// ======================= KONFIGURASI PIN ============================
// I2C untuk BNO055 (default ESP32: SDA=21, SCL=22)
#define I2C_SDA_PIN        21
#define I2C_SCL_PIN        22

// UART2 untuk modul GPS NEO-M8N
#define GPS_RX_PIN         16   // ESP32 RX2 <- TX modul GPS
#define GPS_TX_PIN         17   // ESP32 TX2 -> RX modul GPS
#define GPS_BAUD           9600

// UART1 untuk modul GSM SIM800L
#define GSM_RX_PIN         26   // ESP32 RX1 <- TX SIM800L
#define GSM_TX_PIN         27   // ESP32 TX1 -> RX SIM800L
#define GSM_BAUD           9600

// Tombol pembatalan alarm (aktif rendah, gunakan INPUT_PULLUP)
#define BUTTON_CANCEL_PIN  4

// Indikator visual & audio
#define BUZZER_PIN         25
#define LED_PIN            2

// ======================= KONFIGURASI SISTEM =========================
// Ganti sesuai kendaraan tempat perangkat dipasang: 0=Sepeda, 1=Motor, 2=Mobil
#define VEHICLE_PROFILE_DEFAULT   1

#define SAMPLE_RATE_HZ     100
#define SAMPLE_PERIOD_MS   (1000 / SAMPLE_RATE_HZ)   // 10 ms -> Ts = 0.01 s

#define PRE_BUFFER_SEC     10     // circular buffer pre-impact
#define POST_BUFFER_SEC    5      // buffer post-impact
#define PRE_BUFFER_LEN     (PRE_BUFFER_SEC  * SAMPLE_RATE_HZ)   // 1000 sampel
#define POST_BUFFER_LEN    (POST_BUFFER_SEC * SAMPLE_RATE_HZ)   // 500 sampel

#define CALIBRATION_SAMPLES 200   // ~2 detik pengambilan sampel referensi awal

#define STILLNESS_CONFIRM_MS   3000   // 3 detik post-impact stillness
#define ORIENTATION_CONFIRM_MS 2000   // 2 detik abnormal orientation confirmation
#define ROLLOVER_PERSIST_MS    1500   // Troll = 1.5 detik

#define ALARM_CANCEL_MIN_MS   3000    // jendela pembatalan minimum 3 detik
#define ALARM_CANCEL_MAX_MS   30000   // jendela pembatalan maksimum 30 detik

#define ROLL_THRESHOLD_DEG     60.0f
#define PITCH_THRESHOLD_DEG    90.0f
#define ROLLOVER_THRESHOLD_DEG 60.0f

#define STILLNESS_GYRO_DPS     5.0f    // ambang "hampir diam" (deg/s)
#define STILLNESS_ACCEL_G      0.15f   // toleransi deviasi accel dari 1g saat diam

#define LPF_ALPHA          0.20f       // koefisien low-pass filter (0 < alpha <= 1)

#define MAX_EMERGENCY_CONTACTS 5

const float G_TO_MS2 = 9.81f;

// ======================= PROFIL KENDARAAN (Tabel 4.2) ================
struct VehicleThreshold {
  const char *name;
  float Gth;     // dalam satuan g
  float Rth;     // rotational jerk, deg/s^2
  float Jth;     // jerk, m/s^3
};

VehicleThreshold vehicleProfiles[3] = {
  { "Sepeda", 2.0f, 600.0f, 30.0f },
  { "Motor",  2.5f, 800.0f, 40.0f },
  { "Mobil",  3.0f, 500.0f, 50.0f }
};

uint8_t activeVehicleProfile = VEHICLE_PROFILE_DEFAULT;

// ======================= OBJEK PERIFERAL =============================
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
HardwareSerial gpsSerial(2);   // UART2
HardwareSerial gsmSerial(1);   // UART1
TinyGPSPlus gps;
Preferences prefs;

// ======================= STRUKTUR DATA SENSOR =========================
struct SensorSample {
  float ax, ay, az;   // m/s^2, sudah dikurangi baseline gravitasi bila perlu
  float gx, gy, gz;   // deg/s
};

struct EventLogEntry {           // Tabel 4.4 Format Struktur Data Event Log
  uint32_t timestamp;
  float    gPeak;
  float    jPeak;
  float    rPeak;
  float    lat;
  float    lon;
  uint8_t  severity;   // 0=Minor, 1=Moderate, 2=Severe
};

// Circular buffer pre-impact (disimpan di RAM/SRAM internal ESP32)
SensorSample preBuffer[PRE_BUFFER_LEN];
uint16_t preBufferHead = 0;
bool     preBufferFull = false;

// Buffer post-impact (linear, direset setiap event baru)
SensorSample postBuffer[POST_BUFFER_LEN];
uint16_t postBufferIndex = 0;

// ======================= STATE MACHINE SISTEM =========================
enum SystemState {
  STATE_CALIBRATING,
  STATE_MONITORING,
  STATE_EVENT_DETECTED,      // f1 terpicu, memasuki Accident Verification Mode
  STATE_ALARM_PENDING,       // f[n] terkonfirmasi, menunggu jendela pembatalan
  STATE_SENDING_EMERGENCY,
  STATE_POST_EVENT_COOLDOWN
};

SystemState currentState = STATE_CALIBRATING;

// ======================= VARIABEL GLOBAL RUNTIME =======================
float baselineAx = 0, baselineAy = 0, baselineAz = 0;   // referensi gravitasi awal
float baselineGx = 0, baselineGy = 0, baselineGz = 0;   // offset drift giroskop
float baselineRoll = 0, baselinePitch = 0;

float lpfAx = 0, lpfAy = 0, lpfAz = 0;                  // hasil low-pass filter
float lpfGx = 0, lpfGy = 0, lpfGz = 0;

float prevGForce = 1.0f;      // untuk perhitungan jerk (turunan G-Force)
unsigned long prevSampleTime = 0;

unsigned long lastSampleMillis = 0;
unsigned long eventDetectedMillis = 0;
unsigned long verificationStartMillis = 0;
unsigned long rollAbnormalStartMillis = 0;
bool rollAbnormalActive = false;

volatile bool cancelButtonPressed = false;

float currentGPeak = 0, currentJPeak = 0, currentRPeak = 0;

// Kontak darurat (disimpan di NVS / Preferences agar persisten)
struct EmergencyContact {
  char name[24];
  char phone[16];
  bool isPriority;   // kontak prioritas -> target missed call
};
EmergencyContact contacts[MAX_EMERGENCY_CONTACTS];
uint8_t contactCount = 0;

char driverName[24] = "Pengendara";

// =====================================================================
//                              SETUP
// =====================================================================
void setup() {
  Serial.begin(115200);
  Serial.println(F("[DriveSafe] Booting..."));

  pinMode(BUTTON_CANCEL_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LED_PIN, LOW);

  attachInterrupt(digitalPinToInterrupt(BUTTON_CANCEL_PIN), onCancelButtonISR, FALLING);

  // I2C untuk BNO055
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  if (!bno.begin()) {
    Serial.println(F("[ERROR] BNO055 tidak terdeteksi. Periksa wiring I2C."));
    while (1) { blinkError(); }
  }
  bno.setExtCrystalUse(true);

  // UART GPS (NEO-M8N)
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  // UART GSM (SIM800L)
  gsmSerial.begin(GSM_BAUD, SERIAL_8N1, GSM_RX_PIN, GSM_TX_PIN);
  delay(2000);
  initGSMModule();

  // Muat kontak darurat & profil kendaraan dari NVS
  loadConfigFromNVS();

  Serial.print(F("[DriveSafe] Profil kendaraan aktif: "));
  Serial.println(vehicleProfiles[activeVehicleProfile].name);

  // Kalibrasi awal sensor (Bab 3.1.1)
  runInitialCalibration();

  currentState = STATE_MONITORING;
  lastSampleMillis = millis();
  Serial.println(F("[DriveSafe] Sistem siap. Memasuki mode monitoring."));
}

// =====================================================================
//                               LOOP
// =====================================================================
void loop() {
  // Selalu proses data GPS yang masuk secara asinkron
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  unsigned long now = millis();
  if (now - lastSampleMillis < SAMPLE_PERIOD_MS) {
    return;  // menjaga sampling rate tetap 100 Hz
  }
  lastSampleMillis = now;

  SensorSample raw = readIMU();
  SensorSample filtered = applyLowPassFilter(raw);

  // Simpan selalu ke circular buffer pre-impact
  pushToPreBuffer(filtered);

  float gForce = computeGForce(filtered);           // magnitudo total, satuan g
  float rollRate = computeRotationalJerk(filtered);  // deg/s^2 (turunan angular velocity)
  float jerk = computeJerk(gForce, now);             // m/s^3

  float roll, pitch;
  computeRollPitch(filtered, roll, pitch);

  switch (currentState) {
    case STATE_MONITORING:
      handleMonitoring(gForce, jerk, rollRate, roll, pitch, gForce, now);
      break;

    case STATE_EVENT_DETECTED:
      handleVerification(filtered, gForce, roll, pitch, now);
      break;

    case STATE_ALARM_PENDING:
      handleAlarmPending(now);
      break;

    case STATE_SENDING_EMERGENCY:
      // Ditangani secara sinkron di dalam triggerEmergencyProcedure()
      break;

    case STATE_POST_EVENT_COOLDOWN:
      handlePostEventCooldown(filtered, now);
      break;

    default:
      break;
  }
}

// =====================================================================
//                     TAHAP 1: EVENT DETECTION  (f1[n])
//   f1[n] = (G[n] > Gth) OR (J[n] > Jth) OR (R[n] > Rth)      -- Pers. 4.16
// =====================================================================
void handleMonitoring(float gForce, float jerk, float rollRate,
                       float roll, float pitch, float gPeakSoFar, unsigned long now) {

  VehicleThreshold vt = vehicleProfiles[activeVehicleProfile];

  bool f1_gforce = (gForce > vt.Gth);
  bool f1_jerk    = (fabs(jerk) > vt.Jth);
  bool f1_rollrate = (fabs(rollRate) > vt.Rth);

  bool f1 = f1_gforce || f1_jerk || f1_rollrate;

  // Deteksi orientasi ekstrem (rollover) berjalan paralel & independen
  checkRolloverCondition(roll, pitch, now);

  if (f1) {
    Serial.println(F("[EVENT] Kejadian abnormal terdeteksi. Memasuki Accident Verification Mode."));
    eventDetectedMillis = now;
    verificationStartMillis = now;
    currentGPeak = gForce;
    currentJPeak = fabs(jerk);
    currentRPeak = fabs(rollRate);
    postBufferIndex = 0;

    currentState = STATE_EVENT_DETECTED;
    setIndicator(true);   // nyalakan LED sebagai indikator verifikasi berjalan
  }
}

// =====================================================================
//         TAHAP 2 & 3: VERIFICATION  f[n] = f1[n] AND (f2[n] OR f3[n])
//   f2 : post-impact stillness (3 detik)
//   f3 : abnormal orientation  (2 detik)
// =====================================================================
void handleVerification(SensorSample &s, float gForce, float roll, float pitch,
                         unsigned long now) {

  // Update nilai puncak selama verifikasi
  if (gForce > currentGPeak) currentGPeak = gForce;

  // Simpan sampel ke post-impact buffer selama fase verifikasi
  if (postBufferIndex < POST_BUFFER_LEN) {
    postBuffer[postBufferIndex++] = s;
  }

  unsigned long elapsed = now - verificationStartMillis;

  bool f2_stillness   = checkPostImpactStillness(s, elapsed);
  bool f3_orientation = checkAbnormalOrientation(roll, pitch, elapsed);

  bool confirmed = f2_stillness || f3_orientation;

  // Batas waktu maksimum fase verifikasi (5 detik: 3s stillness + 2s orientasi,
  // ditambah sedikit overhead komputasi sesuai Tabel 6.2)
  unsigned long maxVerificationMs = STILLNESS_CONFIRM_MS + ORIENTATION_CONFIRM_MS + 1000;

  if (confirmed) {
    Serial.println(F("[VERIFIKASI] Kecelakaan terkonfirmasi -> menunggu pembatalan pengguna."));
    eventDetectedMillis = now;   // titik acuan jendela pembatalan alarm
    cancelButtonPressed = false;
    currentState = STATE_ALARM_PENDING;
    setBuzzer(true);
  } else if (elapsed > maxVerificationMs) {
    Serial.println(F("[VERIFIKASI] Tidak terkonfirmasi dalam jendela waktu. Kembali ke monitoring."));
    currentState = STATE_MONITORING;
    setIndicator(false);
  }
}

// f2[n]: kondisi kendaraan diam / hampir diam setelah kejadian
bool checkPostImpactStillness(SensorSample &s, unsigned long elapsed) {
  float gyroMag = sqrt(s.gx * s.gx + s.gy * s.gy + s.gz * s.gz);
  float accelMag = sqrt(s.ax * s.ax + s.ay * s.ay + s.az * s.az) / G_TO_MS2;

  bool isStillNow = (gyroMag < STILLNESS_GYRO_DPS) &&
                     (fabs(accelMag - 1.0f) < STILLNESS_ACCEL_G);

  static unsigned long stillnessStartMs = 0;
  static bool stillnessTracking = false;

  if (isStillNow) {
    if (!stillnessTracking) {
      stillnessTracking = true;
      stillnessStartMs = millis();
    }
    if (millis() - stillnessStartMs >= STILLNESS_CONFIRM_MS) {
      stillnessTracking = false;   // reset untuk event berikutnya
      return true;
    }
  } else {
    stillnessTracking = false;
  }
  return false;
}

// f3[n]: orientasi kendaraan abnormal (miring/terguling) bertahan >= 2 detik
bool checkAbnormalOrientation(float roll, float pitch, unsigned long elapsed) {
  bool abnormalNow = (fabs(roll) > ROLL_THRESHOLD_DEG) ||
                      (fabs(pitch) > PITCH_THRESHOLD_DEG);

  static unsigned long orientationStartMs = 0;
  static bool orientationTracking = false;

  if (abnormalNow) {
    if (!orientationTracking) {
      orientationTracking = true;
      orientationStartMs = millis();
    }
    if (millis() - orientationStartMs >= ORIENTATION_CONFIRM_MS) {
      orientationTracking = false;
      return true;
    }
  } else {
    orientationTracking = false;
  }
  return false;
}

// Deteksi rollover/overturn persisten -> dapat memicu notifikasi independen
// rollover[n] = 1 jika |roll|>60 atau |pitch|>60 bertahan selama Troll = 1.5s (Pers. 4.23)
void checkRolloverCondition(float roll, float pitch, unsigned long now) {
  bool extreme = (fabs(roll) > ROLLOVER_THRESHOLD_DEG) || (fabs(pitch) > ROLLOVER_THRESHOLD_DEG);

  if (extreme) {
    if (!rollAbnormalActive) {
      rollAbnormalActive = true;
      rollAbnormalStartMillis = now;
    } else if (now - rollAbnormalStartMillis >= ROLLOVER_PERSIST_MS) {
      // Rollover terkonfirmasi -> picu langsung ke tahap alarm bila masih monitoring
      if (currentState == STATE_MONITORING) {
        Serial.println(F("[ROLLOVER] Kendaraan terguling terdeteksi secara independen."));
        eventDetectedMillis = now;
        currentGPeak = max(currentGPeak, 1.0f);
        currentJPeak = 0;
        currentRPeak = 0;
        cancelButtonPressed = false;
        currentState = STATE_ALARM_PENDING;
        setBuzzer(true);
      }
      rollAbnormalActive = false;  // hindari re-trigger berulang
    }
  } else {
    rollAbnormalActive = false;
  }
}

// =====================================================================
//         TAHAP FALSE ALARM CANCELLATION (jendela 3 - 30 detik)
// =====================================================================
void handleAlarmPending(unsigned long now) {
  unsigned long elapsedSinceConfirm = now - eventDetectedMillis;

  // Tombol hanya berlaku pada jendela waktu minimum-maksimum yang ditentukan
  if (cancelButtonPressed &&
      elapsedSinceConfirm >= ALARM_CANCEL_MIN_MS &&
      elapsedSinceConfirm <= ALARM_CANCEL_MAX_MS) {
    Serial.println(F("[ALARM] Dibatalkan oleh pengguna."));
    cancelButtonPressed = false;
    setBuzzer(false);
    setIndicator(false);
    currentState = STATE_MONITORING;
    return;
  }

  if (elapsedSinceConfirm > ALARM_CANCEL_MAX_MS) {
    Serial.println(F("[ALARM] Jendela pembatalan berakhir. Menjalankan prosedur darurat."));
    setBuzzer(false);
    currentState = STATE_SENDING_EMERGENCY;
    triggerEmergencyProcedure();
  }
}

void IRAM_ATTR onCancelButtonISR() {
  cancelButtonPressed = true;
}

// =====================================================================
//                  PROSEDUR DARURAT (SMS + Missed Call)
// =====================================================================
void triggerEmergencyProcedure() {
  uint8_t severity = classifySeverity(currentGPeak);

  double lat = gps.location.isValid() ? gps.location.lat() : 0.0;
  double lon = gps.location.isValid() ? gps.location.lng() : 0.0;

  EventLogEntry ev;
  ev.timestamp = gps.time.isValid() ? gps.time.value() : (millis() / 1000);
  ev.gPeak = currentGPeak;
  ev.jPeak = currentJPeak;
  ev.rPeak = currentRPeak;
  ev.lat = lat;
  ev.lon = lon;
  ev.severity = severity;

  logEventToSerial(ev);   // event log RAM (35.2 KB pre+post buffer sudah tersimpan)

  String message = buildEmergencyMessage(ev, lat, lon);

  bool anySent = false;
  for (uint8_t i = 0; i < contactCount; i++) {
    if (sendSMS(contacts[i].phone, message)) {
      anySent = true;
    }
  }

  // Missed call hanya ke kontak prioritas utama
  for (uint8_t i = 0; i < contactCount; i++) {
    if (contacts[i].isPriority) {
      makeMissedCall(contacts[i].phone);
      break;
    }
  }

  Serial.println(anySent ? F("[GSM] Notifikasi darurat terkirim.")
                          : F("[GSM] Gagal mengirim notifikasi darurat."));

  currentState = STATE_POST_EVENT_COOLDOWN;
  eventDetectedMillis = millis();
  setIndicator(false);
}

String buildEmergencyMessage(EventLogEntry &ev, double lat, double lon) {
  String status;
  if (ev.rPeak > 0 && ev.jPeak == 0) {
    status = "ROLLOVER TERDETEKSI";
  } else {
    status = "KECELAKAAN TERDETEKSI";
  }

  String level = (ev.severity == 2) ? "TINGGI" : (ev.severity == 1) ? "SEDANG" : "RENDAH";

  char timeBuf[24];
  if (gps.date.isValid() && gps.time.isValid()) {
    snprintf(timeBuf, sizeof(timeBuf), "%02d/%02d/%04d %02d:%02d:%02d",
             gps.date.day(), gps.date.month(), gps.date.year(),
             gps.time.hour(), gps.time.minute(), gps.time.second());
  } else {
    snprintf(timeBuf, sizeof(timeBuf), "t+%lus", millis() / 1000);
  }

  String msg = "[DRIVESAFE - NOTIFIKASI DARURAT]\n";
  msg += "Pengendara : " + String(driverName) + "\n";
  msg += "Status     : " + status + "\n";
  msg += "Level      : " + level + "\n";
  msg += "Waktu      : " + String(timeBuf) + "\n";
  msg += "Lokasi     : " + String(lat, 6) + "," + String(lon, 6) + "\n";
  msg += "Link       : https://maps.google.com/?q=" + String(lat, 6) + "," + String(lon, 6) + "\n";
  msg += "Mohon segera melakukan pengecekan dan bantuan.";

  // Batasi ke 160 karakter GSM 7-bit sesuai spesifikasi (Bab 3.2.1)
  if (msg.length() > 160) {
    msg = msg.substring(0, 160);
  }
  return msg;
}

uint8_t classifySeverity(float gPeak) {
  // Persamaan 4.23 (Estimasi Crash Severity)
  if (gPeak < 4.0f) return 0;      // Minor
  if (gPeak < 7.0f) return 1;      // Moderate
  return 2;                        // Severe
}

// =====================================================================
//                      MODUL GSM SIM800L (AT COMMAND)
// =====================================================================
void initGSMModule() {
  sendATCommand("AT", 1000);
  sendATCommand("AT+CMGF=1", 1000);       // mode teks SMS
  sendATCommand("AT+CNMI=1,2,0,0,0", 1000);
}

bool waitNetworkReady() {
  String resp = sendATCommand("AT+CREG?", 2000);
  bool registered = (resp.indexOf(",1") != -1) || (resp.indexOf(",5") != -1);

  String sigResp = sendATCommand("AT+CSQ", 2000);
  int csqIndex = sigResp.indexOf("+CSQ:");
  int signalQuality = 0;
  if (csqIndex != -1) {
    signalQuality = sigResp.substring(csqIndex + 6).toInt();
  }
  return registered && (signalQuality > 5) && (signalQuality != 99);
}

bool sendSMS(const char *phoneNumber, String &message) {
  const uint8_t MAX_RETRY = 2;

  for (uint8_t attempt = 0; attempt <= MAX_RETRY; attempt++) {
    if (!waitNetworkReady()) {
      Serial.println(F("[GSM] Jaringan/sinyal belum siap, mencoba ulang..."));
      delay(1500);
      continue;
    }

    sendATCommand("AT+CMGF=1", 500);

    gsmSerial.print("AT+CMGS=\"");
    gsmSerial.print(phoneNumber);
    gsmSerial.println("\"");
    delay(500);

    gsmSerial.print(message);
    delay(200);
    gsmSerial.write(0x1A);   // Ctrl+Z untuk menyelesaikan pengiriman

    String resp = readGSMResponse(8000);
    if (resp.indexOf("OK") != -1 || resp.indexOf("+CMGS") != -1) {
      Serial.print(F("[GSM] SMS terkirim ke "));
      Serial.println(phoneNumber);
      return true;
    }
    Serial.print(F("[GSM] Percobaan gagal ke "));
    Serial.println(phoneNumber);
  }

  Serial.print(F("[GSM] SMS gagal setelah retry ke "));
  Serial.println(phoneNumber);
  return false;
}

void makeMissedCall(const char *phoneNumber) {
  Serial.print(F("[GSM] Melakukan missed call ke kontak prioritas: "));
  Serial.println(phoneNumber);

  gsmSerial.print("ATD");
  gsmSerial.print(phoneNumber);
  gsmSerial.println(";");
  delay(5000);              // durasi panggilan singkat
  sendATCommand("ATH", 1000);   // putuskan panggilan otomatis
}

String sendATCommand(const char *cmd, unsigned long timeoutMs) {
  gsmSerial.println(cmd);
  return readGSMResponse(timeoutMs);
}

String readGSMResponse(unsigned long timeoutMs) {
  String response = "";
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    while (gsmSerial.available()) {
      response += (char)gsmSerial.read();
    }
  }
  return response;
}

// =====================================================================
//                      AKUISISI & PENGOLAHAN DATA IMU
// =====================================================================
SensorSample readIMU() {
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);  // m/s^2
  imu::Vector<3> gyro  = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);      // rad/s -> lib biasanya deg/s tergantung setting

  SensorSample s;
  s.ax = accel.x();
  s.ay = accel.y();
  s.az = accel.z();
  s.gx = gyro.x() - baselineGx;
  s.gy = gyro.y() - baselineGy;
  s.gz = gyro.z() - baselineGz;
  return s;
}

SensorSample applyLowPassFilter(SensorSample &raw) {
  // y[n] = alpha * x[n] + (1 - alpha) * y[n-1]   (Bab 3.1.2)
  lpfAx = LPF_ALPHA * raw.ax + (1 - LPF_ALPHA) * lpfAx;
  lpfAy = LPF_ALPHA * raw.ay + (1 - LPF_ALPHA) * lpfAy;
  lpfAz = LPF_ALPHA * raw.az + (1 - LPF_ALPHA) * lpfAz;
  lpfGx = LPF_ALPHA * raw.gx + (1 - LPF_ALPHA) * lpfGx;
  lpfGy = LPF_ALPHA * raw.gy + (1 - LPF_ALPHA) * lpfGy;
  lpfGz = LPF_ALPHA * raw.gz + (1 - LPF_ALPHA) * lpfGz;

  SensorSample f;
  f.ax = lpfAx; f.ay = lpfAy; f.az = lpfAz;
  f.gx = lpfGx; f.gy = lpfGy; f.gz = lpfGz;
  return f;
}

// Magnitudo percepatan total dalam satuan g (absolute value acceleration, Bab 3.1)
float computeGForce(SensorSample &s) {
  float magnitude = sqrt(s.ax * s.ax + s.ay * s.ay + s.az * s.az); // m/s^2
  return magnitude / G_TO_MS2;
}

// Jerk = turunan G-Force terhadap waktu (m/s^3)
float computeJerk(float gForceNow, unsigned long now) {
  if (prevSampleTime == 0) {
    prevSampleTime = now;
    prevGForce = gForceNow;
    return 0;
  }
  float dt = (now - prevSampleTime) / 1000.0f;
  if (dt <= 0) dt = SAMPLE_PERIOD_MS / 1000.0f;

  float jerk = ((gForceNow - prevGForce) * G_TO_MS2) / dt;

  prevGForce = gForceNow;
  prevSampleTime = now;
  return jerk;
}

// Rotational jerk / roll rate sederhana: magnitudo kecepatan sudut giroskop (deg/s^2 approx)
float computeRotationalJerk(SensorSample &s) {
  static float prevGyroMag = 0;
  static unsigned long prevT = 0;

  float gyroMag = sqrt(s.gx * s.gx + s.gy * s.gy + s.gz * s.gz);
  unsigned long now = millis();

  if (prevT == 0) {
    prevT = now;
    prevGyroMag = gyroMag;
    return 0;
  }
  float dt = (now - prevT) / 1000.0f;
  if (dt <= 0) dt = SAMPLE_PERIOD_MS / 1000.0f;

  float rate = (gyroMag - prevGyroMag) / dt;
  prevGyroMag = gyroMag;
  prevT = now;
  return rate;
}

// Sudut roll & pitch berbasis pembacaan akselerometer (Pers. 4.21 & 4.22)
void computeRollPitch(SensorSample &s, float &roll, float &pitch) {
  roll  = degrees(atan2(s.ay, s.az)) - baselineRoll;
  pitch = degrees(atan2(-s.ax, sqrt(s.ay * s.ay + s.az * s.az))) - baselinePitch;
}

// =====================================================================
//                    KALIBRASI AWAL SENSOR (Bab 3.1.1)
// =====================================================================
void runInitialCalibration() {
  Serial.println(F("[KALIBRASI] Pastikan kendaraan diam & tegak. Mulai kalibrasi..."));
  blinkCalibrating();

  double sumAx = 0, sumAy = 0, sumAz = 0;
  double sumGx = 0, sumGy = 0, sumGz = 0;

  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> gyro  = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

    sumAx += accel.x(); sumAy += accel.y(); sumAz += accel.z();
    sumGx += gyro.x();  sumGy += gyro.y();  sumGz += gyro.z();

    delay(SAMPLE_PERIOD_MS);
  }

  baselineAx = sumAx / CALIBRATION_SAMPLES;
  baselineAy = sumAy / CALIBRATION_SAMPLES;
  baselineAz = sumAz / CALIBRATION_SAMPLES;
  baselineGx = sumGx / CALIBRATION_SAMPLES;
  baselineGy = sumGy / CALIBRATION_SAMPLES;
  baselineGz = sumGz / CALIBRATION_SAMPLES;

  baselineRoll  = degrees(atan2(baselineAy, baselineAz));
  baselinePitch = degrees(atan2(-baselineAx, sqrt(baselineAy * baselineAy + baselineAz * baselineAz)));

  // Inisialisasi filter dengan nilai baseline agar tidak ada lonjakan awal
  lpfAx = baselineAx; lpfAy = baselineAy; lpfAz = baselineAz;
  lpfGx = 0; lpfGy = 0; lpfGz = 0;

  Serial.println(F("[KALIBRASI] Selesai. Baseline referensi tersimpan."));
}

// =====================================================================
//                    MANAJEMEN BUFFER EVENT LOGGING
// =====================================================================
void pushToPreBuffer(SensorSample &s) {
  preBuffer[preBufferHead] = s;
  preBufferHead = (preBufferHead + 1) % PRE_BUFFER_LEN;
  if (preBufferHead == 0) preBufferFull = true;
}

void logEventToSerial(EventLogEntry &ev) {
  Serial.println(F("===== EVENT LOG ====="));
  Serial.print(F("Timestamp : ")); Serial.println(ev.timestamp);
  Serial.print(F("G_peak    : ")); Serial.print(ev.gPeak, 2); Serial.println(F(" g"));
  Serial.print(F("J_peak    : ")); Serial.print(ev.jPeak, 2); Serial.println(F(" m/s^3"));
  Serial.print(F("R_peak    : ")); Serial.print(ev.rPeak, 2); Serial.println(F(" deg/s^2"));
  Serial.print(F("Lokasi    : ")); Serial.print(ev.lat, 6); Serial.print(F(", ")); Serial.println(ev.lon, 6);
  Serial.print(F("Severity  : ")); Serial.println(ev.severity);
  Serial.println(F("Pre-buffer 10s & post-buffer 5s tersimpan di RAM (~35.2 KB)."));
  Serial.println(F("======================"));
}

void handlePostEventCooldown(SensorSample &s, unsigned long now) {
  // Beri jeda sebelum kembali memantau agar tidak re-trigger dari sisa getaran
  const unsigned long COOLDOWN_MS = 5000;
  if (now - eventDetectedMillis > COOLDOWN_MS) {
    Serial.println(F("[SYSTEM] Kembali ke mode monitoring normal."));
    prevSampleTime = 0;
    currentState = STATE_MONITORING;
  }
}

// =====================================================================
//               KONFIGURASI KONTAK DARURAT & PROFIL (NVS)
// =====================================================================
void loadConfigFromNVS() {
  prefs.begin("drivesafe", true);   // read-only

  activeVehicleProfile = prefs.getUChar("profile", VEHICLE_PROFILE_DEFAULT);
  contactCount = prefs.getUChar("contactCount", 0);

  String dName = prefs.getString("driverName", "Pengendara");
  dName.toCharArray(driverName, sizeof(driverName));

  for (uint8_t i = 0; i < contactCount && i < MAX_EMERGENCY_CONTACTS; i++) {
    String keyName = "c" + String(i) + "n";
    String keyPhone = "c" + String(i) + "p";
    String keyPrio = "c" + String(i) + "x";

    String n = prefs.getString(keyName.c_str(), "");
    String p = prefs.getString(keyPhone.c_str(), "");
    bool prio = prefs.getBool(keyPrio.c_str(), false);

    n.toCharArray(contacts[i].name, sizeof(contacts[i].name));
    p.toCharArray(contacts[i].phone, sizeof(contacts[i].phone));
    contacts[i].isPriority = prio;
  }

  prefs.end();

  // Fallback: jika belum ada kontak terkonfigurasi (mis. saat pengujian awal)
  if (contactCount == 0) {
    strcpy(contacts[0].name, "Kontak Darurat 1");
    strcpy(contacts[0].phone, "+6281234567890");
    contacts[0].isPriority = true;
    contactCount = 1;
    Serial.println(F("[WARNING] Belum ada kontak darurat tersimpan di NVS, menggunakan default sementara."));
  }
}

// Dipanggil dari aplikasi mobile via BLE (di luar cakupan file ini) untuk
// menyimpan konfigurasi kontak & profil kendaraan ke NVS.
void saveConfigToNVS() {
  prefs.begin("drivesafe", false);
  prefs.putUChar("profile", activeVehicleProfile);
  prefs.putUChar("contactCount", contactCount);
  prefs.putString("driverName", driverName);

  for (uint8_t i = 0; i < contactCount; i++) {
    String keyName = "c" + String(i) + "n";
    String keyPhone = "c" + String(i) + "p";
    String keyPrio = "c" + String(i) + "x";

    prefs.putString(keyName.c_str(), contacts[i].name);
    prefs.putString(keyPhone.c_str(), contacts[i].phone);
    prefs.putBool(keyPrio.c_str(), contacts[i].isPriority);
  }
  prefs.end();
}

// =====================================================================
//                        INDIKATOR VISUAL & AUDIO
// =====================================================================
void setIndicator(bool on) {
  digitalWrite(LED_PIN, on ? HIGH : LOW);
}

void setBuzzer(bool on) {
  digitalWrite(BUZZER_PIN, on ? HIGH : LOW);
}

void blinkError() {
  digitalWrite(LED_PIN, HIGH); delay(150);
  digitalWrite(LED_PIN, LOW);  delay(150);
}

void blinkCalibrating() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH); delay(200);
    digitalWrite(LED_PIN, LOW);  delay(200);
  }
}
