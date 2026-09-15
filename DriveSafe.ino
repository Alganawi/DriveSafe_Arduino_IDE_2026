g
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Preferences.h>

#include "config.h"
#include "detection.h"
#include "gsm_gps.h"
#include "ble_pairing.h"


// OBJEK GLOBAL
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

HardwareSerial SIM800L(1);   // UART1
HardwareSerial GPSSerial(2); // UART2
TinyGPSPlus gps;

Preferences prefs;
BLECharacteristic *statusChar = nullptr;


// STATE GLOBAL (dideklarasikan extern di detection.h)
SystemState currentState = STATE_NORMAL;
VehicleProfile activeProfile = PROFILE_MOTOR;   // default, akan di-overwrite oleh NVS

float lastG = 1.0f;    // asumsi awal diam = 1g
float lastGx = 0, lastGy = 0, lastGz = 0;
float peakG = 0;

unsigned long verificationStartMs = 0;
unsigned long countdownStartMs = 0;
unsigned long rolloverStartMs = 0;
bool rolloverTimerActive = false;

String emergencyContacts[MAX_EMERGENCY_CONTACTS] = {
  // TODO: isi via BLE pairing, contoh sementara untuk testing:
  // "+628123456789"
};
int emergencyContactCount = 0;

String vehicleId = "DRIVESAFE-001";

unsigned long lastSampleMs = 0;
bool buttonPressed = false;

// SETUP
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("=== DriveSafe Firmware Boot ===");

  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);

  // --- I2C & BNO055 ---
  Wire.begin(PIN_SDA, PIN_SCL);
  if (!bno.begin(Adafruit_BNO055::OPERATION_MODE_NDOF)) {
    Serial.println("ERROR: BNO055 tidak terdeteksi! Cek wiring I2C.");
    while (1) { delay(1000); }
  }
  bno.setExtCrystalUse(true);
  Serial.println("BNO055 siap (mode NDOF).");

  // --- GPS & GSM ---
  gpsInit();
  gsmInit();
  Serial.println("GPS & GSM UART siap.");

  // --- BLE pairing + load config tersimpan ---
  bleLoadSavedConfig();
  bleInit(&statusChar);

  lastSampleMs = millis();
  Serial.println("=== Sistem siap, memasuki mode monitoring ===");
}

// LOOP UTAMA
void loop() {
  unsigned long now = millis();

  // Feed parser GPS terus-menerus (non-blocking)
  gpsFeed();

  // Sampling sensor pada 100 Hz
  if (now - lastSampleMs >= SAMPLE_PERIOD_MS) {
    lastSampleMs = now;
    runDetectionCycle(now);
  }

  // Cek tombol fisik (pembatalan false alarm / mode pairing)
  handleButton(now);

  // Kirim status via BLE tiap 1 detik (kalau ada koneksi)
  static unsigned long lastStatusMs = 0;
  if (now - lastStatusMs >= 1000) {
    lastStatusMs = now;
    float battPct = readBatteryPercent();
    bleNotifyStatus(statusChar, battPct, gps.location.isValid(), gps.speed.kmph());
  }
}

// SATU SIKLUS SAMPLING + STATE MACHINE DETEKSI
void runDetectionCycle(unsigned long now) {
  // --- baca data mentah dari BNO055 ---
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER); // m/s^2
  imu::Vector<3> gyro  = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);      // rad/s (lib default)

  SensorSample s;
  // konversi ke satuan g (BNO055 accel default m/s^2, 1g = 9.80665 m/s^2)
  s.ax = accel.x() / 9.80665f;
  s.ay = accel.y() / 9.80665f;
  s.az = accel.z() / 9.80665f;
  // konversi rad/s -> deg/s
  s.gx = gyro.x() * 180.0f / PI;
  s.gy = gyro.y() * 180.0f / PI;
  s.gz = gyro.z() * 180.0f / PI;

  DerivedFeatures f = computeFeatures(s);

  switch (currentState) {

    case STATE_NORMAL: {
      if (detectEvent(f, s)) {
        Serial.println("[EVENT] Kejadian abnormal terdeteksi -> masuk verifikasi.");
        currentState = STATE_VERIFICATION;
        verificationStartMs = now;
        peakG = f.G;
      }
      // deteksi rollover independen (Bab 4.5.3), bisa langsung trigger darurat
      if (checkRolloverPersistent(f, now)) {
        Serial.println("[EVENT] Rollover persisten terdeteksi -> konfirmasi langsung.");
        peakG = max(peakG, f.G);
        currentState = STATE_CONFIRMED;
      }
      break;
    }

    case STATE_VERIFICATION: {
      peakG = max(peakG, f.G);

      bool isStill = checkStillness(s);
      bool abnormalOrientation = checkAbnormalOrientation(f);

      if (isStill || abnormalOrientation) {
        Serial.println("[VERIFIKASI] Terkonfirmasi (f2 atau f3 terpenuhi).");
        currentState = STATE_CONFIRMED;
      } else if (now - verificationStartMs >= VERIFICATION_WINDOW_MS) {
        Serial.println("[VERIFIKASI] Gagal terkonfirmasi -> kembali ke NORMAL (false positive dicegah).");
        currentState = STATE_NORMAL;
        peakG = 0;
      }
      break;
    }

    case STATE_CONFIRMED: {
      Serial.println("[CONFIRMED] Kecelakaan terkonfirmasi. Mulai countdown false-alarm 30 detik.");
      countdownStartMs = now;
      currentState = STATE_COUNTDOWN;
      digitalWrite(PIN_BUZZER, HIGH);
      break;
    }

    case STATE_COUNTDOWN: {
      unsigned long elapsed = now - countdownStartMs;
      if (elapsed >= FALSE_ALARM_COUNTDOWN_MS) {
        Serial.println("[COUNTDOWN] Waktu habis, tidak ada pembatalan -> kirim notifikasi darurat.");
        digitalWrite(PIN_BUZZER, LOW);
        triggerEmergencyProcedure();
        currentState = STATE_ALARM_SENT;
      }
      // pembatalan ditangani di handleButton()
      break;
    }

    case STATE_ALARM_SENT: {
      // menunggu reset manual / restart sistem setelah insiden ditangani
      break;
    }
  }
}

// PENANGANAN TOMBOL FISIK
void handleButton(unsigned long now) {
  bool pressed = (digitalRead(PIN_BUTTON) == LOW);

  if (pressed && !buttonPressed) {
    buttonPressed = true;

    if (currentState == STATE_COUNTDOWN) {
      unsigned long elapsed = now - countdownStartMs;
      // pembatalan hanya valid pada jendela 3-30 detik (Bab 4.5.4)
      if (elapsed >= FALSE_ALARM_MIN_MS && elapsed < FALSE_ALARM_COUNTDOWN_MS) {
        Serial.println("[BATAL] False alarm dibatalkan oleh pengguna.");
        digitalWrite(PIN_BUZZER, LOW);
        currentState = STATE_NORMAL;
        peakG = 0;
      }
    }
    // TODO: tombol ditekan lama (long-press) -> aktifkan mode pairing BLE
  }

  if (!pressed) {
    buttonPressed = false;
  }
}


// PROSEDUR DARURAT (Bab 8.3 - Alur Komunikasi)
void triggerEmergencyProcedure() {
  // 1. Ambil koordinat GPS terkini
  double lat = 0, lng = 0;
  bool hasFix = gpsGetLocation(lat, lng);
  if (!hasFix) {
    Serial.println("[GPS] Fix tidak tersedia, kirim koordinat 0,0 sebagai fallback.");
  }

  // 2. Cek kualitas sinyal sebelum kirim
  int csq = gsmGetSignalQuality();
  Serial.print("[GSM] Kualitas sinyal (CSQ): ");
  Serial.println(csq);

  // 3. Susun & kirim SMS ke seluruh kontak darurat (dengan retry internal)
  int severity = computeSeverity(peakG);
  String message = buildAlertMessage(vehicleId, "Benturan/Kecelakaan", peakG, lat, lng, severity);

  if (emergencyContactCount == 0) {
    Serial.println("[WARNING] Tidak ada kontak darurat tersimpan! Konfigurasikan via app.");
    return;
  }

  int sentCount = gsmBroadcastSMS(emergencyContacts, emergencyContactCount, message);
  Serial.print("[GSM] SMS terkirim ke ");
  Serial.print(sentCount);
  Serial.print(" dari ");
  Serial.print(emergencyContactCount);
  Serial.println(" kontak.");

  // 4. Missed call ke kontak prioritas pertama
  if (emergencyContactCount > 0) {
    Serial.println("[GSM] Melakukan missed call ke kontak prioritas...");
    gsmMissedCall(emergencyContacts[0]);
  }

  // 5. TODO: simpan pre-event buffer (10 detik sebelum kejadian) ke SPIFFS
  //    -> perlu ring buffer terpisah yang terus merekam sample selama operasi normal
}


// UTIL
float readBatteryPercent() {
  int raw = analogRead(PIN_BATTERY_ADC);
  // TODO: kalibrasi sesuai voltage divider & kurva discharge baterai 18650
  float voltage = (raw / 4095.0f) * 3.3f * 2.0f;  // asumsi divider 1:1
  float pct = (voltage - 3.0f) / (4.2f - 3.0f) * 100.0f;
  return constrain(pct, 0.0f, 100.0f);
}
