#ifndef DETECTION_H
#define DETECTION_H

#include <Arduino.h>
#include <math.h>
#include "config.h"

// =====================================================================
// STRUKTUR DATA
// =====================================================================
struct SensorSample {
  float ax, ay, az;      // percepatan (g)
  float gx, gy, gz;      // kecepatan sudut (deg/s)
};

struct DerivedFeatures {
  float G;          // G-Force total (Persamaan 4.13)
  float J;           // jerk (Persamaan 4.14)
  float R;           // rotational jerk (Persamaan 4.15)
  float roll;         // sudut roll (deg) (Persamaan 4.21)
  float pitch;        // sudut pitch (deg) (Persamaan 4.22)
};

enum SystemState {
  STATE_NORMAL,
  STATE_VERIFICATION,     // fase konfirmasi pasca-spike
  STATE_CONFIRMED,        // kecelakaan terkonfirmasi -> mulai countdown
  STATE_COUNTDOWN,        // menunggu pembatalan pengguna
  STATE_ALARM_SENT        // notifikasi darurat sudah dikirim
};

// =====================================================================
// VARIABEL GLOBAL STATE (didefinisikan di file .ino utama)
// =====================================================================
extern SystemState currentState;
extern VehicleProfile activeProfile;
extern float lastG;
extern float lastGx, lastGy, lastGz;
extern float peakG;
extern unsigned long verificationStartMs;
extern unsigned long countdownStartMs;
extern unsigned long rolloverStartMs;
extern bool rolloverTimerActive;

// =====================================================================
// PERHITUNGAN FITUR TURUNAN
// =====================================================================
inline DerivedFeatures computeFeatures(const SensorSample &s) {
  DerivedFeatures f;

  // G-Force total (4.13)
  f.G = sqrtf(s.ax * s.ax + s.ay * s.ay + s.az * s.az);

  // Jerk (4.14)
  f.J = (f.G - lastG) / TS_SECONDS;

  // Rotational jerk (4.15)
  float dgx = (s.gx - lastGx) / TS_SECONDS;
  float dgy = (s.gy - lastGy) / TS_SECONDS;
  float dgz = (s.gz - lastGz) / TS_SECONDS;
  f.R = sqrtf(dgx * dgx + dgy * dgy + dgz * dgz);

  // Roll & pitch dari akselerometer (4.21, 4.22)
  f.roll  = atan2f(s.ay, s.az) * 180.0f / PI;
  f.pitch = atan2f(s.ax, sqrtf(s.ay * s.ay + s.az * s.az)) * 180.0f / PI;

  // simpan untuk sample berikutnya
  lastG  = f.G;
  lastGx = s.gx; lastGy = s.gy; lastGz = s.gz;

  return f;
}

// =====================================================================
// ADAPTIVE THRESHOLD (Persamaan 4.9, 4.10, 4.12)
// =====================================================================
inline float computeSlopeFactor(float pitchDeg) {
  float pitchRad = pitchDeg * PI / 180.0f;
  float c = cosf(pitchRad);
  if (fabsf(c) < 0.05f) c = 0.05f;   // guard pembagian ekstrem
  return 1.0f / fabsf(c);
}

inline float computeSpeedFactor(float gx, float gy, float gz) {
  float omegaMag = sqrtf(gx * gx + gy * gy + gz * gz);
  return 1.0f + BETA_SENSITIVITY * (omegaMag / OMEGA_REF_DEG_S);
}

inline float getAdaptiveGThreshold(const DerivedFeatures &f, const SensorSample &s) {
  float base = VEHICLE_THRESHOLDS[activeProfile].Gth_base;
  float kSlope = computeSlopeFactor(f.pitch);
  float kSpeed = computeSpeedFactor(s.gx, s.gy, s.gz);
  return base * kSlope * kSpeed;
}

// =====================================================================
// TAHAP 1 - DETEKSI KEJADIAN ABNORMAL (Persamaan 4.16)
// =====================================================================
inline bool detectEvent(const DerivedFeatures &f, const SensorSample &s) {
  float Gth = getAdaptiveGThreshold(f, s);
  float Jth = VEHICLE_THRESHOLDS[activeProfile].Jth_base;
  float Rth = VEHICLE_THRESHOLDS[activeProfile].Rth_base;

  bool overG = f.G > Gth;
  bool overJ = fabsf(f.J) > Jth;
  bool overR = f.R > Rth;

  return overG || overJ || overR;
}

// =====================================================================
// TAHAP 2 - VERIFIKASI: kendaraan diam (f2) atau orientasi abnormal (f3)
// =====================================================================
inline bool checkStillness(const SensorSample &s) {
  float gyroMag = sqrtf(s.gx * s.gx + s.gy * s.gy + s.gz * s.gz);
  bool accelNearGravity = (s.az > STILLNESS_G_MIN && s.az < STILLNESS_G_MAX);
  bool gyroLow = gyroMag < STILLNESS_GYRO_MAX;
  return accelNearGravity && gyroLow;
}

inline bool checkAbnormalOrientation(const DerivedFeatures &f) {
  return (fabsf(f.roll) > ROLL_THRESHOLD_DEG) || (fabsf(f.pitch) > PITCH_THRESHOLD_DEG);
}

// Deteksi rollover persisten (Persamaan 4.23), dipanggil tiap sample
inline bool checkRolloverPersistent(const DerivedFeatures &f, unsigned long nowMs) {
  bool exceeds = (fabsf(f.roll) > ROLLOVER_THRESHOLD_DEG) || (fabsf(f.pitch) > ROLLOVER_THRESHOLD_DEG);

  if (exceeds) {
    if (!rolloverTimerActive) {
      rolloverTimerActive = true;
      rolloverStartMs = nowMs;
    }
    if (nowMs - rolloverStartMs >= ROLLOVER_PERSIST_MS) {
      return true;
    }
  } else {
    rolloverTimerActive = false;
  }
  return false;
}

// =====================================================================
// ESTIMASI SEVERITY (Persamaan 4.23 bawah)
// =====================================================================
inline int computeSeverity(float GpeakVal) {
  if (GpeakVal < SEVERITY_MODERATE_G) return 0;      // Minor
  if (GpeakVal < SEVERITY_SEVERE_G)   return 1;      // Moderate
  return 2;                                          // Severe
}

#endif
