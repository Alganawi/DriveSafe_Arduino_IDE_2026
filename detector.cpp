/* ============================================================================
 *  DriveSafe - detector.cpp
 * ========================================================================== */
#include <math.h>
#include "detector.h"

CrashDetector Detector;

void CrashDetector::begin(VehicleProfile profile) {
  setProfile(profile);
  _hasPrev = false;
  resetVerification();
}

void CrashDetector::setProfile(VehicleProfile profile) {
  if (profile >= PROFILE_COUNT) profile = PROFILE_MOTOR;
  _profile = profile;
}

void CrashDetector::resetVerification() {
  _still_since    = 0;
  _orient_since   = 0;
  _rollover_since = 0;
}

/* -------------------------------------------------------------------------- */
/*  Fitur turunan + threshold adaptif                                         */
/* -------------------------------------------------------------------------- */
void CrashDetector::computeFeatures(const SensorSample &s, float roll,
                                    float pitch, float yaw, SensorFeature &f) {
  /* Pers. 4.13 : magnitudo percepatan total, lalu Pers. 5.3 ke satuan g */
  f.a_mag   = sqrtf(s.ax * s.ax + s.ay * s.ay + s.az * s.az);
  f.g_force = f.a_mag / GRAVITY_MS2;

  /* Pers. 4.14 : jerk linear (m/s^3) */
  if (_hasPrev) f.jerk = fabsf(f.a_mag - _prev_amag) / TS_SECONDS;
  else          f.jerk = 0.0f;

  /* Pers. 4.15 : rotational jerk (deg/s^2) */
  if (_hasPrev) {
    const float dx = (s.gx - _prev_gx) / TS_SECONDS;
    const float dy = (s.gy - _prev_gy) / TS_SECONDS;
    const float dz = (s.gz - _prev_gz) / TS_SECONDS;
    f.rot_jerk = sqrtf(dx * dx + dy * dy + dz * dz);
  } else {
    f.rot_jerk = 0.0f;
  }

  f.omega_mag = sqrtf(s.gx * s.gx + s.gy * s.gy + s.gz * s.gz);
  f.roll  = roll;
  f.pitch = pitch;
  f.yaw   = yaw;

  /* ---- Threshold adaptif (Pers. 4.9 - 4.12) --------------------------- */

  /* Pers. 4.11 : sudut kemiringan dari akselerometer */
  const float theta_pitch = atan2f(s.ax, sqrtf(s.ay * s.ay + s.az * s.az));

  /* Pers. 4.12 : k_slope = 1 / cos(theta_pitch), dibatasi agar tidak divergen
     saat theta mendekati 90 derajat (cos -> 0). */
  float c = cosf(theta_pitch);
  if (fabsf(c) < 0.05f) c = 0.05f;
  f.k_slope = 1.0f / fabsf(c);
  if (f.k_slope > K_SLOPE_MAX) f.k_slope = K_SLOPE_MAX;
  if (f.k_slope < 1.0f)        f.k_slope = 1.0f;

  /* Pers. 4.10 : k_speed = 1 + beta * |omega| / omega_ref */
  f.k_speed = 1.0f + BETA_SENSITIVITY * (f.omega_mag / OMEGA_REF_DPS);
  if (f.k_speed > K_SPEED_MAX) f.k_speed = K_SPEED_MAX;

  /* Pers. 4.9 */
  f.g_threshold = PROFILE_TABLE[_profile].g_th * f.k_slope * f.k_speed;

  /* Simpan state untuk sampel berikutnya */
  _prev_amag = f.a_mag;
  _prev_gx = s.gx; _prev_gy = s.gy; _prev_gz = s.gz;
  _hasPrev = true;
}

/* -------------------------------------------------------------------------- */
/*  Tahap 1 : deteksi kejadian abnormal, f1[n]  (Pers. 4.16)                   */
/* -------------------------------------------------------------------------- */
bool CrashDetector::stageEventDetection(const SensorSample &s,
                                        const SensorFeature &f,
                                        uint8_t &flags_out) {
  uint8_t flags = 0;

  if (f.g_force  > f.g_threshold)     flags |= TRIG_G;
  if (f.jerk     > jerkThreshold())   flags |= TRIG_J;
  if (f.rot_jerk > rotJerkThreshold())flags |= TRIG_R;

  /* Tabel 5.7 : shock bila |ax| DAN |ay| sama-sama melewati 12 m/s^2 */
  if (fabsf(s.ax) > SHOCK_AXIS_TH_MS2 && fabsf(s.ay) > SHOCK_AXIS_TH_MS2)
    flags |= TRIG_SHOCK;

  flags_out = flags;
  return flags != 0;
}

/* -------------------------------------------------------------------------- */
/*  Tahap 2 : f2[n] - kendaraan diam pascabenturan (3 detik berturut-turut)    */
/* -------------------------------------------------------------------------- */
bool CrashDetector::stageStillness(const SensorFeature &f, uint32_t now_ms) {
  const bool still_now = (fabsf(f.g_force - 1.0f) < STILL_G_TOLERANCE) &&
                         (f.omega_mag < STILL_GYRO_TH_DPS);

  if (!still_now) { _still_since = 0; return false; }
  if (_still_since == 0) { _still_since = now_ms; return false; }
  return (now_ms - _still_since) >= VERIFY_STILL_MS;
}

/* -------------------------------------------------------------------------- */
/*  Tahap 3 : f3[n] - orientasi abnormal (2 detik berturut-turut)              */
/*  Sub-bab 4.5.3 : |roll| > 60 deg  ATAU  |pitch| >= 90 deg                   */
/* -------------------------------------------------------------------------- */
bool CrashDetector::stageAbnormalOrientation(const SensorFeature &f,
                                             uint32_t now_ms) {
  const bool abnormal = (fabsf(f.roll)  >  ROLL_ANGLE_TH_DEG) ||
                        (fabsf(f.pitch) >= PITCH_ANGLE_TH_DEG);

  if (!abnormal) { _orient_since = 0; return false; }
  if (_orient_since == 0) { _orient_since = now_ms; return false; }
  return (now_ms - _orient_since) >= VERIFY_ORIENT_MS;
}

/* -------------------------------------------------------------------------- */
/*  Jalur paralel : overturn / rollover persisten (Pers. 4.23)                 */
/*  Bekerja independen dan dapat memicu notifikasi darurat sendiri.            */
/* -------------------------------------------------------------------------- */
bool CrashDetector::checkRolloverPersistent(const SensorFeature &f,
                                            uint32_t now_ms) {
  /* Tabel 5.7 : overturn bila |phi| DAN |theta| sama-sama >= 60 deg.
     Sub-bab 4.5.3 juga menerima salah satu sudut melampaui 60 deg secara
     persisten; kondisi OR dipakai sebagai pemicu utama karena lebih
     konservatif terhadap false negative (prioritas recall, Tabel 6.2 no.1). */
  const bool tilted = (fabsf(f.roll)  > OVERTURN_TH_DEG) ||
                      (fabsf(f.pitch) > OVERTURN_TH_DEG);

  if (!tilted) { _rollover_since = 0; return false; }
  if (_rollover_since == 0) { _rollover_since = now_ms; return false; }
  return (now_ms - _rollover_since) >= ROLLOVER_PERSIST_MS;
}

/* -------------------------------------------------------------------------- */
/*  Klasifikasi                                                               */
/* -------------------------------------------------------------------------- */
uint8_t CrashDetector::classifySeverity(float g_peak) {
  if (g_peak < 4.0f) return SEVERITY_MINOR;
  if (g_peak < 7.0f) return SEVERITY_MODERATE;
  return SEVERITY_SEVERE;
}

uint8_t CrashDetector::classifyCrashType(float ax_peak, float ay_peak,
                                         float roll_peak, float pitch_peak,
                                         bool rollover) {
  if (rollover) return CRASH_ROLLOVER;

  if (fabsf(roll_peak) > ROLL_ANGLE_TH_DEG ||
      fabsf(pitch_peak) > ROLL_ANGLE_TH_DEG)
    return CRASH_FALL;

  /* Sumbu X sejajar arah gerak kendaraan (Bab 8.2.4).
     Perlambatan mendadak (ax negatif besar) -> tabrakan frontal.
     Percepatan mendadak (ax positif besar)  -> ditabrak dari belakang. */
  if (fabsf(ax_peak) > fabsf(ay_peak))
    return (ax_peak < 0) ? CRASH_FRONTAL : CRASH_REAR;

  return CRASH_SIDE;
}
