/* ============================================================================
 *  DriveSafe - detector.h
 *  Algoritma deteksi kecelakaan rule-based multi-tahap (Gambar 4.3).
 *
 *  Mengimplementasikan:
 *    Pers. 4.13  G[n]  = sqrt(ax^2 + ay^2 + az^2)               (G-Force)
 *    Pers. 4.14  J[n]  = (|a|[n] - |a|[n-1]) / Ts               (jerk)
 *    Pers. 4.15  R[n]  = || (w[n] - w[n-1]) / Ts ||             (rotational jerk)
 *    Pers. 4.9   Gth[n] = Gth,base * k_slope(n) * k_speed(n)
 *    Pers. 4.10  k_speed = 1 + beta * |w| / w_ref
 *    Pers. 4.11  theta_pitch = arctan(ax / sqrt(ay^2 + az^2))
 *    Pers. 4.12  k_slope = 1 / cos(theta_pitch)
 *    Pers. 4.16  f1[n] = (G>Gth) OR (J>Jth) OR (R>Rth)
 *    Pers. 4.17  f[n]  = f1[n] AND (f2[n] OR f3[n])
 *    Pers. 4.23  rollover persisten 1,5 s  &  klasifikasi severity
 *    Tabel 5.7   threshold shock, roll, pitch, overturn
 * ========================================================================== */
#ifndef DRIVESAFE_DETECTOR_H
#define DRIVESAFE_DETECTOR_H

#include "config.h"

struct DetectionResult {
  bool          f1;              /* deteksi kejadian abnormal        */
  bool          f2;              /* konfirmasi kendaraan diam        */
  bool          f3;              /* konfirmasi orientasi abnormal    */
  bool          confirmed;       /* f[n] = f1 AND (f2 OR f3)         */
  bool          rollover;        /* jalur independen (Sub-bab 4.5.3) */
  uint8_t       trigger_flags;
};

class CrashDetector {
public:
  void  begin(VehicleProfile profile);
  void  setProfile(VehicleProfile profile);
  VehicleProfile profile() const { return _profile; }
  const ProfileThreshold &thresholds() const { return PROFILE_TABLE[_profile]; }

  /* Hitung fitur turunan + threshold adaptif untuk satu cuplikan. */
  void  computeFeatures(const SensorSample &s, float roll, float pitch,
                        float yaw, SensorFeature &f);

  /* Tahap 1 : f1[n] (Pers. 4.16) + shock Tabel 5.7 */
  bool  stageEventDetection(const SensorSample &s, const SensorFeature &f,
                            uint8_t &flags_out);

  /* Tahap 2 : f2[n] kendaraan diam pascabenturan */
  bool  stageStillness(const SensorFeature &f, uint32_t now_ms);

  /* Tahap 3 : f3[n] orientasi abnormal */
  bool  stageAbnormalOrientation(const SensorFeature &f, uint32_t now_ms);

  /* Jalur paralel : rollover / overturn persisten 1,5 s (Pers. 4.23) */
  bool  checkRolloverPersistent(const SensorFeature &f, uint32_t now_ms);

  /* Reset penghitung persistensi saat masuk fase verifikasi baru */
  void  resetVerification();

  /* Threshold efektif setelah pengali kalibrasi empiris diterapkan */
  float jerkThreshold()    const { return PROFILE_TABLE[_profile].j_th * JERK_THRESHOLD_SCALE; }
  float rotJerkThreshold() const { return PROFILE_TABLE[_profile].r_th * ROTJERK_THRESHOLD_SCALE; }

  /* Klasifikasi (Pers. 4.23 & Bab 5.7) */
  static uint8_t classifySeverity(float g_peak);
  static uint8_t classifyCrashType(float ax_peak, float ay_peak,
                                   float roll_peak, float pitch_peak,
                                   bool rollover);

private:
  VehicleProfile _profile   = PROFILE_MOTOR;
  bool     _hasPrev         = false;
  float    _prev_amag       = 0;
  float    _prev_gx = 0, _prev_gy = 0, _prev_gz = 0;

  uint32_t _still_since     = 0;
  uint32_t _orient_since    = 0;
  uint32_t _rollover_since  = 0;
};

extern CrashDetector Detector;

#endif /* DRIVESAFE_DETECTOR_H */
