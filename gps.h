/* ============================================================================
 *  DriveSafe - gps.h
 *  Akuisisi koordinat dari modul GNSS NEO-M8N melalui UART (Bab 8.3 poin 1).
 *
 *  Menyediakan cache "last known good fix" agar koordinat tetap dapat
 *  dikirimkan ketika kecelakaan terjadi di area tanpa sinyal terbuka
 *  (terowongan, basement, kanopi lebat) sesuai batasan Bab 7.3.
 * ========================================================================== */
#ifndef DRIVESAFE_GPS_H
#define DRIVESAFE_GPS_H

#include "config.h"

struct GpsFix {
  double   lat       = 0;
  double   lon       = 0;
  float    speed_kmh = 0;
  float    hdop      = 99.9f;
  uint8_t  satellites= 0;
  uint32_t epoch     = 0;      /* detik sejak 1 Jan 1970 (UTC)  */
  bool     valid     = false;
  uint32_t age_ms    = 0;      /* usia fix saat diambil          */
};

class DriveSafeGps {
public:
  void begin();
  void poll();                 /* wajib dipanggil rutin agar UART tak overflow */

  bool   hasFix()      const { return _live.valid; }
  bool   hasAnyFix()   const { return _last.valid; }
  GpsFix current()     const { return _live; }
  GpsFix lastKnown()   const { return _last; }

  float  speedKmh()    const { return _live.valid ? _live.speed_kmh : 0.0f; }
  uint8_t satellites() const { return _live.satellites; }
  uint32_t epochNow();

  /* Tautan Google Maps untuk disisipkan ke SMS (Tabel 6.2 no.5) */
  String mapsLink();
  String coordString();

private:
  GpsFix _live;
  GpsFix _last;
  uint32_t _lastFixMs = 0;
};

extern DriveSafeGps Gps;

#endif /* DRIVESAFE_GPS_H */
