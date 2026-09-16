/* ============================================================================
 *  DriveSafe - gps.cpp
 * ========================================================================== */
#include <TinyGPSPlus.h>
#include "gps.h"

DriveSafeGps Gps;

static TinyGPSPlus   nmea;
static HardwareSerial GpsSerial(1);          /* UART1 */

/* Konversi tanggal/waktu UTC ke epoch Unix (tanpa detik kabisat). */
static uint32_t toEpoch(uint16_t year, uint8_t month, uint8_t day,
                        uint8_t hour, uint8_t minute, uint8_t second) {
  if (year < 1970 || month == 0 || month > 12) return 0;

  static const uint16_t cum[12] = {0,31,59,90,120,151,181,212,243,273,304,334};
  uint32_t days = (year - 1970) * 365UL;
  days += (year - 1969) / 4;                       /* koreksi tahun kabisat  */
  days -= (year - 1901) / 100;
  days += (year - 1601) / 400;
  days += cum[month - 1];

  const bool leap = ((year % 4 == 0 && year % 100 != 0) || year % 400 == 0);
  if (leap && month > 2) days += 1;
  days += (day - 1);

  return days * 86400UL + hour * 3600UL + minute * 60UL + second;
}

void DriveSafeGps::begin() {
  GpsSerial.setRxBufferSize(1024);          /* wajib sebelum begin()        */
  GpsSerial.begin(GPS_BAUD, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
  Serial.printf("[GPS] UART1 aktif pada RX=%d TX=%d @%lu bps\n",
                PIN_GPS_RX, PIN_GPS_TX, (unsigned long)GPS_BAUD);
}

void DriveSafeGps::poll() {
  while (GpsSerial.available()) nmea.encode((char)GpsSerial.read());

  const bool fresh = nmea.location.isValid() && nmea.location.age() < 3000;

  _live.valid      = fresh;
  _live.satellites = nmea.satellites.isValid() ? (uint8_t)nmea.satellites.value() : 0;
  _live.hdop       = nmea.hdop.isValid() ? (float)(nmea.hdop.value() / 100.0) : 99.9f;
  _live.speed_kmh  = nmea.speed.isValid() ? (float)nmea.speed.kmph() : 0.0f;

  if (fresh) {
    _live.lat    = nmea.location.lat();
    _live.lon    = nmea.location.lng();
    _live.age_ms = nmea.location.age();

    if (nmea.date.isValid() && nmea.time.isValid() && nmea.date.year() >= 2020) {
      _live.epoch = toEpoch(nmea.date.year(), nmea.date.month(), nmea.date.day(),
                            nmea.time.hour(), nmea.time.minute(), nmea.time.second());
    }

    _last      = _live;
    _lastFixMs = millis();
  }
}

uint32_t DriveSafeGps::epochNow() {
  if (_last.epoch == 0) return 0;
  /* Ekstrapolasi dari fix terakhir memakai jam sistem */
  return _last.epoch + (millis() - _lastFixMs) / 1000UL;
}

String DriveSafeGps::coordString() {
  const GpsFix f = _last.valid ? _last : _live;
  if (!f.valid) return String("0,0");
  char buf[32];
  snprintf(buf, sizeof(buf), "%.6f,%.6f", f.lat, f.lon);
  return String(buf);
}

String DriveSafeGps::mapsLink() {
  const GpsFix f = _last.valid ? _last : _live;
  if (!f.valid) return String("LOKASI TIDAK TERSEDIA");
  char buf[64];
  /* Format pendek agar hemat karakter pada SMS 160 karakter */
  snprintf(buf, sizeof(buf), "maps.google.com/?q=%.5f,%.5f", f.lat, f.lon);
  return String(buf);
}
