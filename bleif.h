/* ============================================================================
 *  DriveSafe - bleif.h
 *  ESP32 sebagai BLE peripheral untuk aplikasi mobile DriveSafe
 *  (Bab 2.3.2.2, Bab 8.5, Sub-bab 10.2.1).
 *
 *  Layanan 4f1a0000-... dengan empat characteristic:
 *    CONFIG  (WRITE)         : konfigurasi JSON dari aplikasi -> NVS
 *    STATUS  (READ + NOTIFY) : status perangkat tiap 1 detik
 *    EVENT   (READ + NOTIFY) : detail kejadian kecelakaan
 *    CMD     (WRITE)         : perintah singkat (CANCEL, TEST_SMS, dll)
 *
 *  Setelah konfigurasi tersimpan di NVS, koneksi BLE tidak diperlukan lagi
 *  selama berkendara normal demi menghemat daya (Sub-bab 10.2.1).
 * ========================================================================== */
#ifndef DRIVESAFE_BLEIF_H
#define DRIVESAFE_BLEIF_H

#include "config.h"

/* Perintah dari aplikasi yang diteruskan ke mesin keadaan utama */
enum BleCommand : uint8_t {
  BLECMD_NONE = 0,
  BLECMD_CANCEL,          /* batalkan hitung mundur false alarm   */
  BLECMD_TEST_SMS,        /* uji kirim SMS ke kontak prioritas    */
  BLECMD_RECALIBRATE,     /* ulangi kalibrasi awal                */
  BLECMD_SIMULATE_CRASH,  /* pemicu buatan untuk pengujian        */
  BLECMD_CLEAR_CONTACTS
};

class DriveSafeBle {
public:
  void begin();
  void startPairing();            /* mulai advertising                 */
  void stopPairing();
  bool isAdvertising() const { return _advertising; }
  bool isConnected()   const { return _connected;   }
  String deviceName()  const { return _name;        }

  /* Kirim status periodik ke aplikasi (Sub-bab 10.2.4). */
  void notifyStatus(SystemState state, float speed_kmh, uint8_t battery_pct,
                    bool sensor_ok, bool gps_fix, uint8_t sat,
                    int gsm_pct, uint8_t contacts, uint32_t countdown_ms);

  /* Kirim detail kejadian ke aplikasi (Sub-bab 10.2.6 - 10.2.7). */
  void notifyEvent(const EventLog &ev, const char *maps_link, const char *phase);

  BleCommand consumeCommand();

private:
  String _name;
  bool   _connected   = false;
  bool   _advertising = false;
};

extern DriveSafeBle Ble;

#endif /* DRIVESAFE_BLEIF_H */
