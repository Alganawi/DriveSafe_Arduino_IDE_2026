/* ============================================================================
 *  DriveSafe - storage.h
 *  Penyimpanan konfigurasi permanen pada NVS ESP32 + event log pada LittleFS.
 *
 *  Sub-bab 4.6.3 / Pers. 4.31 : 5 kontak x 16 byte = 80 byte pada NVS.
 *  Bab 8.5                    : konfigurasi dari aplikasi disimpan ke NVS
 *                               agar tetap ada setelah perangkat dimatikan.
 *  Bab 8.3 poin 4             : pre-event buffer disalin ke flash internal.
 * ========================================================================== */
#ifndef DRIVESAFE_STORAGE_H
#define DRIVESAFE_STORAGE_H

#include "config.h"
#include "eventbuf.h"

struct EmergencyContact {
  char    name[CONTACT_NAME_LEN];
  char    number[CONTACT_NUMBER_LEN];   /* format E.164, mis. +6281234567890 */
  uint8_t priority;                     /* 1 = prioritas utama (missed call) */
  bool    active;
};

struct DeviceConfig {
  char             owner[OWNER_NAME_LEN];
  uint8_t          profile;             /* VehicleProfile                    */
  uint16_t         countdown_s;         /* preferensi countdown false alarm  */
  uint8_t          contact_count;
  EmergencyContact contacts[MAX_CONTACTS];
};

class DriveSafeStorage {
public:
  bool begin();

  DeviceConfig &config() { return _cfg; }
  bool save();
  bool load();
  void loadDefaults();

  /* Kontak darurat */
  bool addContact(const char *name, const char *number, uint8_t priority);
  bool removeContact(uint8_t index);
  void clearContacts();
  int8_t primaryContactIndex() const;       /* prioritas terkecil, -1 bila kosong */
  uint8_t contactCount() const { return _cfg.contact_count; }
  const EmergencyContact *contact(uint8_t i) const;

  /* Profil kendaraan */
  bool setProfileByName(const char *name);
  const char *profileName() const;

  /* Offset kalibrasi BNO055 (22 byte) */
  bool saveCalib(const uint8_t *buf22);
  bool loadCalib(uint8_t *buf22);
  bool hasCalib();

  /* Event log ke flash internal (opsional) */
  bool persistEvent(const EventLog &ev, const EventBuffer &buf);
  uint16_t storedEventCount();
  bool lastEvent(EventLog &ev);

  static uint32_t contactMemoryBytes() { return MAX_CONTACTS * CONTACT_NUMBER_LEN; }

private:
  DeviceConfig _cfg;
  bool _fsReady = false;
};

extern DriveSafeStorage Store;

#endif /* DRIVESAFE_STORAGE_H */
