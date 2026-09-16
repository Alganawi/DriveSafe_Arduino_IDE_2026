/* ============================================================================
 *  DriveSafe - storage.cpp
 * ========================================================================== */
#include <Preferences.h>
#include <LittleFS.h>
#include <string.h>
#include <strings.h>
#include "storage.h"

DriveSafeStorage Store;
static Preferences prefs;

/* -------------------------------------------------------------------------- */
bool DriveSafeStorage::begin() {
  if (!prefs.begin(NVS_NAMESPACE, false)) {
    Serial.println(F("[NVS] Gagal membuka namespace"));
    loadDefaults();
    return false;
  }

  if (!load()) {
    Serial.println(F("[NVS] Konfigurasi belum ada, memakai nilai default"));
    loadDefaults();
    save();
  }

#if ENABLE_FLASH_LOG
  _fsReady = LittleFS.begin(true);
  if (!_fsReady) Serial.println(F("[FS] LittleFS gagal dimount"));
  else           Serial.println(F("[FS] LittleFS siap"));
#endif

  Serial.printf("[NVS] Profil=%s  Pemilik=%s  Kontak=%u/%u  Countdown=%us\n",
                profileName(), _cfg.owner, _cfg.contact_count,
                MAX_CONTACTS, _cfg.countdown_s);
  Serial.printf("[NVS] Alokasi kontak darurat: %lu byte (Pers. 4.31)\n",
                (unsigned long)contactMemoryBytes());
  return true;
}

void DriveSafeStorage::loadDefaults() {
  memset(&_cfg, 0, sizeof(_cfg));
  strncpy(_cfg.owner, "Pengguna", OWNER_NAME_LEN - 1);
  _cfg.profile       = PROFILE_MOTOR;
  _cfg.countdown_s   = SPARE_TOTAL_MS / 1000;
  _cfg.contact_count = 0;
}

bool DriveSafeStorage::save() {
  size_t n = prefs.putBytes("cfg", &_cfg, sizeof(_cfg));
  if (n != sizeof(_cfg)) {
    Serial.println(F("[NVS] Gagal menyimpan konfigurasi"));
    return false;
  }
  Serial.println(F("[NVS] Konfigurasi tersimpan"));
  return true;
}

bool DriveSafeStorage::load() {
  size_t len = prefs.getBytesLength("cfg");
  if (len != sizeof(_cfg)) return false;
  return prefs.getBytes("cfg", &_cfg, sizeof(_cfg)) == sizeof(_cfg);
}

/* -------------------------------------------------------------------------- */
/*  Kontak darurat (maksimal 5, Tabel 6.2 no.8)                               */
/* -------------------------------------------------------------------------- */
bool DriveSafeStorage::addContact(const char *name, const char *number,
                                  uint8_t priority) {
  if (_cfg.contact_count >= MAX_CONTACTS) {
    Serial.println(F("[NVS] Kontak penuh (maks 5)"));
    return false;
  }
  if (!number || strlen(number) < 8) return false;

  EmergencyContact &c = _cfg.contacts[_cfg.contact_count];
  memset(&c, 0, sizeof(c));
  strncpy(c.name,   name   ? name : "-", CONTACT_NAME_LEN   - 1);
  strncpy(c.number, number,              CONTACT_NUMBER_LEN - 1);
  c.priority = priority ? priority : (uint8_t)(_cfg.contact_count + 1);
  c.active   = true;
  _cfg.contact_count++;
  return true;
}

bool DriveSafeStorage::removeContact(uint8_t index) {
  if (index >= _cfg.contact_count) return false;
  for (uint8_t i = index; i + 1 < _cfg.contact_count; i++)
    _cfg.contacts[i] = _cfg.contacts[i + 1];
  _cfg.contact_count--;
  memset(&_cfg.contacts[_cfg.contact_count], 0, sizeof(EmergencyContact));
  return true;
}

void DriveSafeStorage::clearContacts() {
  memset(_cfg.contacts, 0, sizeof(_cfg.contacts));
  _cfg.contact_count = 0;
}

int8_t DriveSafeStorage::primaryContactIndex() const {
  int8_t  best = -1;
  uint8_t bestPrio = 255;
  for (uint8_t i = 0; i < _cfg.contact_count; i++) {
    if (!_cfg.contacts[i].active) continue;
    if (_cfg.contacts[i].priority < bestPrio) {
      bestPrio = _cfg.contacts[i].priority;
      best = (int8_t)i;
    }
  }
  return best;
}

const EmergencyContact *DriveSafeStorage::contact(uint8_t i) const {
  if (i >= _cfg.contact_count) return nullptr;
  return &_cfg.contacts[i];
}

/* -------------------------------------------------------------------------- */
bool DriveSafeStorage::setProfileByName(const char *name) {
  if (!name) return false;
  for (uint8_t i = 0; i < PROFILE_COUNT; i++) {
    if (strcasecmp(name, PROFILE_TABLE[i].name) == 0) {
      _cfg.profile = i;
      return true;
    }
  }
  /* sinonim yang umum dipakai aplikasi */
  if (strcasecmp(name, "car")        == 0) { _cfg.profile = PROFILE_MOBIL;  return true; }
  if (strcasecmp(name, "motorcycle") == 0 ||
      strcasecmp(name, "motor")      == 0) { _cfg.profile = PROFILE_MOTOR;  return true; }
  if (strcasecmp(name, "bike")       == 0 ||
      strcasecmp(name, "bicycle")    == 0) { _cfg.profile = PROFILE_SEPEDA; return true; }
  return false;
}

const char *DriveSafeStorage::profileName() const {
  uint8_t p = _cfg.profile < PROFILE_COUNT ? _cfg.profile : PROFILE_MOTOR;
  return PROFILE_TABLE[p].name;
}

/* -------------------------------------------------------------------------- */
/*  Offset kalibrasi BNO055                                                   */
/* -------------------------------------------------------------------------- */
bool DriveSafeStorage::saveCalib(const uint8_t *buf22) {
  return prefs.putBytes("calib", buf22, 22) == 22;
}

bool DriveSafeStorage::loadCalib(uint8_t *buf22) {
  return prefs.getBytes("calib", buf22, 22) == 22;
}

bool DriveSafeStorage::hasCalib() {
  return prefs.getBytesLength("calib") == 22;
}

/* -------------------------------------------------------------------------- */
/*  Event log ke flash internal (Bab 8.3 poin 4)                              */
/* -------------------------------------------------------------------------- */
bool DriveSafeStorage::persistEvent(const EventLog &ev, const EventBuffer &buf) {
#if !ENABLE_FLASH_LOG
  (void)ev; (void)buf;
  return false;
#else
  if (!_fsReady) return false;

  uint16_t idx = prefs.getUShort("evidx", 0);
  char path[24];
  snprintf(path, sizeof(path), "/ev_%u.bin", (unsigned)(idx % MAX_FLASH_EVENTS));

  File f = LittleFS.open(path, "w");
  if (!f) { Serial.println(F("[FS] Gagal membuka file event")); return false; }

  /* Header: magic + metadata (Tabel 4.4) */
  const uint32_t magic = 0x44535631;        /* "DSV1" */
  f.write((const uint8_t *)&magic, sizeof(magic));
  f.write((const uint8_t *)&ev,    sizeof(ev));

  uint16_t total = buf.totalSamples();
  f.write((const uint8_t *)&total, sizeof(total));

  SensorSample s;
  for (uint16_t i = 0; i < total; i++)
    if (buf.at(i, s)) f.write((const uint8_t *)&s, sizeof(s));
  f.close();

  /* Ringkasan CSV agar mudah dibaca manusia */
  File c = LittleFS.open("/events.csv", "a");
  if (c) {
    if (c.size() == 0)
      c.println(F("ts,g_peak,j_peak,r_peak,lat,lon,roll,pitch,severity,type,profile,flags"));
    c.printf("%lu,%.2f,%.1f,%.1f,%.6f,%.6f,%.1f,%.1f,%u,%u,%u,0x%02X\n",
             (unsigned long)ev.timestamp, ev.g_peak, ev.j_peak, ev.r_peak,
             ev.lat, ev.lon, ev.roll_peak, ev.pitch_peak,
             ev.severity, ev.crash_type, ev.profile, ev.trigger_flags);
    c.close();
  }

  prefs.putUShort("evidx", (uint16_t)(idx + 1));
  Serial.printf("[FS] Event log tersimpan: %s (%u sampel)\n", path, total);
  return true;
#endif
}

uint16_t DriveSafeStorage::storedEventCount() {
  uint16_t idx = prefs.getUShort("evidx", 0);
  return idx < MAX_FLASH_EVENTS ? idx : MAX_FLASH_EVENTS;
}

bool DriveSafeStorage::lastEvent(EventLog &ev) {
#if !ENABLE_FLASH_LOG
  (void)ev;
  return false;
#else
  if (!_fsReady) return false;
  uint16_t idx = prefs.getUShort("evidx", 0);
  if (idx == 0) return false;

  char path[24];
  snprintf(path, sizeof(path), "/ev_%u.bin", (unsigned)((idx - 1) % MAX_FLASH_EVENTS));
  File f = LittleFS.open(path, "r");
  if (!f) return false;

  uint32_t magic = 0;
  f.read((uint8_t *)&magic, sizeof(magic));
  if (magic != 0x44535631) { f.close(); return false; }
  bool ok = f.read((uint8_t *)&ev, sizeof(ev)) == sizeof(ev);
  f.close();
  return ok;
#endif
}
