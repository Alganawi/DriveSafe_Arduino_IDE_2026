/* ============================================================================
 *  DriveSafe - bleif.cpp
 * ========================================================================== */
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <string.h>
#include <strings.h>
#include <stdlib.h>

#include "bleif.h"
#include "storage.h"
#include "detector.h"

DriveSafeBle Ble;

static BLEServer         *g_server   = nullptr;
static BLECharacteristic *g_chrConfig = nullptr;
static BLECharacteristic *g_chrStatus = nullptr;
static BLECharacteristic *g_chrEvent  = nullptr;
static BLECharacteristic *g_chrCmd    = nullptr;

static volatile bool     g_connected  = false;
static volatile uint8_t  g_command    = BLECMD_NONE;

/* Buffer akumulasi konfigurasi: paket BLE dipecah oleh MTU, sehingga
   potongan dikumpulkan sampai jumlah '{' dan '}' seimbang. */
static char     g_rx[BLE_RX_BUFFER_LEN];
static uint16_t g_rxLen = 0;

/* ==========================================================================
 *  Parser JSON ringan
 *  Dirancang khusus untuk skema konfigurasi DriveSafe agar firmware tidak
 *  bergantung pada pustaka eksternal dan tetap hemat memori.
 * ========================================================================== */
static const char *jsonFindKey(const char *src, const char *key) {
  char pat[40];
  snprintf(pat, sizeof(pat), "\"%s\"", key);
  const char *p = strstr(src, pat);
  if (!p) return nullptr;
  p += strlen(pat);
  while (*p == ' ' || *p == ':') p++;
  return p;
}

static bool jsonGetString(const char *src, const char *key,
                          char *out, size_t outLen) {
  const char *p = jsonFindKey(src, key);
  if (!p || *p != '"') return false;
  p++;
  size_t i = 0;
  while (*p && *p != '"' && i + 1 < outLen) {
    if (*p == '\\' && *(p + 1)) p++;        /* lewati escape sederhana      */
    out[i++] = *p++;
  }
  out[i] = '\0';
  return i > 0;
}

static bool jsonGetInt(const char *src, const char *key, long &out) {
  const char *p = jsonFindKey(src, key);
  if (!p) return false;
  if (*p == '"') p++;
  char *end = nullptr;
  long v = strtol(p, &end, 10);
  if (end == p) return false;
  out = v;
  return true;
}

/* ==========================================================================
 *  Penerapan konfigurasi dari aplikasi
 *
 *  Format yang diharapkan (satu paket, boleh terpecah beberapa write):
 *  {"owner":"Lisa","profile":"motor","countdown":30,
 *   "contacts":[{"n":"Ayah","p":"+628123456789","pr":1},
 *               {"n":"Ibu","p":"+628987654321","pr":2}]}
 * ========================================================================== */
static void applyConfigJson(const char *json) {
  DeviceConfig &cfg = Store.config();
  bool changed = false;

  char tmp[40];
  if (jsonGetString(json, "owner", tmp, sizeof(tmp))) {
    strncpy(cfg.owner, tmp, OWNER_NAME_LEN - 1);
    cfg.owner[OWNER_NAME_LEN - 1] = '\0';
    changed = true;
  }

  if (jsonGetString(json, "profile", tmp, sizeof(tmp))) {
    if (Store.setProfileByName(tmp)) {
      Detector.setProfile((VehicleProfile)cfg.profile);
      changed = true;
    }
  }

  long v;
  if (jsonGetInt(json, "countdown", v)) {
    if (v >= 5 && v <= 120) { cfg.countdown_s = (uint16_t)v; changed = true; }
  }

  /* Daftar kontak: bila kunci "contacts" ada, daftar lama diganti penuh. */
  const char *arr = jsonFindKey(json, "contacts");
  if (arr && *arr == '[') {
    Store.clearContacts();

    const char *p = arr;
    while ((p = strchr(p, '{')) != nullptr) {
      const char *end = strchr(p, '}');
      if (!end) break;

      const size_t len = (size_t)(end - p + 1);
      if (len < sizeof(tmp) * 3) {
        char obj[96];
        size_t n = len < sizeof(obj) - 1 ? len : sizeof(obj) - 1;
        memcpy(obj, p, n);
        obj[n] = '\0';

        char name[CONTACT_NAME_LEN]   = "-";
        char num[CONTACT_NUMBER_LEN]  = "";
        long prio = 0;

        jsonGetString(obj, "n", name, sizeof(name));
        jsonGetString(obj, "p", num,  sizeof(num));
        jsonGetInt(obj, "pr", prio);

        if (strlen(num) >= 8) Store.addContact(name, num, (uint8_t)prio);
      }
      p = end + 1;
      if (Store.contactCount() >= MAX_CONTACTS) break;
    }
    changed = true;
  }

  if (changed) {
    Store.save();
    Serial.printf("[BLE] Konfigurasi diterapkan: profil=%s, kontak=%u, "
                  "countdown=%us\n",
                  Store.profileName(), Store.contactCount(), cfg.countdown_s);
  }
}

/* ==========================================================================
 *  Callback BLE
 * ========================================================================== */
class ServerCb : public BLEServerCallbacks {
  void onConnect(BLEServer *) override {
    g_connected = true;
    g_rxLen = 0;
    Serial.println(F("[BLE] Aplikasi terhubung"));
  }
  void onDisconnect(BLEServer *s) override {
    g_connected = false;
    Serial.println(F("[BLE] Aplikasi terputus, lanjutkan advertising"));
    s->startAdvertising();
  }
};

class ConfigCb : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    /* getData()/getLength() tersedia pada ESP32 core 2.x maupun 3.x */
    const uint8_t *data = c->getData();
    const size_t   len  = c->getLength();
    if (!data || len == 0) return;

    for (size_t i = 0; i < len && g_rxLen < BLE_RX_BUFFER_LEN - 1; i++)
      g_rx[g_rxLen++] = (char)data[i];
    g_rx[g_rxLen] = '\0';

    /* Cek kelengkapan objek JSON */
    int depth = 0;
    bool inStr = false;
    for (uint16_t i = 0; i < g_rxLen; i++) {
      const char ch = g_rx[i];
      if (ch == '"' && (i == 0 || g_rx[i - 1] != '\\')) inStr = !inStr;
      if (inStr) continue;
      if (ch == '{') depth++;
      else if (ch == '}') depth--;
    }

    if (g_rxLen > 1 && depth == 0 && strchr(g_rx, '{')) {
      applyConfigJson(g_rx);
      g_rxLen = 0;
    } else if (g_rxLen >= BLE_RX_BUFFER_LEN - 2) {
      Serial.println(F("[BLE] Buffer konfigurasi penuh, dibuang"));
      g_rxLen = 0;
    }
  }
};

class CmdCb : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    const uint8_t *data = c->getData();
    const size_t   len  = c->getLength();
    if (!data || len == 0) return;

    char cmd[24] = {0};
    const size_t n = len < sizeof(cmd) - 1 ? len : sizeof(cmd) - 1;
    memcpy(cmd, data, n);

    Serial.printf("[BLE] Perintah diterima: %s\n", cmd);

    if      (strncasecmp(cmd, "CANCEL",    6) == 0) g_command = BLECMD_CANCEL;
    else if (strncasecmp(cmd, "TEST_SMS",  8) == 0) g_command = BLECMD_TEST_SMS;
    else if (strncasecmp(cmd, "RECAL",     5) == 0) g_command = BLECMD_RECALIBRATE;
    else if (strncasecmp(cmd, "SIMCRASH",  8) == 0) g_command = BLECMD_SIMULATE_CRASH;
    else if (strncasecmp(cmd, "CLEARCON",  8) == 0) g_command = BLECMD_CLEAR_CONTACTS;
  }
};

/* ==========================================================================
 *  API publik
 * ========================================================================== */
void DriveSafeBle::begin() {
  /* Nama perangkat "DriveSafe-XXXX" dengan 4 digit terakhir MAC (Bab 8.5) */
  const uint64_t mac = ESP.getEfuseMac();
  char suffix[8];
  snprintf(suffix, sizeof(suffix), "%04X", (unsigned)(mac & 0xFFFF));
  _name = String(BLE_DEVICE_PREFIX) + suffix;

  BLEDevice::init(_name.c_str());
  BLEDevice::setMTU(247);

  g_server = BLEDevice::createServer();
  g_server->setCallbacks(new ServerCb());

  BLEService *svc = g_server->createService(BLE_SVC_UUID);

  g_chrConfig = svc->createCharacteristic(
      BLE_CHR_CONFIG_UUID,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  g_chrConfig->setCallbacks(new ConfigCb());

  g_chrStatus = svc->createCharacteristic(
      BLE_CHR_STATUS_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  g_chrStatus->addDescriptor(new BLE2902());

  g_chrEvent = svc->createCharacteristic(
      BLE_CHR_EVENT_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  g_chrEvent->addDescriptor(new BLE2902());

  g_chrCmd = svc->createCharacteristic(
      BLE_CHR_CMD_UUID,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  g_chrCmd->setCallbacks(new CmdCb());

  svc->start();

  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(BLE_SVC_UUID);
  adv->setScanResponse(true);
  adv->setMinPreferred(0x06);
  adv->setMinPreferred(0x12);

  Serial.printf("[BLE] Peripheral siap sebagai \"%s\"\n", _name.c_str());
}

void DriveSafeBle::startPairing() {
  if (_advertising) return;
  BLEDevice::startAdvertising();
  _advertising = true;
  Serial.println(F("[BLE] Mode pairing aktif (advertising)"));
}

void DriveSafeBle::stopPairing() {
  if (!_advertising) return;
  BLEDevice::stopAdvertising();
  _advertising = false;
  Serial.println(F("[BLE] Mode pairing dimatikan"));
}

BleCommand DriveSafeBle::consumeCommand() {
  const uint8_t c = g_command;
  g_command = BLECMD_NONE;
  return (BleCommand)c;
}

void DriveSafeBle::notifyStatus(SystemState state, float speed_kmh,
                                uint8_t battery_pct, bool sensor_ok,
                                bool gps_fix, uint8_t sat, int gsm_pct,
                                uint8_t contacts, uint32_t countdown_ms) {
  _connected = g_connected;
  if (!g_connected || !g_chrStatus) return;

  char buf[220];
  const int n = snprintf(buf, sizeof(buf),
      "{\"st\":\"%s\",\"bat\":%u,\"sens\":%u,\"gps\":%u,\"sat\":%u,"
      "\"gsm\":%d,\"spd\":%.1f,\"con\":%u,\"cd\":%lu,\"prof\":\"%s\"}",
      stateName(state), battery_pct, sensor_ok ? 1 : 0, gps_fix ? 1 : 0,
      sat, gsm_pct, speed_kmh, contacts,
      (unsigned long)(countdown_ms / 1000), Store.profileName());

  if (n <= 0) return;
  g_chrStatus->setValue((uint8_t *)buf, (size_t)n);
  g_chrStatus->notify();
}

void DriveSafeBle::notifyEvent(const EventLog &ev, const char *maps_link,
                               const char *phase) {
  if (!g_chrEvent) return;

  char buf[300];
  const int n = snprintf(buf, sizeof(buf),
      "{\"ph\":\"%s\",\"ts\":%lu,\"g\":%.2f,\"j\":%.1f,\"r\":%.1f,"
      "\"roll\":%.1f,\"pitch\":%.1f,\"sev\":%u,\"sevn\":\"%s\","
      "\"type\":%u,\"typen\":\"%s\",\"lat\":%.6f,\"lon\":%.6f,\"map\":\"%s\"}",
      phase ? phase : "-", (unsigned long)ev.timestamp, ev.g_peak, ev.j_peak,
      ev.r_peak, ev.roll_peak, ev.pitch_peak,
      ev.severity, severityName(ev.severity),
      ev.crash_type, crashTypeName(ev.crash_type),
      ev.lat, ev.lon, maps_link ? maps_link : "");

  if (n <= 0) return;
  g_chrEvent->setValue((uint8_t *)buf, (size_t)n);
  if (g_connected) g_chrEvent->notify();
}
