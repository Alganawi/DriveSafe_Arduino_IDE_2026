#ifndef BLE_PAIRING_H
#define BLE_PAIRING_H

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Preferences.h>
#include "config.h"

// UUID - generate sendiri UUID unik untuk produksi (pakai uuidgenerator.net)
#define SERVICE_UUID           "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define CONFIG_CHAR_UUID       "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  // app -> ESP32 (write)
#define STATUS_CHAR_UUID       "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  // ESP32 -> app (notify)

extern Preferences prefs;
extern String emergencyContacts[MAX_EMERGENCY_CONTACTS];
extern int emergencyContactCount;
extern VehicleProfile activeProfile;

class ConfigCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pChar) override {
    String jsonStr = pChar->getValue();
    if (jsonStr.length() == 0) return;

    // NOTE: untuk parsing JSON yang lebih robust, install library "ArduinoJson"
    // dan ganti parser sederhana ini dengan deserializeJson().
    // Format yang diharapkan dari app (Bab 8.5):
    // {"contacts":["+62811...","+62812..."],"profile":"motor","countdown":30}
    parseAndSaveConfig(jsonStr);
  }

  void parseAndSaveConfig(const String &jsonStr) {
    // --- placeholder parsing, ganti dengan ArduinoJson di implementasi nyata ---
    Serial.println("Konfigurasi diterima via BLE:");
    Serial.println(jsonStr);

    // Contoh penyimpanan ke NVS (Preferences) - sesuaikan setelah parsing JSON asli
    prefs.begin("drivesafe", false);
    prefs.putString("raw_config", jsonStr);
    prefs.end();
  }
};

inline void bleLoadSavedConfig() {
  prefs.begin("drivesafe", true);
  String raw = prefs.getString("raw_config", "");
  int profileIdx = prefs.getInt("profile", PROFILE_MOTOR);
  prefs.end();

  activeProfile = (VehicleProfile)profileIdx;
  if (raw.length() > 0) {
    Serial.println("Konfigurasi tersimpan ditemukan di NVS:");
    Serial.println(raw);
    // TODO: parse 'raw' jadi emergencyContacts[] & emergencyContactCount
  }
}

inline void bleInit(BLECharacteristic **statusCharOut) {
  String macSuffix = String((uint32_t)(ESP.getEfuseMac() & 0xFFFF), HEX);
  String deviceName = "DriveSafe-" + macSuffix;

  BLEDevice::init(deviceName.c_str());
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);

  BLECharacteristic *pConfigChar = pService->createCharacteristic(
      CONFIG_CHAR_UUID,
      BLECharacteristic::PROPERTY_WRITE);
  pConfigChar->setCallbacks(new ConfigCallback());

  BLECharacteristic *pStatusChar = pService->createCharacteristic(
      STATUS_CHAR_UUID,
      BLECharacteristic::PROPERTY_NOTIFY);
  pStatusChar->addDescriptor(new BLE2902());

  pService->start();
  pServer->getAdvertising()->start();

  Serial.println("BLE pairing aktif, nama perangkat: " + deviceName);
  *statusCharOut = pStatusChar;
}

// Kirim status device secara berkala (baterai, GPS fix, speed, dst.)
inline void bleNotifyStatus(BLECharacteristic *pStatusChar, float batteryPct,
                             bool gpsFix, float speedKmh) {
  if (pStatusChar == nullptr) return;
  String status = "{\"batt\":" + String(batteryPct, 0) +
                   ",\"gps\":" + String(gpsFix ? "true" : "false") +
                   ",\"speed\":" + String(speedKmh, 1) + "}";
  pStatusChar->setValue(status.c_str());
  pStatusChar->notify();
}

#endif
