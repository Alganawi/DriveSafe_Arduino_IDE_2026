#ifndef GSM_GPS_H
#define GSM_GPS_H

#include <Arduino.h>
#include <TinyGPS++.h>     // Install via Library Manager: "TinyGPSPlus"
#include "config.h"

// =====================================================================
// GPS (NEO-6M / NEO-M8N) - UART2
// =====================================================================
extern TinyGPSPlus gps;
extern HardwareSerial GPSSerial;   // Serial2

inline void gpsInit() {
  GPSSerial.begin(GPS_BAUD, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
}

// Panggil tiap loop supaya buffer NMEA terus ter-parsing
inline void gpsFeed() {
  while (GPSSerial.available() > 0) {
    gps.encode(GPSSerial.read());
  }
}

inline bool gpsGetLocation(double &lat, double &lng) {
  if (gps.location.isValid() && gps.location.isUpdated()) {
    lat = gps.location.lat();
    lng = gps.location.lng();
    return true;
  }
  // fallback: pakai fix terakhir walau belum updated, asal valid
  if (gps.location.isValid()) {
    lat = gps.location.lat();
    lng = gps.location.lng();
    return true;
  }
  return false;
}

// =====================================================================
// GSM SIM800L - UART1, komunikasi via AT Command (Bab 8.3)
// =====================================================================
extern HardwareSerial SIM800L;   // Serial1

inline void gsmInit() {
  SIM800L.begin(SIM800L_BAUD, SERIAL_8N1, PIN_SIM800L_RX, PIN_SIM800L_TX);
  delay(3000);           // beri waktu modul boot
  SIM800L.println("AT"); // cek modul hidup
  delay(500);
  SIM800L.println("AT+CMGF=1"); // mode SMS teks
  delay(500);
}

// Kirim command AT dan tunggu respons berisi salah satu 'expected' string
inline bool sendATCommand(const String &cmd, const char *expected, unsigned long timeoutMs = 3000) {
  while (SIM800L.available()) SIM800L.read();  // flush buffer lama
  SIM800L.println(cmd);

  unsigned long start = millis();
  String response = "";
  while (millis() - start < timeoutMs) {
    while (SIM800L.available()) {
      response += (char)SIM800L.read();
    }
    if (response.indexOf(expected) != -1) {
      return true;
    }
  }
  return false;
}

// AT+CSQ - cek kualitas sinyal sebelum kirim (Tabel 10.1 poin 3)
// Mengembalikan RSSI 0-31 (99 = tidak diketahui/gagal baca)
inline int gsmGetSignalQuality() {
  while (SIM800L.available()) SIM800L.read();
  SIM800L.println("AT+CSQ");

  unsigned long start = millis();
  String response = "";
  while (millis() - start < 2000) {
    while (SIM800L.available()) response += (char)SIM800L.read();
  }

  int idx = response.indexOf("+CSQ:");
  if (idx == -1) return 99;
  int rssi = response.substring(idx + 5).toInt();
  return rssi;
}

// AT+CMGS - kirim SMS ke satu nomor (Bab 8.3 poin 2)
inline bool gsmSendSMS(const String &phoneNumber, const String &message) {
  while (SIM800L.available()) SIM800L.read();

  SIM800L.print("AT+CMGS=\"");
  SIM800L.print(phoneNumber);
  SIM800L.println("\"");
  delay(200);

  // tunggu prompt ">" dari SIMCOM sebelum kirim isi pesan
  unsigned long start = millis();
  bool promptReceived = false;
  String resp = "";
  while (millis() - start < 3000) {
    while (SIM800L.available()) resp += (char)SIM800L.read();
    if (resp.indexOf(">") != -1) { promptReceived = true; break; }
  }
  if (!promptReceived) return false;

  SIM800L.print(message);
  SIM800L.write(26);   // Ctrl+Z -> kirim SMS

  // tunggu konfirmasi "+CMGS:" (sukses)
  start = millis();
  resp = "";
  while (millis() - start < 10000) {
    while (SIM800L.available()) resp += (char)SIM800L.read();
    if (resp.indexOf("+CMGS:") != -1) return true;
    if (resp.indexOf("ERROR") != -1) return false;
  }
  return false;
}

// ATD - missed call ke kontak prioritas selama beberapa detik (Bab 8.3 poin 3)
inline void gsmMissedCall(const String &phoneNumber) {
  SIM800L.print("ATD");
  SIM800L.print(phoneNumber);
  SIM800L.println(";");
  delay(MISSED_CALL_DURATION_MS);
  SIM800L.println("ATH");   // hang up
}

// Kirim SMS ke seluruh kontak darurat, dengan retry (Bab 8.3 poin 5)
inline int gsmBroadcastSMS(const String contacts[], int contactCount, const String &message) {
  int successCount = 0;
  for (int i = 0; i < contactCount; i++) {
    bool sent = false;
    for (int attempt = 0; attempt <= SMS_RETRY_MAX && !sent; attempt++) {
      sent = gsmSendSMS(contacts[i], message);
      if (!sent) delay(1000);
    }
    if (sent) successCount++;
  }
  return successCount;
}

// Susun isi pesan SMS (Bab 8, informasi kejadian)
inline String buildAlertMessage(const String &vehicleId, const char *impactType,
                                 float Gpeak, double lat, double lng, int severity) {
  const char *severityStr[] = {"Minor", "Moderate", "Severe"};
  String msg = "DRIVESAFE ALERT!\n";
  msg += "Kendaraan: " + vehicleId + "\n";
  msg += "Tipe benturan: " + String(impactType) + "\n";
  msg += "G-Force: " + String(Gpeak, 2) + "g\n";
  msg += "Tingkat: " + String(severityStr[severity]) + "\n";
  msg += "Lokasi: https://maps.google.com/?q=" + String(lat, 6) + "," + String(lng, 6);
  return msg;
}

#endif
