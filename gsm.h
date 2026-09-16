/* ============================================================================
 *  DriveSafe - gsm.h
 *  Driver modul GSM SIM800L berbasis AT Command (Bab 8.3).
 *
 *    AT+CMGS  : pengiriman SMS darurat ke seluruh kontak terdaftar
 *    ATD      : missed call 5 detik ke kontak prioritas pertama
 *    Retry    : maksimal 2 kali bila pengiriman gagal (Bab 8.3 poin 5)
 *    Batasan  : 160 karakter GSM 7-bit agar tetap single-part SMS
 * ========================================================================== */
#ifndef DRIVESAFE_GSM_H
#define DRIVESAFE_GSM_H

#include "config.h"

enum GsmState : uint8_t {
  GSM_OFFLINE = 0,
  GSM_READY,
  GSM_NO_SIM,
  GSM_NO_NETWORK
};

class DriveSafeGsm {
public:
  bool     begin();
  bool     isReady()   const { return _state == GSM_READY; }
  GsmState state()     const { return _state; }
  int      rssi()      const { return _rssi; }      /* 0..31, 99 = unknown  */
  int      signalPercent();

  bool     refreshStatus();                          /* CSQ + CREG          */
  bool     sendSMS(const char *number, const String &text);
  bool     missedCall(const char *number,
                      uint16_t ring_ms = MISSED_CALL_DURATION_MS);

  /* Bersihkan karakter non-GSM dan potong ke 160 karakter. */
  static String sanitize(const String &in, uint8_t maxLen = SMS_MAX_CHARS);

private:
  bool     sendAT(const char *cmd, const char *expect = "OK",
                  uint32_t timeout = GSM_AT_TIMEOUT_MS);
  bool     waitFor(const char *token, uint32_t timeout, String *capture = nullptr);
  void     flushInput();
  void     hardwareReset();

  GsmState _state = GSM_OFFLINE;
  int      _rssi  = 99;
};

extern DriveSafeGsm Gsm;

#endif /* DRIVESAFE_GSM_H */
