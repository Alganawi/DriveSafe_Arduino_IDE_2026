/* ============================================================================
 *  DriveSafe - gsm.cpp
 * ========================================================================== */
#include "gsm.h"

DriveSafeGsm Gsm;

static HardwareSerial GsmSerial(2);          /* UART2 */

/* -------------------------------------------------------------------------- */
void DriveSafeGsm::flushInput() {
  while (GsmSerial.available()) GsmSerial.read();
}

void DriveSafeGsm::hardwareReset() {
#if (PIN_GSM_RST >= 0)
  pinMode(PIN_GSM_RST, OUTPUT);
  digitalWrite(PIN_GSM_RST, LOW);
  delay(120);
  digitalWrite(PIN_GSM_RST, HIGH);
  delay(2500);
#endif
}

bool DriveSafeGsm::waitFor(const char *token, uint32_t timeout, String *capture) {
  const uint32_t t0 = millis();
  String buf;
  buf.reserve(128);

  while (millis() - t0 < timeout) {
    while (GsmSerial.available()) {
      char c = (char)GsmSerial.read();
      buf += c;
      if (buf.length() > 400) buf.remove(0, 200);   /* jaga memori           */
      if (buf.indexOf(token) >= 0) {
        if (capture) *capture = buf;
        return true;
      }
      if (buf.indexOf("ERROR") >= 0) {
        if (capture) *capture = buf;
        return false;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
  if (capture) *capture = buf;
  return false;
}

bool DriveSafeGsm::sendAT(const char *cmd, const char *expect, uint32_t timeout) {
  flushInput();
  GsmSerial.print(cmd);
  GsmSerial.print("\r\n");
  return waitFor(expect, timeout);
}

/* -------------------------------------------------------------------------- */
bool DriveSafeGsm::begin() {
  GsmSerial.setRxBufferSize(512);           /* wajib sebelum begin()        */
  GsmSerial.begin(GSM_BAUD, SERIAL_8N1, PIN_GSM_RX, PIN_GSM_TX);
  Serial.printf("[GSM] UART2 aktif pada RX=%d TX=%d @%lu bps\n",
                PIN_GSM_RX, PIN_GSM_TX, (unsigned long)GSM_BAUD);

  hardwareReset();

  /* SIM800L butuh beberapa detik hingga merespons AT */
  bool alive = false;
  const uint32_t t0 = millis();
  while (millis() - t0 < GSM_BOOT_TIMEOUT_MS) {
    if (sendAT("AT", "OK", 1200)) { alive = true; break; }
    delay(400);
  }

  if (!alive) {
    Serial.println(F("[GSM] Modul tidak merespons. Periksa catu daya 4V/2A "
                     "dan kapasitor bulk (Bab 7.3)."));
    _state = GSM_OFFLINE;
    return false;
  }

  sendAT("ATE0");                     /* matikan echo agar parsing bersih   */
  sendAT("AT+CMEE=1");                /* kode error numerik                 */
  sendAT("AT+CMGF=1");                /* SMS text mode                      */
  sendAT("AT+CSCS=\"GSM\"");          /* charset GSM 7-bit                  */
  sendAT("AT+CNMI=2,0,0,0,0");        /* jangan dorong SMS masuk ke UART    */

  /* Cek kartu SIM */
  if (!sendAT("AT+CPIN?", "READY", 5000)) {
    Serial.println(F("[GSM] Kartu SIM tidak terbaca"));
    _state = GSM_NO_SIM;
    return false;
  }

  refreshStatus();
  Serial.printf("[GSM] Status=%d  RSSI=%d (%d%%)\n", _state, _rssi, signalPercent());
  return _state == GSM_READY;
}

/* -------------------------------------------------------------------------- */
bool DriveSafeGsm::refreshStatus() {
  String resp;

  flushInput();
  GsmSerial.print("AT+CSQ\r\n");
  if (waitFor("+CSQ:", 2000, &resp)) {
    int p = resp.indexOf("+CSQ:");
    if (p >= 0) _rssi = resp.substring(p + 5, resp.indexOf(',', p)).toInt();
  }

  flushInput();
  GsmSerial.print("AT+CREG?\r\n");
  bool registered = false;
  if (waitFor("+CREG:", 3000, &resp)) {
    int p = resp.indexOf("+CREG:");
    if (p >= 0) {
      int comma = resp.indexOf(',', p);
      int stat  = resp.substring(comma + 1, comma + 2).toInt();
      registered = (stat == 1 || stat == 5);   /* home atau roaming         */
    }
  }

  _state = registered ? GSM_READY : GSM_NO_NETWORK;
  return registered;
}

int DriveSafeGsm::signalPercent() {
  if (_rssi >= 99 || _rssi < 0) return 0;
  int pct = (_rssi * 100) / 31;
  return pct > 100 ? 100 : pct;
}

/* -------------------------------------------------------------------------- */
/*  Sanitasi teks ke alfabet GSM 7-bit dan pemotongan 160 karakter            */
/* -------------------------------------------------------------------------- */
String DriveSafeGsm::sanitize(const String &in, uint8_t maxLen) {
  String out;
  out.reserve(in.length() + 1);

  for (size_t i = 0; i < in.length(); i++) {
    char c = in[i];
    if (c == '\r') continue;
    if (c == '\n') { out += '\n'; continue; }
    if ((uint8_t)c < 32 || (uint8_t)c > 126) { out += ' '; continue; }
    /* Karakter di bawah ini memerlukan escape pada GSM 7-bit; diganti
       agar panjang pesan tetap dapat diprediksi. */
    if (c == '[' || c == ']' || c == '{' || c == '}' ||
        c == '\\'|| c == '^' || c == '~' || c == '|') { out += '-'; continue; }
    out += c;
  }

  if (out.length() > maxLen) out = out.substring(0, maxLen);
  return out;
}

/* -------------------------------------------------------------------------- */
/*  Pengiriman SMS (AT+CMGS) dengan retry maksimal 2 kali                     */
/* -------------------------------------------------------------------------- */
bool DriveSafeGsm::sendSMS(const char *number, const String &text) {
  if (!number || strlen(number) < 8) return false;

  const String body = sanitize(text);

  for (uint8_t attempt = 0; attempt <= SMS_MAX_RETRY; attempt++) {
    if (attempt > 0) {
      Serial.printf("[GSM] Retry SMS ke-%u untuk %s\n", attempt, number);
      sendAT("AT+CMGF=1");
      delay(800);
    }

    flushInput();
    GsmSerial.print("AT+CMGS=\"");
    GsmSerial.print(number);
    GsmSerial.print("\"\r");

    /* Tunggu prompt '>' dari SMSC (Bab 8.3 poin 3) */
    if (!waitFor(">", 6000)) {
      Serial.println(F("[GSM] Prompt '>' tidak muncul"));
      GsmSerial.write(27);                 /* ESC, batalkan perintah        */
      delay(300);
      continue;
    }

    GsmSerial.print(body);
    GsmSerial.write(26);                   /* Ctrl-Z, kirim                 */

    String resp;
    if (waitFor("+CMGS:", GSM_SMS_TIMEOUT_MS, &resp) ||
        resp.indexOf("OK") >= 0) {
      Serial.printf("[GSM] SMS terkirim ke %s (%u karakter)\n",
                    number, (unsigned)body.length());
      return true;
    }

    Serial.printf("[GSM] SMS gagal ke %s\n", number);
  }
  return false;
}

/* -------------------------------------------------------------------------- */
/*  Missed call 5 detik (ATD ... ; lalu ATH)                                  */
/* -------------------------------------------------------------------------- */
bool DriveSafeGsm::missedCall(const char *number, uint16_t ring_ms) {
  if (!number || strlen(number) < 8) return false;

  char cmd[32];
  snprintf(cmd, sizeof(cmd), "ATD%s;", number);

  flushInput();
  GsmSerial.print(cmd);
  GsmSerial.print("\r\n");

  if (!waitFor("OK", 8000)) {
    Serial.printf("[GSM] Panggilan ke %s gagal dimulai\n", number);
    sendAT("ATH");
    return false;
  }

  Serial.printf("[GSM] Missed call ke %s selama %u ms\n", number, ring_ms);
  vTaskDelay(pdMS_TO_TICKS(ring_ms));
  sendAT("ATH");
  return true;
}
