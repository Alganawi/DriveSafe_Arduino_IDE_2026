/* ============================================================================
 *  DriveSafe - ui.cpp
 * ========================================================================== */
#include "ui.h"

DriveSafeUi Ui;

/* --- Status tombol yang dibagi antara ISR dan task ------------------------ */
static volatile uint32_t s_pressStartMs = 0;
static volatile uint32_t s_lastEdgeMs   = 0;
static volatile uint8_t  s_pendingEvent = BTN_NONE;

static void IRAM_ATTR buttonIsr() {
  const uint32_t now = millis();
  if (now - s_lastEdgeMs < BUTTON_DEBOUNCE_MS) return;   /* debounce kasar   */
  s_lastEdgeMs = now;

  if (digitalRead(PIN_BUTTON) == LOW) {
    s_pressStartMs = now;                                 /* tombol ditekan  */
  } else if (s_pressStartMs != 0) {
    const uint32_t held = now - s_pressStartMs;
    s_pressStartMs = 0;
    if (held >= BUTTON_LONGPRESS_MS)      s_pendingEvent = BTN_LONG;
    else if (held >= BUTTON_DEBOUNCE_MS)  s_pendingEvent = BTN_SHORT;
  }
}

/* -------------------------------------------------------------------------- */
void DriveSafeUi::begin() {
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);

  pinMode(PIN_LED_STATUS, OUTPUT);
  digitalWrite(PIN_LED_STATUS, LOW);

  pinMode(PIN_BUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_BUTTON), buttonIsr, CHANGE);

  Serial.printf("[UI] Buzzer GPIO%d, Tombol GPIO%d (aktif-rendah), LED GPIO%d\n",
                PIN_BUZZER, PIN_BUTTON, PIN_LED_STATUS);
}

ButtonEvent DriveSafeUi::consumeButtonEvent() {
  noInterrupts();
  const uint8_t e = s_pendingEvent;
  s_pendingEvent  = BTN_NONE;
  interrupts();
  return (ButtonEvent)e;
}

void DriveSafeUi::setBuzzer(BuzzPattern p) {
  if (_pattern == p) return;
  _pattern   = p;
  _patternT0 = millis();
  _step      = 0;
  _buzzOn    = false;
  digitalWrite(PIN_BUZZER, LOW);
}

void DriveSafeUi::tick(SystemState state, uint32_t countdown_remaining_ms) {
  updateBuzzer(countdown_remaining_ms);
  updateLed(state);
}

/* -------------------------------------------------------------------------- */
/*  Pola buzzer non-blocking                                                  */
/* -------------------------------------------------------------------------- */
void DriveSafeUi::updateBuzzer(uint32_t countdown_remaining_ms) {
  const uint32_t t = millis() - _patternT0;

  switch (_pattern) {

    case BUZZ_OFF:
      if (_buzzOn) { digitalWrite(PIN_BUZZER, LOW); _buzzOn = false; }
      break;

    case BUZZ_BOOT: {                       /* bip-bip lalu diam            */
      const bool on = (t < 80) || (t >= 180 && t < 260);
      digitalWrite(PIN_BUZZER, on ? HIGH : LOW);
      _buzzOn = on;
      if (t > 400) setBuzzer(BUZZ_OFF);
      break;
    }

    case BUZZ_VERIFY: {                     /* satu bip 150 ms              */
      const bool on = (t < 150);
      digitalWrite(PIN_BUZZER, on ? HIGH : LOW);
      _buzzOn = on;
      if (t > 250) setBuzzer(BUZZ_OFF);
      break;
    }

    case BUZZ_COUNTDOWN: {
      /* Periode bip memendek seiring sisa waktu menuju nol agar pengguna
         makin sadar batas pembatalan hampir habis (Sub-bab 4.5.4). */
      uint32_t period = 1000;
      if      (countdown_remaining_ms < 5000)  period = 200;
      else if (countdown_remaining_ms < 10000) period = 400;
      else if (countdown_remaining_ms < 20000) period = 700;

      const uint32_t phase = (millis() - _patternT0) % period;
      const bool on = phase < (period / 3);
      digitalWrite(PIN_BUZZER, on ? HIGH : LOW);
      _buzzOn = on;
      break;
    }

    case BUZZ_ALERT: {                      /* nada panjang berselang       */
      const uint32_t phase = t % 1200;
      const bool on = phase < 900;
      digitalWrite(PIN_BUZZER, on ? HIGH : LOW);
      _buzzOn = on;
      break;
    }

    case BUZZ_CANCEL: {                     /* satu nada panjang 600 ms     */
      const bool on = (t < 600);
      digitalWrite(PIN_BUZZER, on ? HIGH : LOW);
      _buzzOn = on;
      if (t > 700) setBuzzer(BUZZ_OFF);
      break;
    }

    case BUZZ_ERROR: {                      /* tiga bip pendek              */
      const bool on = (t < 70) || (t >= 160 && t < 230) || (t >= 320 && t < 390);
      digitalWrite(PIN_BUZZER, on ? HIGH : LOW);
      _buzzOn = on;
      if (t > 500) setBuzzer(BUZZ_OFF);
      break;
    }

    case BUZZ_PAIRING: {                    /* bip pendek tiap 2 detik      */
      const uint32_t phase = t % 2000;
      const bool on = phase < 60;
      digitalWrite(PIN_BUZZER, on ? HIGH : LOW);
      _buzzOn = on;
      break;
    }
  }
}

/* -------------------------------------------------------------------------- */
/*  Pola LED status                                                           */
/* -------------------------------------------------------------------------- */
void DriveSafeUi::updateLed(SystemState state) {
  uint32_t period = 2000;
  uint32_t duty   = 60;

  switch (state) {
    case ST_BOOT:
    case ST_CALIBRATING: period = 300;  duty = 150; break;   /* kedip cepat  */
    case ST_MONITORING:  period = 2000; duty = 60;  break;   /* heartbeat    */
    case ST_VERIFYING:   period = 400;  duty = 200; break;
    case ST_COUNTDOWN:   period = 200;  duty = 100; break;
    case ST_ALERTING:    period = 100;  duty = 50;  break;
    case ST_CANCELLED:   period = 1000; duty = 500; break;
    case ST_COOLDOWN:    period = 1500; duty = 100; break;
    case ST_PAIRING:     period = 600;  duty = 300; break;
  }

  const uint32_t phase = millis() % period;
  const bool on = phase < duty;
  if (on != _ledOn) {
    digitalWrite(PIN_LED_STATUS, on ? HIGH : LOW);
    _ledOn = on;
  }
}
