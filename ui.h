/* ============================================================================
 *  DriveSafe - ui.h
 *  Antarmuka lokal perangkat: buzzer 100 dB, LED status, dan tombol interrupt
 *  aktif-rendah untuk pembatalan false alarm (Tabel 6.1, Sub-bab 4.5.4).
 *
 *  Satu tombol melayani dua fungsi sesuai daftar komponen Tabel 9.1:
 *    - tekan singkat  : batalkan hitung mundur alarm (jendela 3 s - 30 s)
 *    - tekan >= 2 s   : aktif/nonaktifkan mode pairing BLE (Bab 8.5)
 * ========================================================================== */
#ifndef DRIVESAFE_UI_H
#define DRIVESAFE_UI_H

#include "config.h"

enum BuzzPattern : uint8_t {
  BUZZ_OFF = 0,
  BUZZ_BOOT,          /* dua bip pendek saat sistem siap        */
  BUZZ_VERIFY,        /* bip tunggal saat masuk fase verifikasi */
  BUZZ_COUNTDOWN,     /* bip berulang makin rapat               */
  BUZZ_ALERT,         /* nada panjang saat notifikasi dikirim   */
  BUZZ_CANCEL,        /* nada konfirmasi pembatalan             */
  BUZZ_ERROR,         /* tiga bip pendek                        */
  BUZZ_PAIRING        /* bip lambat selama mode pairing         */
};

enum ButtonEvent : uint8_t {
  BTN_NONE = 0,
  BTN_SHORT,
  BTN_LONG
};

class DriveSafeUi {
public:
  void begin();

  /* Panggil rutin (>= 20 Hz) dari task komunikasi. */
  void tick(SystemState state, uint32_t countdown_remaining_ms);

  void setBuzzer(BuzzPattern p);
  BuzzPattern buzzer() const { return _pattern; }

  ButtonEvent consumeButtonEvent();

private:
  void updateBuzzer(uint32_t countdown_remaining_ms);
  void updateLed(SystemState state);

  BuzzPattern _pattern     = BUZZ_OFF;
  uint32_t    _patternT0   = 0;
  uint8_t     _step        = 0;
  bool        _buzzOn      = false;

  uint32_t    _ledT0       = 0;
  bool        _ledOn       = false;
};

extern DriveSafeUi Ui;

#endif /* DRIVESAFE_UI_H */
