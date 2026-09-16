/* ============================================================================
 *  ####  ####  #  #  #  #  ####  ####  ####  #####
 *  #  #  #  #  #  #  #  #  #     #  #  #     #
 *  #  #  ####  #  #  ####  ####  ####  ####  ###
 *  #  #  # #   #  #  #        #  #  #  #     #
 *  ####  #  #  ####  #  #  ####  #  #  #     #####
 *
 *  SISTEM DETEKSI KECELAKAAN KENDARAAN DAN RESPONS DARURAT CERDAS
 *  BERBASIS IOT DENGAN PERANGKAT MULTISENSOR
 *
 *  Dokumen C-251 | Capstone Project Kelompok B-06
 *  Departemen Teknik Elektro dan Teknologi Informasi, FT UGM
 *
 *  Target papan : ESP32 DevKit V1
 *  Sensor       : BNO055 (I2C), NEO-M8N (UART1), SIM800L (UART2)
 *
 *  Berkas ini berisi mesin keadaan utama (Gambar 4.2 dan Gambar 4.3),
 *  penjadwalan dua task FreeRTOS, penyusunan pesan SMS darurat, dan
 *  antarmuka baris perintah untuk pengujian (Tabel 10.2).
 * ========================================================================== */

#include <Arduino.h>
#include <math.h>
#include "config.h"
#include "sensor.h"
#include "detector.h"
#include "eventbuf.h"
#include "storage.h"
#include "gps.h"
#include "gsm.h"
#include "bleif.h"
#include "ui.h"

/* ==========================================================================
 *  Nama-nama untuk pelaporan
 * ========================================================================== */
const char *stateName(SystemState s) {
  switch (s) {
    case ST_BOOT:        return "BOOT";
    case ST_CALIBRATING: return "CALIB";
    case ST_MONITORING:  return "MONITOR";
    case ST_VERIFYING:   return "VERIFY";
    case ST_COUNTDOWN:   return "COUNTDOWN";
    case ST_ALERTING:    return "ALERT";
    case ST_CANCELLED:   return "CANCELLED";
    case ST_COOLDOWN:    return "COOLDOWN";
    case ST_PAIRING:     return "PAIRING";
  }
  return "?";
}

const char *severityName(uint8_t s) {
  switch (s) {
    case SEVERITY_MINOR:    return "Ringan";
    case SEVERITY_MODERATE: return "Sedang";
    case SEVERITY_SEVERE:   return "Parah";
  }
  return "-";
}

const char *crashTypeName(uint8_t t) {
  switch (t) {
    case CRASH_FRONTAL:  return "Tabrakan depan";
    case CRASH_REAR:      return "Ditabrak belakang";
    case CRASH_SIDE:      return "Benturan samping";
    case CRASH_FALL:      return "Terjatuh";
    case CRASH_ROLLOVER:  return "Terguling";
  }
  return "Tidak diketahui";
}

/* ==========================================================================
 *  Keadaan bersama antar task
 * ========================================================================== */
static SemaphoreHandle_t g_mux = nullptr;

static volatile SystemState g_state        = ST_BOOT;
static volatile uint32_t    g_stateEnterMs = 0;
static volatile uint32_t    g_triggerMs    = 0;
static volatile uint32_t    g_countdownMs  = 0;   /* awal hitung mundur      */
static volatile bool        g_cancelReq    = false;
static volatile bool        g_simulateReq  = false;
static volatile bool        g_recalibReq   = false;
static volatile bool        g_pairingOn    = false;

static EventLog     g_event;          /* akumulasi puncak kejadian berjalan  */
static SensorFeature g_lastFeature;   /* untuk pelaporan status              */
static float        g_axPeak = 0, g_ayPeak = 0;
static bool         g_sensorOk = false;
static uint8_t      g_batteryPct = 0;

/* Pengukuran baseline manuver normal (Tabel 10.1 tahap 2) */
static volatile bool     g_baselineActive = false;
static volatile uint32_t g_baselineEndMs  = 0;
static volatile float    g_bMaxG = 0, g_bMaxJ = 0, g_bMaxR = 0, g_bMaxTilt = 0;

static uint32_t     g_sampleCount = 0;
static uint32_t     g_missedSamples = 0;
static float        g_actualHz = 0;

#define LOCK()   xSemaphoreTake(g_mux, portMAX_DELAY)
#define UNLOCK() xSemaphoreGive(g_mux)

/* ==========================================================================
 *  Utilitas
 * ========================================================================== */
static void setState(SystemState s) {
  if (g_state == s) return;
  Serial.printf("[FSM] %s -> %s  (t=%lu ms)\n",
                stateName((SystemState)g_state), stateName(s),
                (unsigned long)millis());
  g_state        = s;
  g_stateEnterMs = millis();
}

static uint32_t countdownTotalMs() {
  uint32_t s = Store.config().countdown_s;
  if (s < 5 || s > 120) s = SPARE_TOTAL_MS / 1000;
  return s * 1000UL;
}

static uint32_t countdownRemainingMs() {
  if (g_state != ST_COUNTDOWN) return 0;
  const uint32_t total   = countdownTotalMs();
  const uint32_t elapsed = millis() - g_countdownMs;
  return elapsed >= total ? 0 : (total - elapsed);
}

static uint8_t readBatteryPercent() {
#if defined(ARDUINO_ARCH_ESP32)
  uint32_t mv = 0;
  for (uint8_t i = 0; i < 8; i++) mv += analogReadMilliVolts(PIN_VBAT_ADC);
  const float vbat = (mv / 8.0f / 1000.0f) * VBAT_DIVIDER_RATIO;
  float pct = (vbat - VBAT_EMPTY) / (VBAT_FULL - VBAT_EMPTY) * 100.0f;
  if (pct < 0)   pct = 0;
  if (pct > 100) pct = 100;
  return (uint8_t)pct;
#else
  return 100;
#endif
}

/* Waktu lokal WIB (UTC+7) dari epoch GPS. */
static void formatWib(uint32_t epoch, char *out, size_t len) {
  if (epoch == 0) { snprintf(out, len, "--/-- --:--"); return; }
  const uint32_t t = epoch + 7UL * 3600UL;
  const uint32_t days = t / 86400UL;
  const uint32_t secs = t % 86400UL;

  /* Konversi hari sejak epoch ke tanggal (algoritma civil_from_days) */
  int32_t z = (int32_t)days + 719468;
  const int32_t era = (z >= 0 ? z : z - 146096) / 146097;
  const uint32_t doe = (uint32_t)(z - era * 146097);
  const uint32_t yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  const int32_t y = (int32_t)yoe + era * 400;
  const uint32_t doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
  const uint32_t mp = (5 * doy + 2) / 153;
  const uint32_t d = doy - (153 * mp + 2) / 5 + 1;
  const uint32_t m = mp + (mp < 10 ? 3 : -9);
  (void)y;

  snprintf(out, len, "%02u/%02u %02u:%02u",
           (unsigned)d, (unsigned)m,
           (unsigned)(secs / 3600), (unsigned)((secs % 3600) / 60));
}

/* ==========================================================================
 *  Penyusunan pesan SMS darurat (maksimal 160 karakter GSM 7-bit)
 * ========================================================================== */
static String buildEmergencySms(const EventLog &ev) {
  char waktu[16];
  formatWib(ev.timestamp, waktu, sizeof(waktu));

  const String lokasi = Gps.mapsLink();

  char buf[SMS_MAX_CHARS + 32];
  snprintf(buf, sizeof(buf),
           "DRIVESAFE - KECELAKAAN\n"
           "%s (%s)\n"
           "%s %.1fg %s\n"
           "%s WIB\n"
           "%s",
           Store.config().owner,
           PROFILE_TABLE[ev.profile < PROFILE_COUNT ? ev.profile : PROFILE_MOTOR].name,
           crashTypeName(ev.crash_type), ev.g_peak, severityName(ev.severity),
           waktu, lokasi.c_str());

  return DriveSafeGsm::sanitize(String(buf), SMS_MAX_CHARS);
}

/* ==========================================================================
 *  Prosedur darurat (Bab 8.3)
 *   1. ambil koordinat GPS terkini
 *   2. AT+CMGS ke seluruh kontak terdaftar
 *   3. ATD missed call 5 detik ke kontak prioritas pertama
 *   4. simpan event log ke memori flash internal
 * ========================================================================== */
static void runEmergencyProcedure() {
  const uint32_t t0 = millis();

  EventLog ev;
  LOCK();
  ev = g_event;
  UNLOCK();

  /* Langkah 1: koordinat terkini */
  Gps.poll();
  const GpsFix fix = Gps.hasFix() ? Gps.current() : Gps.lastKnown();
  ev.lat       = (float)fix.lat;
  ev.lon       = (float)fix.lon;
  ev.timestamp = Gps.epochNow();

  const String pesan = buildEmergencySms(ev);
  Serial.println(F("\n================ NOTIFIKASI DARURAT ================"));
  Serial.println(pesan);
  Serial.printf("(%u karakter)\n", (unsigned)pesan.length());
  Serial.println(F("===================================================="));

  Ble.notifyEvent(ev, Gps.mapsLink().c_str(), "SENDING");

  /* Langkah 2: SMS ke seluruh kontak */
  uint8_t sukses = 0;
  const uint8_t total = Store.contactCount();

  if (total == 0) {
    Serial.println(F("[ALERT] Tidak ada kontak darurat terdaftar!"));
  } else if (!Gsm.isReady()) {
    Gsm.refreshStatus();
  }

  for (uint8_t i = 0; i < total; i++) {
    const EmergencyContact *c = Store.contact(i);
    if (!c || !c->active) continue;
    if (Gsm.sendSMS(c->number, pesan)) sukses++;
  }

  /* Langkah 3: missed call ke kontak prioritas pertama */
  const int8_t prim = Store.primaryContactIndex();
  if (prim >= 0) {
    const EmergencyContact *c = Store.contact((uint8_t)prim);
    if (c) {
      Serial.printf("[ALERT] Eskalasi missed call ke %s (%s)\n",
                    c->name, c->number);
      Gsm.missedCall(c->number);
    }
  }

  /* Langkah 4: simpan event log */
  LOCK();
  g_event = ev;
  UNLOCK();
  Store.persistEvent(ev, EvBuf);

  Ble.notifyEvent(ev, Gps.mapsLink().c_str(), "SENT");

  Serial.printf("[ALERT] Selesai: %u/%u SMS berhasil, durasi %lu ms "
                "(target <= 30000 ms, Tabel 6.2 no.4)\n",
                sukses, total, (unsigned long)(millis() - t0));
}

/* ==========================================================================
 *  TASK 1 - Akuisisi & deteksi, 100 Hz, dipatri ke core 1
 * ========================================================================== */
static void sensorTask(void *pv) {
  (void)pv;
  TickType_t lastWake = xTaskGetTickCount();
  uint32_t   hzT0 = millis(), hzCount = 0;

  SensorSample  s;
  SensorFeature f;

  for (;;) {
    if (!Sensors.read(s)) {
      g_sensorOk = false;
      g_missedSamples++;
      vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
      continue;
    }
    g_sensorOk = true;
    g_sampleCount++;
    hzCount++;

    Detector.computeFeatures(s, Sensors.roll(), Sensors.pitch(), Sensors.yaw(), f);
    EvBuf.push(s);

    const uint32_t now = millis();
    const SystemState st = (SystemState)g_state;

    /* ---- Perekaman baseline manuver normal ----------------------------- */
    if (g_baselineActive) {
      if (f.g_force  > g_bMaxG) g_bMaxG = f.g_force;
      if (f.jerk     > g_bMaxJ) g_bMaxJ = f.jerk;
      if (f.rot_jerk > g_bMaxR) g_bMaxR = f.rot_jerk;
      const float tilt = fmaxf(fabsf(f.roll), fabsf(f.pitch));
      if (tilt > g_bMaxTilt) g_bMaxTilt = tilt;
    }

    /* ---- Pembaruan nilai puncak selama kejadian berlangsung ------------ */
    if (st == ST_VERIFYING || st == ST_COUNTDOWN) {
      LOCK();
      if (f.g_force  > g_event.g_peak)     g_event.g_peak     = f.g_force;
      if (f.jerk     > g_event.j_peak)     g_event.j_peak     = f.jerk;
      if (f.rot_jerk > g_event.r_peak)     g_event.r_peak     = f.rot_jerk;
      if (fabsf(f.roll)  > fabsf(g_event.roll_peak))  g_event.roll_peak  = f.roll;
      if (fabsf(f.pitch) > fabsf(g_event.pitch_peak)) g_event.pitch_peak = f.pitch;
      if (fabsf(s.ax) > fabsf(g_axPeak)) g_axPeak = s.ax;
      if (fabsf(s.ay) > fabsf(g_ayPeak)) g_ayPeak = s.ay;
      UNLOCK();
    }

    LOCK();
    g_lastFeature = f;
    UNLOCK();

    /* ---- Mesin keadaan ------------------------------------------------- */
    switch (st) {

      case ST_MONITORING: {
        /* Jalur paralel: rollover persisten dapat memicu langsung
           (Sub-bab 4.5.3, Pers. 4.23) */
        const bool rollover = Detector.checkRolloverPersistent(f, now);

        uint8_t flags = 0;
        const bool f1 = Detector.stageEventDetection(s, f, flags);

        if (g_simulateReq) {                /* pemicu buatan untuk pengujian */
          g_simulateReq = false;
          flags |= TRIG_G;
        } else if (!f1 && !rollover) {
          break;
        }

        /* Inisialisasi catatan kejadian */
        LOCK();
        memset(&g_event, 0, sizeof(g_event));
        g_event.g_peak        = f.g_force;
        g_event.j_peak        = f.jerk;
        g_event.r_peak        = f.rot_jerk;
        g_event.roll_peak     = f.roll;
        g_event.pitch_peak    = f.pitch;
        g_event.profile       = (uint8_t)Detector.profile();
        g_event.trigger_flags = flags | (rollover ? TRIG_ROLLOVER : 0);
        g_axPeak = s.ax;
        g_ayPeak = s.ay;
        UNLOCK();

        g_triggerMs = now;
        EvBuf.armPostCapture();
        Detector.resetVerification();
        g_cancelReq = false;

        Serial.printf("\n[DETEKSI] f1=1 flags=0x%02X | G=%.2fg (Gth=%.2fg, "
                      "ks=%.2f kv=%.2f) J=%.1f R=%.1f roll=%.1f pitch=%.1f\n",
                      g_event.trigger_flags, f.g_force, f.g_threshold,
                      f.k_slope, f.k_speed, f.jerk, f.rot_jerk, f.roll, f.pitch);

        if (rollover) {
          /* Orientasi ekstrem persisten langsung dianggap terkonfirmasi */
          Serial.println(F("[DETEKSI] Rollover persisten -> konfirmasi langsung"));
          g_countdownMs = now;
          setState(ST_COUNTDOWN);
        } else {
          setState(ST_VERIFYING);
        }
        break;
      }

      case ST_VERIFYING: {
        const bool f2 = Detector.stageStillness(f, now);
        const bool f3 = Detector.stageAbnormalOrientation(f, now);
        const bool rollover = Detector.checkRolloverPersistent(f, now);

        /* Pers. 4.17 : f[n] = f1 AND (f2 OR f3) */
        if (f2 || f3 || rollover) {
          LOCK();
          if (rollover) g_event.trigger_flags |= TRIG_ROLLOVER;
          UNLOCK();
          Serial.printf("[KONFIRMASI] f2=%d f3=%d rollover=%d -> KECELAKAAN VALID\n",
                        f2, f3, rollover);
          g_countdownMs = now;
          setState(ST_COUNTDOWN);
          break;
        }

        if (now - g_triggerMs >= VERIFY_WINDOW_MS) {
          Serial.println(F("[KONFIRMASI] Tidak terpenuhi dalam 5 s "
                           "-> false trigger, kembali monitoring"));
          EvBuf.release();
          Detector.resetVerification();
          setState(ST_MONITORING);
        }
        break;
      }

      case ST_COUNTDOWN: {
        const uint32_t elapsed = now - g_countdownMs;

        /* Jendela pembatalan sah: 3 s s.d. 30 s (Sub-bab 4.5.4) */
        if (g_cancelReq) {
          g_cancelReq = false;
          /* Batas atas jendela mengikuti durasi countdown aktif bila pengguna
             mengonfigurasi nilai lebih panjang dari 30 s bawaan dokumen. */
          const uint32_t cancelMax = countdownTotalMs() > CANCEL_WINDOW_MAX_MS
                                     ? countdownTotalMs() : CANCEL_WINDOW_MAX_MS;
          if (elapsed >= CANCEL_WINDOW_MIN_MS && elapsed <= cancelMax) {
            Serial.printf("[BATAL] False alarm dibatalkan pada t=%lu ms\n",
                          (unsigned long)elapsed);
            EvBuf.release();
            Detector.resetVerification();
            setState(ST_CANCELLED);
            break;
          }
          Serial.printf("[BATAL] Ditolak, di luar jendela %lu-%lu ms (t=%lu ms)\n",
                        (unsigned long)CANCEL_WINDOW_MIN_MS,
                        (unsigned long)cancelMax, (unsigned long)elapsed);
        }

        if (elapsed >= countdownTotalMs()) {
          LOCK();
          g_event.severity   = CrashDetector::classifySeverity(g_event.g_peak);
          g_event.crash_type = CrashDetector::classifyCrashType(
              g_axPeak, g_ayPeak, g_event.roll_peak, g_event.pitch_peak,
              (g_event.trigger_flags & TRIG_ROLLOVER) != 0);
          UNLOCK();
          setState(ST_ALERTING);
        }
        break;
      }

      case ST_CANCELLED:
        if (now - g_stateEnterMs > 2500) setState(ST_MONITORING);
        break;

      case ST_COOLDOWN:
        if (now - g_stateEnterMs > COOLDOWN_MS) {
          EvBuf.release();
          Detector.resetVerification();
          setState(ST_MONITORING);
        }
        break;

      default:
        break;      /* ST_ALERTING, ST_PAIRING ditangani task komunikasi */
    }

    /* Statistik laju cuplikan efektif */
    if (now - hzT0 >= 5000) {
      g_actualHz = hzCount * 1000.0f / (now - hzT0);
      hzT0 = now;
      hzCount = 0;
    }

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
  }
}

/* ==========================================================================
 *  TASK 2 - Komunikasi, UI, dan prosedur darurat, dipatri ke core 0
 * ========================================================================== */
static void handleButton();
static void handleBleCommand();
static void handleSerialCli();

static void commsTask(void *pv) {
  (void)pv;
  uint32_t lastStatus = 0, lastGsm = 0, lastBat = 0;

  for (;;) {
    Gps.poll();
    handleButton();
    handleBleCommand();
    handleSerialCli();

    Ui.tick((SystemState)g_state, countdownRemainingMs());

    /* Pola buzzer mengikuti keadaan */
    switch ((SystemState)g_state) {
      case ST_VERIFYING:
        if (Ui.buzzer() != BUZZ_VERIFY && Ui.buzzer() != BUZZ_OFF)
          Ui.setBuzzer(BUZZ_VERIFY);
        break;
      case ST_COUNTDOWN:  Ui.setBuzzer(BUZZ_COUNTDOWN); break;
      case ST_ALERTING:   Ui.setBuzzer(BUZZ_ALERT);     break;
      case ST_CANCELLED:  Ui.setBuzzer(BUZZ_CANCEL);    break;
      case ST_PAIRING:    Ui.setBuzzer(BUZZ_PAIRING);   break;
      case ST_MONITORING:
      case ST_COOLDOWN:
        if (Ui.buzzer() == BUZZ_COUNTDOWN || Ui.buzzer() == BUZZ_ALERT ||
            Ui.buzzer() == BUZZ_PAIRING)
          Ui.setBuzzer(BUZZ_OFF);
        break;
      default: break;
    }

    /* Eksekusi prosedur darurat (memblokir beberapa detik) */
    if (g_state == ST_ALERTING) {
      runEmergencyProcedure();
      Ui.setBuzzer(BUZZ_OFF);
      setState(ST_COOLDOWN);
    }

    /* Kalibrasi ulang atas permintaan */
    if (g_recalibReq) {
      g_recalibReq = false;
      Sensors.calibrateBias(2000);
      Ui.setBuzzer(BUZZ_BOOT);
    }

    const uint32_t now = millis();

    /* Akhiri sesi baseline dan laporkan usulan threshold empiris */
    if (g_baselineActive && (int32_t)(now - g_baselineEndMs) >= 0) {
      g_baselineActive = false;
      Ui.setBuzzer(BUZZ_VERIFY);

      const ProfileThreshold &th = Detector.thresholds();
      Serial.println(F("\n===== HASIL BASELINE MANUVER NORMAL ====="));
      Serial.println(F("(Tabel 10.1 tahap 2 - penetapan ambang batas empiris)"));
      Serial.printf("Profil aktif      : %s\n", th.name);
      Serial.printf("G maksimum        : %.2f g        (Gth dokumen %.2f g)\n",
                    g_bMaxG, th.g_th);
      Serial.printf("J maksimum        : %.1f m/s3    (Jth aktif %.1f m/s3)\n",
                    g_bMaxJ, Detector.jerkThreshold());
      Serial.printf("R maksimum        : %.1f deg/s2  (Rth aktif %.1f deg/s2)\n",
                    g_bMaxR, Detector.rotJerkThreshold());
      Serial.printf("Kemiringan maks   : %.1f deg\n", g_bMaxTilt);
      Serial.println(F("--- Usulan (margin 1,3x di atas nilai normal maksimum) ---"));
      Serial.printf("  G_th            : %.2f g   %s\n", g_bMaxG * 1.3f,
                    (g_bMaxG * 1.3f < th.g_th)
                      ? "-> nilai dokumen sudah aman"
                      : "-> NAIKKAN, nilai dokumen terlalu rendah");
      Serial.printf("  JERK_THRESHOLD_SCALE    : %.1f  (J_th -> %.0f m/s3)\n",
                    (g_bMaxJ * 1.3f) / th.j_th, g_bMaxJ * 1.3f);
      Serial.printf("  ROTJERK_THRESHOLD_SCALE : %.1f  (R_th -> %.0f deg/s2)\n",
                    (g_bMaxR * 1.3f) / th.r_th, g_bMaxR * 1.3f);
      Serial.println(F("Perbarui kedua pengali tersebut pada config.h lalu "
                       "unggah ulang firmware."));
      Serial.println(F("=========================================\n"));
    }

    if (now - lastBat > 10000) {
      lastBat = now;
      g_batteryPct = readBatteryPercent();
    }

    if (now - lastGsm > 30000) {
      lastGsm = now;
      if (g_state == ST_MONITORING) Gsm.refreshStatus();
    }

    if (now - lastStatus >= BLE_STATUS_PERIOD_MS) {
      lastStatus = now;
      Ble.notifyStatus((SystemState)g_state, Gps.speedKmh(), g_batteryPct,
                       g_sensorOk, Gps.hasFix(), Gps.satellites(),
                       Gsm.signalPercent(), Store.contactCount(),
                       countdownRemainingMs());
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

/* ==========================================================================
 *  Penanganan tombol fisik
 * ========================================================================== */
static void handleButton() {
  const ButtonEvent e = Ui.consumeButtonEvent();
  if (e == BTN_NONE) return;

  if (e == BTN_LONG) {
    g_pairingOn = !g_pairingOn;
    if (g_pairingOn) {
      Ble.startPairing();
      if (g_state == ST_MONITORING) setState(ST_PAIRING);
    } else {
      Ble.stopPairing();
      if (g_state == ST_PAIRING) setState(ST_MONITORING);
    }
    return;
  }

  /* Tekan singkat = pembatalan false alarm */
  if (g_state == ST_COUNTDOWN) {
    g_cancelReq = true;
    Serial.println(F("[TOMBOL] Permintaan pembatalan"));
  } else {
    Serial.printf("[TOMBOL] Ditekan saat %s (tidak ada alarm aktif)\n",
                  stateName((SystemState)g_state));
  }
}

/* ==========================================================================
 *  Perintah dari aplikasi mobile via BLE
 * ========================================================================== */
static void handleBleCommand() {
  switch (Ble.consumeCommand()) {
    case BLECMD_CANCEL:
      g_cancelReq = true;
      Serial.println(F("[BLE] Permintaan pembatalan dari aplikasi"));
      break;

    case BLECMD_TEST_SMS: {
      const int8_t i = Store.primaryContactIndex();
      const EmergencyContact *c = i >= 0 ? Store.contact((uint8_t)i) : nullptr;
      if (c) {
        Serial.printf("[UJI] Kirim SMS uji ke %s\n", c->number);
        Gsm.sendSMS(c->number,
                    String("DRIVESAFE: uji koneksi berhasil. Perangkat "
                           "siap memantau perjalanan Anda."));
      } else {
        Serial.println(F("[UJI] Belum ada kontak prioritas"));
      }
      break;
    }

    case BLECMD_RECALIBRATE:   g_recalibReq  = true; break;
    case BLECMD_SIMULATE_CRASH:g_simulateReq = true; break;

    case BLECMD_CLEAR_CONTACTS:
      Store.clearContacts();
      Store.save();
      Serial.println(F("[BLE] Seluruh kontak dihapus"));
      break;

    default: break;
  }
}

/* ==========================================================================
 *  Antarmuka baris perintah serial untuk pengujian (Tabel 10.2)
 * ========================================================================== */
static void printHelp() {
  Serial.println(F(
    "\n--- Perintah DriveSafe ---\n"
    "  status                    tampilkan status sistem\n"
    "  live                      tampilkan 20 cuplikan fitur real-time\n"
    "  contacts                  daftar kontak darurat\n"
    "  add <nama> <nomor> <pri>  tambah kontak (nomor format +62...)\n"
    "  del <index>               hapus kontak\n"
    "  profile <sepeda|motor|mobil>\n"
    "  owner <nama>              atur nama pemilik\n"
    "  countdown <detik>         atur durasi hitung mundur (5-120)\n"
    "  baseline <detik>          rekam manuver normal & usulkan threshold\n"
    "  calib                     ulangi kalibrasi bias giroskop\n"
    "  sim                       simulasikan pemicu kecelakaan\n"
    "  cancel                    batalkan hitung mundur\n"
    "  sms                       kirim SMS uji ke kontak prioritas\n"
    "  gsm                       segarkan status modul GSM\n"
    "  gps                       tampilkan data GPS terkini\n"
    "  pair / unpair             hidup/matikan advertising BLE\n"
    "  log                       tampilkan event log terakhir\n"
    "  mem                       pemakaian memori\n"
    "  help                      tampilkan bantuan ini\n"));
}

static void printStatus() {
  SensorFeature f;
  LOCK(); f = g_lastFeature; UNLOCK();

  const ProfileThreshold &th = Detector.thresholds();

  Serial.println(F("\n===== STATUS DRIVESAFE ====="));
  Serial.printf("Firmware      : %s v%s (%s)\n", FW_NAME, FW_VERSION, DOC_CODE);
  Serial.printf("Keadaan       : %s\n", stateName((SystemState)g_state));
  Serial.printf("Pemilik       : %s\n", Store.config().owner);
  Serial.printf("Profil        : %s  (Gth=%.2fg  Jth=%.0f m/s3  Rth=%.0f deg/s2)\n",
                th.name, th.g_th, Detector.jerkThreshold(), Detector.rotJerkThreshold());
  Serial.printf("Sensor        : %s  laju %.1f Hz (target %.0f Hz)\n",
                g_sensorOk ? "OK" : "GAGAL", g_actualHz, SAMPLE_RATE_HZ);
  Serial.printf("Kalibrasi     : 0x%02X  bias gyro %.2f/%.2f/%.2f deg/s\n",
                Sensors.calibStatus(), Sensors.biasX(), Sensors.biasY(), Sensors.biasZ());
  Serial.printf("Fitur         : G=%.2fg  Gth=%.2fg  J=%.1f  R=%.1f  |w|=%.1f\n",
                f.g_force, f.g_threshold, f.jerk, f.rot_jerk, f.omega_mag);
  Serial.printf("Orientasi     : roll=%.1f  pitch=%.1f  yaw=%.1f deg\n",
                f.roll, f.pitch, f.yaw);
  Serial.printf("Koreksi       : k_slope=%.3f  k_speed=%.3f\n", f.k_slope, f.k_speed);
  Serial.printf("GPS           : %s  sat=%u  hdop=%.1f  %s\n",
                Gps.hasFix() ? "FIX" : "NO FIX", Gps.satellites(),
                Gps.current().hdop, Gps.coordString().c_str());
  Serial.printf("GSM           : state=%d  sinyal=%d%%\n",
                Gsm.state(), Gsm.signalPercent());
  Serial.printf("BLE           : %s  %s  adv=%d\n", Ble.deviceName().c_str(),
                Ble.isConnected() ? "TERHUBUNG" : "tidak terhubung",
                Ble.isAdvertising());
  Serial.printf("Baterai       : %u %%\n", g_batteryPct);
  Serial.printf("Kontak        : %u/%u\n", Store.contactCount(), MAX_CONTACTS);
  Serial.printf("Cuplikan      : %lu (gagal %lu)\n",
                (unsigned long)g_sampleCount, (unsigned long)g_missedSamples);
  Serial.println(F("============================"));
}

static void printContacts() {
  Serial.printf("\nKontak darurat (%u/%u):\n", Store.contactCount(), MAX_CONTACTS);
  const int8_t prim = Store.primaryContactIndex();
  for (uint8_t i = 0; i < Store.contactCount(); i++) {
    const EmergencyContact *c = Store.contact(i);
    if (!c) continue;
    Serial.printf("  [%u] %-16s %-16s prioritas=%u%s\n", i, c->name, c->number,
                  c->priority, (i == (uint8_t)prim) ? "  <- missed call" : "");
  }
  if (Store.contactCount() == 0) Serial.println(F("  (kosong)"));
}

static void printLiveFeatures() {
  Serial.println(F("\nG(g)   Gth(g)  J(m/s3)  R(deg/s2)  |w|    roll    pitch"));
  for (uint8_t i = 0; i < 20; i++) {
    SensorFeature f;
    LOCK(); f = g_lastFeature; UNLOCK();
    Serial.printf("%5.2f  %5.2f  %8.1f  %9.1f  %5.1f  %6.1f  %6.1f\n",
                  f.g_force, f.g_threshold, f.jerk, f.rot_jerk,
                  f.omega_mag, f.roll, f.pitch);
    vTaskDelay(pdMS_TO_TICKS(150));   /* tidak memblokir scheduler */
  }
}

static void handleSerialCli() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  String cmd = line;
  String arg = "";
  const int sp = line.indexOf(' ');
  if (sp > 0) { cmd = line.substring(0, sp); arg = line.substring(sp + 1); arg.trim(); }
  cmd.toLowerCase();

  if (cmd == "help" || cmd == "?")      { printHelp(); }
  else if (cmd == "status")             { printStatus(); }
  else if (cmd == "live")               { printLiveFeatures(); }
  else if (cmd == "contacts")           { printContacts(); }

  else if (cmd == "add") {
    /* add <nama> <nomor> <prioritas> */
    const int s1 = arg.indexOf(' ');
    const int s2 = arg.indexOf(' ', s1 + 1);
    if (s1 < 0) { Serial.println(F("Format: add <nama> <nomor> <prioritas>")); return; }
    String nama  = arg.substring(0, s1);
    String nomor = (s2 > 0) ? arg.substring(s1 + 1, s2) : arg.substring(s1 + 1);
    uint8_t pri  = (s2 > 0) ? (uint8_t)arg.substring(s2 + 1).toInt() : 0;
    if (Store.addContact(nama.c_str(), nomor.c_str(), pri)) {
      Store.save();
      printContacts();
    } else Serial.println(F("Gagal menambah kontak"));
  }

  else if (cmd == "del") {
    if (Store.removeContact((uint8_t)arg.toInt())) { Store.save(); printContacts(); }
    else Serial.println(F("Index tidak valid"));
  }

  else if (cmd == "profile") {
    if (Store.setProfileByName(arg.c_str())) {
      Detector.setProfile((VehicleProfile)Store.config().profile);
      Store.save();
      const ProfileThreshold &th = Detector.thresholds();
      Serial.printf("Profil -> %s (Gth=%.2fg Jth=%.0f Rth=%.0f)\n",
                    th.name, th.g_th, th.j_th, th.r_th);
    } else Serial.println(F("Pilihan: sepeda | motor | mobil"));
  }

  else if (cmd == "owner") {
    strncpy(Store.config().owner, arg.c_str(), OWNER_NAME_LEN - 1);
    Store.config().owner[OWNER_NAME_LEN - 1] = '\0';
    Store.save();
    Serial.printf("Pemilik -> %s\n", Store.config().owner);
  }

  else if (cmd == "countdown") {
    const long v = arg.toInt();
    if (v >= 5 && v <= 120) {
      Store.config().countdown_s = (uint16_t)v;
      Store.save();
      Serial.printf("Countdown -> %ld detik\n", v);
    } else Serial.println(F("Rentang 5-120 detik"));
  }

  else if (cmd == "baseline") {
    long det = arg.toInt();
    if (det < 10 || det > 900) det = 120;
    g_bMaxG = g_bMaxJ = g_bMaxR = g_bMaxTilt = 0;
    g_baselineEndMs  = millis() + (uint32_t)det * 1000UL;
    g_baselineActive = true;
    Serial.printf("\n[BASELINE] Merekam selama %ld detik.\n"
                  "Lakukan manuver normal: pengereman mendadak, akselerasi agresif,\n"
                  "polisi tidur, jalan berlubang, dan tikungan tajam (Bab 5.8).\n"
                  "JANGAN menjatuhkan atau membenturkan perangkat.\n", det);
  }

  else if (cmd == "calib")  { g_recalibReq = true; }
  else if (cmd == "sim")    { g_simulateReq = true; Serial.println(F("Pemicu simulasi diantrikan")); }
  else if (cmd == "cancel") { g_cancelReq = true; }

  else if (cmd == "sms") {
    const int8_t i = Store.primaryContactIndex();
    const EmergencyContact *c = i >= 0 ? Store.contact((uint8_t)i) : nullptr;
    if (c) Gsm.sendSMS(c->number, String("DRIVESAFE: SMS uji dari perangkat ") + Ble.deviceName());
    else   Serial.println(F("Belum ada kontak"));
  }

  else if (cmd == "gsm") {
    Gsm.refreshStatus();
    Serial.printf("GSM state=%d rssi=%d (%d%%)\n",
                  Gsm.state(), Gsm.rssi(), Gsm.signalPercent());
  }

  else if (cmd == "gps") {
    const GpsFix f = Gps.current();
    Serial.printf("fix=%d sat=%u hdop=%.1f %.6f,%.6f %.1f km/h epoch=%lu\n",
                  f.valid, f.satellites, f.hdop, f.lat, f.lon, f.speed_kmh,
                  (unsigned long)Gps.epochNow());
    Serial.println(Gps.mapsLink());
  }

  else if (cmd == "pair")   { g_pairingOn = true;  Ble.startPairing(); }
  else if (cmd == "unpair") { g_pairingOn = false; Ble.stopPairing(); }

  else if (cmd == "log") {
    EventLog ev;
    if (Store.lastEvent(ev)) {
      Serial.printf("Event terakhir: ts=%lu G=%.2fg J=%.1f R=%.1f "
                    "roll=%.1f pitch=%.1f sev=%s tipe=%s %.6f,%.6f flags=0x%02X\n",
                    (unsigned long)ev.timestamp, ev.g_peak, ev.j_peak, ev.r_peak,
                    ev.roll_peak, ev.pitch_peak, severityName(ev.severity),
                    crashTypeName(ev.crash_type), ev.lat, ev.lon, ev.trigger_flags);
    } else Serial.println(F("Belum ada event tersimpan"));
  }

  else if (cmd == "mem") {
    Serial.printf("Buffer event : %lu byte (%.2f%% dari SRAM 520 KB)\n",
                  (unsigned long)EventBuffer::memoryBytes(),
                  (EventBuffer::memoryBytes() + 64) * 100.0f / 532480.0f);
    Serial.printf("Kontak NVS   : %lu byte\n",
                  (unsigned long)DriveSafeStorage::contactMemoryBytes());
    Serial.printf("Free heap    : %lu byte  (min %lu)\n",
                  (unsigned long)ESP.getFreeHeap(),
                  (unsigned long)ESP.getMinFreeHeap());
    Serial.printf("Sampel buffer: %u/%u\n", EvBuf.totalSamples(), TOTAL_SAMPLES);
  }

  else Serial.printf("Perintah tidak dikenal: %s (ketik 'help')\n", cmd.c_str());
}

/* ==========================================================================
 *  setup()
 * ========================================================================== */
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);   /* agar readStringUntil tidak memblokir task */
  delay(400);

  Serial.println();
  Serial.println(F("============================================================"));
  Serial.println(F("  DriveSafe - Sistem Deteksi Kecelakaan Kendaraan Berbasis IoT"));
  Serial.println(F("  Dokumen C-251 | Capstone B-06 | DTETI FT UGM"));
  Serial.printf ("  Firmware %s   CPU %u MHz   Flash %u MB\n",
                 FW_VERSION, (unsigned)getCpuFrequencyMhz(),
                 (unsigned)(ESP.getFlashChipSize() / (1024 * 1024)));
  Serial.println(F("============================================================"));

  g_mux = xSemaphoreCreateMutex();
  memset(&g_event, 0, sizeof(g_event));

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_VBAT_ADC, ADC_11db);

  Ui.begin();
  setState(ST_BOOT);

  Store.begin();
  EvBuf.begin();
  Detector.begin((VehicleProfile)Store.config().profile);

  /* --- Sensor inersial --------------------------------------------------- */
  if (!Sensors.begin()) {
    Serial.println(F("[FATAL] BNO055 tidak terdeteksi. Periksa kabel SDA/SCL "
                     "dan alamat I2C (0x28/0x29)."));
    Ui.setBuzzer(BUZZ_ERROR);
  } else {
    uint8_t calib[22];
    if (Store.hasCalib() && Store.loadCalib(calib)) {
      Sensors.writeCalibOffsets(calib);
      Serial.println(F("[SENSOR] Offset kalibrasi dimuat dari NVS"));
    }
    setState(ST_CALIBRATING);
    Sensors.calibrateBias(2000);
    g_sensorOk = true;
  }

  /* --- GPS & GSM --------------------------------------------------------- */
  Gps.begin();
  Gsm.begin();

  /* --- BLE --------------------------------------------------------------- */
  Ble.begin();
  Ble.startPairing();          /* advertising aktif saat boot agar aplikasi
                                  dapat langsung melakukan pairing awal      */
  g_pairingOn = true;

  g_batteryPct = readBatteryPercent();

  /* --- Task -------------------------------------------------------------- */
  xTaskCreatePinnedToCore(sensorTask, "sensor", 6144, nullptr, 3, nullptr, 1);
  xTaskCreatePinnedToCore(commsTask,  "comms",  8192, nullptr, 1, nullptr, 0);

  setState(ST_MONITORING);
  Ui.setBuzzer(BUZZ_BOOT);

  Serial.printf("\n[SISTEM] Siap. Profil=%s  Kontak=%u  Countdown=%us\n",
                Store.profileName(), Store.contactCount(),
                Store.config().countdown_s);
  Serial.println(F("[SISTEM] Ketik 'help' untuk daftar perintah pengujian.\n"));
}

/* ==========================================================================
 *  loop() dibiarkan kosong: seluruh pekerjaan dijalankan oleh dua task.
 * ========================================================================== */
void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
