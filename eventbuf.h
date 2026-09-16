/* ============================================================================
 *  DriveSafe - eventbuf.h
 *  Circular buffer pre-impact 10 s + buffer linear post-impact 5 s.
 *
 *  Sub-bab 4.6.1 / Pers. 4.27:
 *      M_buffer = 6 sumbu x 1500 sampel x 4 byte = 36.000 byte ~ 35,2 KB
 *  Seluruhnya dialokasikan statis pada SRAM internal ESP32 (520 KB),
 *  tanpa media penyimpanan eksternal (Tabel 6.1).
 * ========================================================================== */
#ifndef DRIVESAFE_EVENTBUF_H
#define DRIVESAFE_EVENTBUF_H

#include "config.h"

class EventBuffer {
public:
  void  begin();

  /* Dipanggil tiap sampel (100 Hz). Menulis ke pre-buffer saat monitoring,
     atau ke post-buffer setelah armPostCapture(). */
  void  push(const SensorSample &s);

  /* Bekukan pre-buffer dan mulai merekam 5 detik pascakejadian. */
  void  armPostCapture();

  /* Buang hasil rekaman dan kembali merekam pre-buffer secara normal. */
  void  release();

  bool  postCaptureDone()  const { return _postCount >= POST_SAMPLES; }
  bool  isCapturing()      const { return _capturing; }
  uint16_t preCount()      const { return _preCount;  }
  uint16_t postCount()     const { return _postCount; }

  /* Ambil sampel ke-i secara kronologis (0 = paling lama, 
     indeks 0..preCount()-1 = pre-impact, sisanya post-impact). */
  bool  at(uint16_t index, SensorSample &out) const;
  uint16_t totalSamples() const { return _preCount + _postCount; }

  static uint32_t memoryBytes() { return (uint32_t)TOTAL_SAMPLES * sizeof(SensorSample); }

private:
  SensorSample _pre[PRE_SAMPLES];
  SensorSample _post[POST_SAMPLES];
  uint16_t _preHead   = 0;      /* posisi tulis berikutnya pada pre-buffer  */
  uint16_t _preCount  = 0;
  uint16_t _postCount = 0;
  bool     _capturing = false;
};

extern EventBuffer EvBuf;

#endif /* DRIVESAFE_EVENTBUF_H */
