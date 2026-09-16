/* ============================================================================
 *  DriveSafe - eventbuf.cpp
 * ========================================================================== */
#include "eventbuf.h"

EventBuffer EvBuf;

void EventBuffer::begin() {
  _preHead = _preCount = _postCount = 0;
  _capturing = false;

  Serial.printf("[BUFFER] Pre %d + Post %d sampel = %lu byte (%.1f KB)\n",
                PRE_SAMPLES, POST_SAMPLES,
                (unsigned long)memoryBytes(), memoryBytes() / 1024.0f);
  Serial.printf("[BUFFER] Beban terhadap SRAM 520 KB: %.2f %%\n",
                (memoryBytes() + 64) * 100.0f / 532480.0f);
}

void EventBuffer::push(const SensorSample &s) {
  if (!_capturing) {
    /* Mode normal: circular buffer 10 detik terakhir */
    _pre[_preHead] = s;
    _preHead = (uint16_t)((_preHead + 1) % PRE_SAMPLES);
    if (_preCount < PRE_SAMPLES) _preCount++;
  } else if (_postCount < POST_SAMPLES) {
    /* Pascakejadian: rekam linear 5 detik lalu berhenti */
    _post[_postCount++] = s;
  }
}

void EventBuffer::armPostCapture() {
  if (_capturing) return;
  _capturing = true;
  _postCount = 0;
}

void EventBuffer::release() {
  _capturing = false;
  _postCount = 0;
}

bool EventBuffer::at(uint16_t index, SensorSample &out) const {
  if (index < _preCount) {
    /* Sampel tertua berada di (_preHead - _preCount) modulo PRE_SAMPLES */
    uint16_t start = (uint16_t)((_preHead + PRE_SAMPLES - _preCount) % PRE_SAMPLES);
    out = _pre[(uint16_t)((start + index) % PRE_SAMPLES)];
    return true;
  }
  uint16_t j = (uint16_t)(index - _preCount);
  if (j < _postCount) { out = _post[j]; return true; }
  return false;
}
