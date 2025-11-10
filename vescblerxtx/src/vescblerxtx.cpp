#include "vescblerxtx.h"
#include <string.h>
#include <math.h>

namespace vescblerxtx {

  static WriteFn s_writer = nullptr;

  static uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
    uint16_t crc = 0;
    for (size_t i = 0; i < len; ++i) {
      crc ^= (uint16_t)data[i] << 8;
      for (int j = 0; j < 8; j++) {
        if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
        else              crc = (crc << 1);
      }
    }
    return crc;
  }

  static void write_i32_be(uint8_t* p, int32_t v) {
    p[0] = (uint8_t)((v >> 24) & 0xFF);
    p[1] = (uint8_t)((v >> 16) & 0xFF);
    p[2] = (uint8_t)((v >> 8)  & 0xFF);
    p[3] = (uint8_t)( v        & 0xFF);
  }

  static size_t vesc_build_packet(const uint8_t* payload, size_t len, uint8_t* out) {
    size_t i = 0;
    if (len <= 255) { out[i++] = 2; out[i++] = (uint8_t)len; }
    else            { out[i++] = 3; out[i++] = (len >> 8) & 0xFF; out[i++] = len & 0xFF; }
    memcpy(out + i, payload, len); i += len;
    uint16_t crc = crc16_ccitt(payload, len);
    out[i++] = (crc >> 8) & 0xFF;
    out[i++] = (crc) & 0xFF;
    out[i++] = 3;
    return i;
  }

  void set_writer(WriteFn fn) { s_writer = fn; }
  bool is_ready() {
    #ifdef VESCBLERXTX_ENABLE_NIMBLE
      extern bool __vesc_tx_is_bound_nimble();
      if (__vesc_tx_is_bound_nimble()) return true;
    #endif
    return (s_writer != nullptr);
  }

  static bool send_framed(const uint8_t* payload, size_t len) {
    uint8_t frame[128];
    size_t n = vesc_build_packet(payload, len, frame);
    #ifdef VESCBLERXTX_ENABLE_NIMBLE
      extern bool __vesc_tx_write_nimble(const uint8_t* data, size_t len);
      if (__vesc_tx_write_nimble(frame, n)) return true;
    #endif
    if (s_writer) return s_writer(frame, n);
    return false;
  }

  bool send_packet(const uint8_t* payload, size_t len) { return send_framed(payload, len); }

  bool set_current(float amps) { // scale 1000
    int32_t q = (int32_t)lroundf(amps * 1000.0f);
    uint8_t p[1 + 4]; p[0] = 6; write_i32_be(&p[1], q);
    return send_framed(p, sizeof(p));
  }
  bool set_brake(float amps) { // scale 1000
    if (amps < 0) amps = -amps;
    int32_t q = (int32_t)lroundf(amps * 1000.0f);
    uint8_t p[1 + 4]; p[0] = 7; write_i32_be(&p[1], q);
    return send_framed(p, sizeof(p));
  }
  bool set_rpm(int rpm) { // scale 1
    uint8_t p[1 + 4]; p[0] = 8; write_i32_be(&p[1], rpm);
    return send_framed(p, sizeof(p));
  }

} // namespace

#ifdef VESCBLERXTX_ENABLE_NIMBLE
  #include <NimBLEDevice.h>
  namespace vescblerxtx { static NimBLERemoteCharacteristic* s_tx = nullptr; void set_tx(NimBLERemoteCharacteristic* tx){ s_tx = tx; } }
  bool __vesc_tx_is_bound_nimble() { using namespace vescblerxtx; extern NimBLERemoteCharacteristic* s_tx; return s_tx != nullptr; }
  bool __vesc_tx_write_nimble(const uint8_t* data, size_t len) { using namespace vescblerxtx; extern NimBLERemoteCharacteristic* s_tx; if(!s_tx) return false; return s_tx->writeValue((uint8_t*)data, len, false); }
#endif
