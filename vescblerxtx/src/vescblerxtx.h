#pragma once
#include <Arduino.h>

namespace vescblerxtx {

  using WriteFn = bool (*)(const uint8_t* data, size_t len);

  void set_writer(WriteFn fn);
  bool is_ready();

  bool set_current(float amps);
  bool set_brake(float amps);
  bool set_rpm(int rpm);

  bool send_packet(const uint8_t* payload, size_t len);
}

#ifdef VESCBLERXTX_ENABLE_NIMBLE
  #include <NimBLEDevice.h>
  namespace vescblerxtx { void set_tx(NimBLERemoteCharacteristic* tx); }
#endif
