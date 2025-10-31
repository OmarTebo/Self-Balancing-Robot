#pragma once
#include "Config.h"
#include "Types.h"
#include <Arduino.h>

class BLEHandler {
public:
  BLEHandler();
  void begin();
  // callback from BLE when new params available: returns true if new pending set
  bool takePending(PIDParams &out);

private:
  void setupBleServer();
  // internal pending buffer
  volatile bool _hasPending;
  PIDParams _pending;
  // mutex for atomic transfer
  portMUX_TYPE mux;
  // friend function for callbacks
  friend void bleWriteCallback(const void* data, size_t len);
};
