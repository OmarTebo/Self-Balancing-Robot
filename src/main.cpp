#include <Arduino.h>
#include "BotController.h"
#include "SerialBridge.h"
#include "Config.h"

BotController controller;
SerialBridge serialBridge;

unsigned long lastMicros = 0;
const unsigned long tickMicros = (1000000UL / CONTROL_LOOP_HZ);

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(50);
  Serial.println("SBR minimal starting...");
  controller.begin();
  serialBridge.begin(SERIAL_BAUD);
  lastMicros = micros();
}

void loop() {
  unsigned long now = micros();
  if (now - lastMicros >= tickMicros) {
    float dt = (now - lastMicros) / 1000000.0f;
    lastMicros += tickMicros; // keep fixed-step
    controller.update(dt);
  }

  // poll serial commands without blocking
  PIDParams p;
  if (serialBridge.poll(p)) {
    controller.requestPidParams((PIDParams&)p);
    Serial.printf("Applied PID from serial: kp=%.4f ki=%.6f kd=%.6f\n", p.kp, p.ki, p.kd);
  }
  // respond to GET PID requests
  if (serialBridge.consumeGetPidRequest()) {
    controller.printCurrentPid();
  }

  // small yield
  delay(0);
}
