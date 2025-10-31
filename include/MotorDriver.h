#pragma once
#include <AccelStepper.h>
#include <Arduino.h>
#include "Config.h"

class MotorDriver {
public:
  MotorDriver(uint8_t stepPin, uint8_t dirPin, int8_t enPin = -1);
  void begin();
  void setSpeedStepsPerSec(float stepsPerSec); // positive/negative
  void runSpeed(); // call frequently
  void enable(bool en);
  void setMaxSpeed(float s);
  void setAcceleration(float a);

private:
  AccelStepper stepper;
  int8_t enPin;
};
