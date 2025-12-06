#ifndef DISPLAY_H
#define DISPLAY_H

#include <Arduino.h>
#include <MD_MAX72xx.h>
#include <SPI.h>
#include "Config.h"

// Define the number of devices (8x8 LED matrices) cascaded
#define MAX_DEVICES 2

// Define the type of hardware. Use FC16_HW for generic MAX7219 modules
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW

class Display {
public:
  Display();
  void begin();
  void update();
  
private:
  void displayBitmap(uint8_t device, const uint8_t *bitmap);
  MD_MAX72XX displayMatrix;
  unsigned long lastDisplayUpdateMs;
  
  // Bitmaps for smiley and sad faces
  static const uint8_t PROGMEM smileyBitmap[8];
  static const uint8_t PROGMEM sadBitmap[8];
};

#endif  // DISPLAY_H
