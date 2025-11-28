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

// External declaration of the MD_MAX72XX object
extern MD_MAX72XX mx;

// Function prototypes
void displaySetup();
void displayUpdate();
void displayBitmap(uint8_t device, const uint8_t *bitmap);

#endif  // DISPLAY_H

