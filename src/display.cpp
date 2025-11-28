#include "display.h"

// Create a new instance of the MD_MAX72XX class with hardware SPI
MD_MAX72XX mx = MD_MAX72XX(HARDWARE_TYPE, DISPLAY_CS1_PIN, MAX_DEVICES);

// Bitmaps for smiley and sad faces
const uint8_t PROGMEM smiley[] = {
  B00111100,
  B01111110,
  B11111111,
  B11111111,
  B11111111,
  B11111111,
  B01111110,
  B00111100
};

const uint8_t PROGMEM sad[] = {
  B00001100,
  B00011100,
  B00111000,
  B00110000,
  B00110000,
  B00111000,
  B00011100,
  B00001100
};

void displaySetup() {
  mx.begin();
  mx.control(MD_MAX72XX::INTENSITY, 2); // Set brightness (0-15)
  displayBitmap(0, sad);
  displayBitmap(1, smiley);
}

void displayUpdate() {
  static unsigned long lastUpdate = 0;
  static bool state = false;
  unsigned long now = millis();
  
  // Update animation every 1000ms
  if (now - lastUpdate >= 1000) {
    if (state) {
      // Display smiley face on Module 0
      displayBitmap(0, smiley);
      // Display sad face on Module 1
      displayBitmap(1, sad);
    } else {
      // Swap images
      displayBitmap(0, sad);
      displayBitmap(1, smiley);
    }
    state = !state;
    lastUpdate = now;
  }
}

// Function to display a bitmap on a specific module
void displayBitmap(uint8_t device, const uint8_t *bitmap) {
  for (uint8_t row = 0; row < 8; row++) {
    mx.setRow(device, row, pgm_read_byte(&bitmap[row]));
  }
}

