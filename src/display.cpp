#include "display.h"

// Bitmaps for smiley and sad faces
const uint8_t PROGMEM Display::smileyBitmap[8] = {
  B00111100,
  B01111110,
  B11111111,
  B11111111,
  B11111111,
  B11111111,
  B01111110,
  B00111100
};

const uint8_t PROGMEM Display::sadBitmap[8] = {
  B00001100,
  B00011100,
  B00111000,
  B00110000,
  B00110000,
  B00111000,
  B00011100,
  B00001100
};

Display::Display() : displayMatrix(HARDWARE_TYPE, DISPLAY_CS1_PIN, MAX_DEVICES) {
  lastDisplayUpdateMs = 0;
}

void Display::begin() {
  displayMatrix.begin();
  displayMatrix.control(MD_MAX72XX::INTENSITY, 2); // Set brightness (0-15)
  displayBitmap(0, sadBitmap);
  displayBitmap(1, smileyBitmap);
}

void Display::update() {
  static bool displayState = false;
  unsigned long now = millis();
  
  // Update animation every 1000ms
  if (now - lastDisplayUpdateMs >= 1000) {
    if (displayState) {
      // Display smiley face on Module 0
      displayBitmap(0, smileyBitmap);
      // Display sad face on Module 1
      displayBitmap(1, sadBitmap);
    } else {
      // Swap images
      displayBitmap(0, sadBitmap);
      displayBitmap(1, smileyBitmap);
    }
    displayState = !displayState;
    lastDisplayUpdateMs = now;
  }
}

void Display::displayBitmap(uint8_t device, const uint8_t *bitmap) {
  for (uint8_t row = 0; row < 8; row++) {
    displayMatrix.setRow(device, row, pgm_read_byte(&bitmap[row]));
  }
}
