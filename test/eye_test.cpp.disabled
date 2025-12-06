#include <Arduino.h>
#include <MD_MAX72xx.h>
#include <SPI.h>

// Pin definitions (from Config.h)
#define DISPLAY_DIN_PIN 23
#define DISPLAY_CLK_PIN 18
#define DISPLAY_CS1_PIN 17
#define DISPLAY_CS2_PIN 16

// Display configuration
#define MAX_DEVICES 2
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW

// Create display instance
MD_MAX72XX mx = MD_MAX72XX(HARDWARE_TYPE, DISPLAY_CS1_PIN, MAX_DEVICES);

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("LED Matrix Eye Test - Making both eyes glow...");
  
  // Initialize display
  mx.begin();
  mx.control(MD_MAX72XX::INTENSITY, 15); // Maximum brightness (0-15)
  mx.control(MD_MAX72XX::UPDATE, MD_MAX72XX::ON);
  
  // Make both eyes glow (all LEDs on)
  // Fill all rows on both devices with all 1s (0xFF = all LEDs on)
  for (uint8_t device = 0; device < MAX_DEVICES; device++) {
    for (uint8_t row = 0; row < 8; row++) {
      mx.setRow(device, row, 0xFF); // 0xFF = all 8 LEDs in row ON
    }
  }
  
  Serial.println("Both eyes should now be glowing!");
}

void loop() {
  // Keep the eyes glowing - nothing to do in loop
  delay(1000);
}

