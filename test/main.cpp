#include <unity.h>
#include <Arduino.h>

// Include test suites (they define the test functions)
#include "test_pid.cpp"
#include "test_kalman.cpp"
#include "test_imu.cpp"

void setUp(void) {
    // Set up test fixtures before each test
}

void tearDown(void) {
    // Clean up after each test
}

void setup() {
    // Wait for serial monitor to connect (optional)
    delay(2000);
    
    Serial.begin(115200);
    Serial.println("\n\n=== SBR Unit Tests ===");
    Serial.println("Starting test suite...\n");
    
    UNITY_BEGIN();
    
    // Run test suites
    Serial.println("Running PID tests...");
    test_pid_suite();
    
    Serial.println("\nRunning Kalman filter tests...");
    test_kalman_suite();
    
    Serial.println("\nRunning IMU algorithm tests...");
    test_imu_suite();
    
    Serial.println("\n=== Test Summary ===");
    UNITY_END();
}

void loop() {
    // Tests run once in setup()
    delay(1000);
}

