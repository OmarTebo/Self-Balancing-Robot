// test_imu.cpp - Unit tests for IMU algorithm (angle calculation)
// Note: These tests verify the algorithm, not the full IMU class (which requires I2C hardware)

#include <unity.h>
#include <cmath>
#include <Arduino.h>

// Test the angle calculation algorithm directly (same as firmware)
// This matches the atan2 calculation from MPU6050.cpp

// Algorithm: Same as firmware
static float calculate_roll(float ay_g, float az_g) {
    return atan2f(ay_g, az_g) * 180.0f / M_PI;
}

static float calculate_pitch(float ax_g, float ay_g, float az_g) {
    return atan2f(-ax_g, sqrtf(ay_g * ay_g + az_g * az_g)) * 180.0f / M_PI;
}

static float apply_calibration_offset(float angle, float offset) {
    return angle - offset;
}

// Note: setUp() and tearDown() are defined in main.cpp
// This file only defines test functions

// Test 1: Roll calculation - level position
void test_roll_calculation_level() {
    // Level: ay = 0, az = 1.0g
    float roll = calculate_roll(0.0f, 1.0f);
    
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, roll);
}

// Test 2: Roll calculation - 10 degree tilt
void test_roll_calculation_tilt_10deg() {
    // 10 degree roll: ay = sin(10°), az = cos(10°)
    float roll = calculate_roll(sinf(10.0f * M_PI / 180.0f), 
                                cosf(10.0f * M_PI / 180.0f));
    
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 10.0f, roll);
}

// Test 3: Roll calculation - 45 degree tilt
void test_roll_calculation_tilt_45deg() {
    // 45 degree roll
    float roll = calculate_roll(sinf(45.0f * M_PI / 180.0f), 
                                cosf(45.0f * M_PI / 180.0f));
    
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 45.0f, roll);
}

// Test 4: Roll calculation - negative tilt
void test_roll_calculation_negative_tilt() {
    // -10 degree roll
    float roll = calculate_roll(sinf(-10.0f * M_PI / 180.0f), 
                                cosf(-10.0f * M_PI / 180.0f));
    
    TEST_ASSERT_FLOAT_WITHIN(0.5f, -10.0f, roll);
}

// Test 5: Pitch calculation - level position
void test_pitch_calculation_level() {
    // Level: ax = 0, ay = 0, az = 1.0g
    float pitch = calculate_pitch(0.0f, 0.0f, 1.0f);
    
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, pitch);
}

// Test 6: Pitch calculation - 10 degree pitch
void test_pitch_calculation_pitch_10deg() {
    // 10 degree pitch: ax = -sin(10°), ay = 0, az = cos(10°)
    float pitch = calculate_pitch(-sinf(10.0f * M_PI / 180.0f), 
                                  0.0f, 
                                  cosf(10.0f * M_PI / 180.0f));
    
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 10.0f, pitch);
}

// Test 7: Pitch calculation - negative pitch
void test_pitch_calculation_negative_pitch() {
    // -10 degree pitch
    float pitch = calculate_pitch(sinf(10.0f * M_PI / 180.0f), 
                                  0.0f, 
                                  cosf(10.0f * M_PI / 180.0f));
    
    TEST_ASSERT_FLOAT_WITHIN(0.5f, -10.0f, pitch);
}

// Test 8: Calibration offset application
void test_calibration_offset_application() {
    // Original angle: 10 degrees
    // Offset: 2 degrees
    // Result: 10 - 2 = 8 degrees
    float angle = apply_calibration_offset(10.0f, 2.0f);
    
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 8.0f, angle);
}

// Test 9: Calibration offset - negative offset
void test_calibration_offset_negative() {
    // Original angle: 5 degrees
    // Offset: -3 degrees
    // Result: 5 - (-3) = 8 degrees
    float angle = apply_calibration_offset(5.0f, -3.0f);
    
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 8.0f, angle);
}

// Test 10: Combined roll and pitch calculation
void test_combined_roll_pitch() {
    // 10 degree roll, 5 degree pitch
    float roll_angle = 10.0f;
    float pitch_angle = 5.0f;
    
    float ax = -sinf(pitch_angle * M_PI / 180.0f);
    float ay = sinf(roll_angle * M_PI / 180.0f);
    float az = cosf(roll_angle * M_PI / 180.0f) * cosf(pitch_angle * M_PI / 180.0f);
    
    float roll = calculate_roll(ay, az);
    float pitch = calculate_pitch(ax, ay, az);
    
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 10.0f, roll);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 5.0f, pitch);
}

// Test 11: Edge case - 90 degree roll
void test_roll_90_degrees() {
    // 90 degree roll: ay = 1.0, az = 0.0
    float roll = calculate_roll(1.0f, 0.0f);
    
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 90.0f, roll);
}

// Test 12: Edge case - -90 degree roll
void test_roll_negative_90_degrees() {
    // -90 degree roll: ay = -1.0, az = 0.0
    float roll = calculate_roll(-1.0f, 0.0f);
    
    TEST_ASSERT_FLOAT_WITHIN(1.0f, -90.0f, roll);
}

// Test 13: Algorithm consistency - same input gives same output
void test_algorithm_consistency() {
    float ay = 0.174f; // sin(10°)
    float az = 0.985f; // cos(10°)
    
    float roll1 = calculate_roll(ay, az);
    float roll2 = calculate_roll(ay, az);
    
    TEST_ASSERT_FLOAT_WITHIN(0.001f, roll1, roll2);
}

// Test 14: Zero acceleration handling
void test_zero_acceleration() {
    // Edge case: all zeros
    float roll = calculate_roll(0.0f, 0.0f);
    float pitch = calculate_pitch(0.0f, 0.0f, 0.0f);
    
    // Should not crash, may return 0 or NaN
    TEST_ASSERT_TRUE(!isinf(roll));
    TEST_ASSERT_TRUE(!isinf(pitch));
}

// Test suite runner
void test_imu_suite() {
    RUN_TEST(test_roll_calculation_level);
    RUN_TEST(test_roll_calculation_tilt_10deg);
    RUN_TEST(test_roll_calculation_tilt_45deg);
    RUN_TEST(test_roll_calculation_negative_tilt);
    RUN_TEST(test_pitch_calculation_level);
    RUN_TEST(test_pitch_calculation_pitch_10deg);
    RUN_TEST(test_pitch_calculation_negative_pitch);
    RUN_TEST(test_calibration_offset_application);
    RUN_TEST(test_calibration_offset_negative);
    RUN_TEST(test_combined_roll_pitch);
    RUN_TEST(test_roll_90_degrees);
    RUN_TEST(test_roll_negative_90_degrees);
    RUN_TEST(test_algorithm_consistency);
    RUN_TEST(test_zero_acceleration);
}

