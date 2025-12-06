// test_pid.cpp - Unit tests for PID controller
#include <unity.h>
#include "../include/PIDController.h"
#include "../include/Config.h"
#include <cmath>
#include <Arduino.h>

// Test fixture
PIDController pid;

void setUp(void) {
    pid = PIDController();
}

void tearDown(void) {
    // Cleanup if needed
}

// Test 1: PID initialization
void test_pid_initialization() {
    pid.begin(1.0f, 0.5f, 0.1f, -100.0f, 100.0f);
    
    float kp, ki, kd;
    pid.getTunings(kp, ki, kd);
    
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, kp);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.5f, ki);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.1f, kd);
}

// Test 2: PID computation with known inputs (P term only)
void test_pid_proportional_only() {
    pid.begin(2.0f, 0.0f, 0.0f, PID_OUTPUT_MIN_F, PID_OUTPUT_MAX_F);
    pid.reset();
    
    // Setpoint = 10°, Measurement = 5° -> Error = 5°
    // Output = kp * error = 2.0 * 5.0 = 10.0 deg/s
    float output = pid.compute(10.0f, 5.0f, 0.005f);
    
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 10.0f, output);
}

// Test 3: PID computation with integral term
void test_pid_integral_term() {
    pid.begin(1.0f, 2.0f, 0.0f, PID_OUTPUT_MIN_F, PID_OUTPUT_MAX_F);
    pid.reset();
    
    // Constant error of 5° over 0.1 seconds
    // I term accumulates: integral += error * dt = 5.0 * 0.1 = 0.5
    // After 10 iterations (1 second total): integral = 5.0
    // I = ki * integral = 2.0 * 5.0 = 10.0
    float total_I = 0.0f;
    for (int i = 0; i < 10; i++) {
        pid.compute(10.0f, 5.0f, 0.1f);
    }
    float output = pid.compute(10.0f, 5.0f, 0.1f);
    
    // P = 1.0 * 5.0 = 5.0, I ≈ 2.0 * 5.0 = 10.0, Total ≈ 15.0
    TEST_ASSERT_TRUE(output > 10.0f); // Should have significant I contribution
}

// Test 4: PID derivative term
void test_pid_derivative_term() {
    pid.begin(1.0f, 0.0f, 1.0f, PID_OUTPUT_MIN_F, PID_OUTPUT_MAX_F);
    pid.reset();
    
    // First call: error = 10°, prevError = 0° -> derivative = 10/0.005 = 2000 deg/s
    // But derivative is filtered, so expect some response
    float output1 = pid.compute(10.0f, 0.0f, 0.005f);
    
    // Second call: error still 10°, derivative should be smaller (error not changing)
    float output2 = pid.compute(10.0f, 0.0f, 0.005f);
    
    // First output should be higher due to initial derivative spike
    TEST_ASSERT_TRUE(output1 > output2 || fabs(output1 - output2) < 1.0f);
}

// Test 5: Output clamping
void test_pid_output_clamping() {
    pid.begin(1000.0f, 0.0f, 0.0f, -100.0f, 100.0f); // High gain, tight limits
    pid.reset();
    
    // Large error should be clamped
    float output = pid.compute(100.0f, 0.0f, 0.005f);
    
    TEST_ASSERT_TRUE(output <= 100.0f);
    TEST_ASSERT_TRUE(output >= -100.0f);
}

// Test 6: Integral windup prevention
void test_pid_integral_windup_prevention() {
    pid.begin(1.0f, 10.0f, 0.0f, -50.0f, 50.0f); // High Ki, tight limits
    pid.reset();
    
    // Large constant error that would cause windup
    // Run many iterations with saturated output
    for (int i = 0; i < 100; i++) {
        float out = pid.compute(100.0f, 0.0f, 0.005f);
        // Output should stay clamped
        TEST_ASSERT_TRUE(out <= 50.0f);
        TEST_ASSERT_TRUE(out >= -50.0f);
    }
    
    // Now reverse error - integral should not cause overshoot
    float output = pid.compute(0.0f, 100.0f, 0.005f);
    TEST_ASSERT_TRUE(output >= -50.0f); // Should not exceed limits
}

// Test 7: Reset function
void test_pid_reset() {
    pid.begin(1.0f, 1.0f, 1.0f, PID_OUTPUT_MIN_F, PID_OUTPUT_MAX_F);
    
    // Build up some state
    for (int i = 0; i < 10; i++) {
        pid.compute(10.0f, 5.0f, 0.005f);
    }
    
    float output_before = pid.compute(10.0f, 5.0f, 0.005f);
    
    pid.reset();
    
    // After reset, output should be different (integral cleared)
    float output_after = pid.compute(10.0f, 5.0f, 0.005f);
    
    // Outputs should be different (integral was cleared)
    TEST_ASSERT_TRUE(fabs(output_before - output_after) > 0.1f);
}

// Test 8: setTunings function
void test_pid_set_tunings() {
    pid.begin(1.0f, 0.5f, 0.1f, PID_OUTPUT_MIN_F, PID_OUTPUT_MAX_F);
    
    pid.setTunings(2.0f, 1.0f, 0.2f);
    
    float kp, ki, kd;
    pid.getTunings(kp, ki, kd);
    
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.0f, kp);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, ki);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.2f, kd);
}

// Test 9: Zero dt handling
void test_pid_zero_dt() {
    pid.begin(1.0f, 1.0f, 1.0f, PID_OUTPUT_MIN_F, PID_OUTPUT_MAX_F);
    pid.reset();
    
    // Zero dt should return 0
    float output = pid.compute(10.0f, 5.0f, 0.0f);
    
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, output);
}

// Test 10: Negative dt handling
void test_pid_negative_dt() {
    pid.begin(1.0f, 1.0f, 1.0f, PID_OUTPUT_MIN_F, PID_OUTPUT_MAX_F);
    pid.reset();
    
    // Negative dt should return 0
    float output = pid.compute(10.0f, 5.0f, -0.005f);
    
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, output);
}

// Test suite runner
void test_pid_suite() {
    RUN_TEST(test_pid_initialization);
    RUN_TEST(test_pid_proportional_only);
    RUN_TEST(test_pid_integral_term);
    RUN_TEST(test_pid_derivative_term);
    RUN_TEST(test_pid_output_clamping);
    RUN_TEST(test_pid_integral_windup_prevention);
    RUN_TEST(test_pid_reset);
    RUN_TEST(test_pid_set_tunings);
    RUN_TEST(test_pid_zero_dt);
    RUN_TEST(test_pid_negative_dt);
}

