#include "ROV.h"
#include <stdio.h>

ROV::ROV(uint thrust1_ena, uint thrust1_in1, uint thrust1_in2,
         uint thrust2_ena, uint thrust2_in1, uint thrust2_in2,
         uint ballast_ena, uint ballast_in1, uint ballast_in2,
         uint full_switch_pin, uint empty_switch_pin,
         uint moisture_digital_pin, uint moisture_analog_pin,
         uint pressure_out_pin, uint pressure_sck_pin)
    : thrust_motor1(thrust1_ena, thrust1_in1, thrust1_in2),
      thrust_motor2(thrust2_ena, thrust2_in1, thrust2_in2),
      ballast_motor(ballast_ena, ballast_in1, ballast_in2),
      full_switch(full_switch_pin),
      empty_switch(empty_switch_pin),
      moisture_sensor(moisture_digital_pin, moisture_analog_pin),
      pressure_sensor(pressure_out_pin, pressure_sck_pin),
      current_depth(0.0f),
      pressure_offset(0.0f),
      pressure_to_depth_factor(1.0f),
      zero_depth_pressure(0.0f),
      moisture_detected(false) {
    
    printf("ROV initialized successfully!\n");
    printf("Thrust Motors: ENA1=%d, ENA2=%d | Ballast: ENA=%d\n", 
           thrust1_ena, thrust2_ena, ballast_ena);
    printf("Limit Switches: Full=%d, Empty=%d\n", full_switch_pin, empty_switch_pin);
    printf("Moisture Sensor: Digital=%d, Analog=%d\n", moisture_digital_pin, moisture_analog_pin);
    printf("Pressure Sensor: OUT=%d, SCK=%d\n", pressure_out_pin, pressure_sck_pin);
}

float ROV::getDepth() {
    // Read pressure sensor (single sample to avoid blocking and reduce interference window)
    float pressure_reading = pressure_sensor.readAveraged(1); // Single reading - fast and less affected by transient noise
    
    // Apply conversion: depth = (pressure - offset) * factor
    // Then subtract the zero depth reference pressure to get relative depth
    current_depth = ((pressure_reading - zero_depth_pressure) - pressure_offset) * pressure_to_depth_factor;
    
    // Ensure depth is non-negative
    if (current_depth < 0.0f) {
        current_depth = 0.0f;
    }
    
    return current_depth;
}

void ROV::calibrateZeroDepth() {
    // Store the RAW pressure reading at the current "zero" depth
    // Use more samples for calibration since it only happens once (when motors are stopped)
    zero_depth_pressure = pressure_sensor.readAveraged(3);  // 3 samples, acceptable for one-time calibration
    printf("Calibrated zero depth at pressure: %.2f\n", zero_depth_pressure);
}

void ROV::setDepthConversion(float offset, float factor) {
    pressure_offset = offset;
    pressure_to_depth_factor = factor;
    printf("Depth conversion updated: offset=%.2f, factor=%.4f\n", offset, factor);
}

bool ROV::isMoistureDetected() {
    // Read analog value from sensor (not just the last stored value)
    float moisture_level = moisture_sensor.readAnalog();
    moisture_detected = moisture_level > 1.0f;  // Threshold: >1% indicates moisture
    return moisture_detected;
}

bool ROV::isFullSwitchPressed() {
    return full_switch.read();
}

bool ROV::isEmptySwitchPressed() {
    return empty_switch.read();
}

void ROV::setThrustMotor1(float thrust_value) {
    // Check for moisture - block motor if water detected
    if (isMoistureDetected()) {
        thrust_motor1.stop();
        printf("THRUST1 BLOCKED - Moisture detected!\n");
        return;
    }
    
    // Clamp thrust value to -1.0 to 1.0 range
    if (thrust_value > 1.0f) thrust_value = 1.0f;
    if (thrust_value < -1.0f) thrust_value = -1.0f;
    
    // Convert to percentage (0-100%)
    float power_percent = fabsf(thrust_value) * 100.0f;
    
    // Apply motor control
    if (thrust_value > 0.0f) {
        thrust_motor1.moveFwd(power_percent);
    } else if (thrust_value < 0.0f) {
        thrust_motor1.moveBckwd(power_percent);
    } else {
        thrust_motor1.stop();
    }
}

void ROV::setThrustMotor2(float thrust_value) {
    // Check for moisture - block motor if water detected
    if (isMoistureDetected()) {
        thrust_motor2.stop();
        printf("THRUST2 BLOCKED - Moisture detected!\n");
        return;
    }
    
    // Clamp thrust value to -1.0 to 1.0 range
    if (thrust_value > 1.0f) thrust_value = 1.0f;
    if (thrust_value < -1.0f) thrust_value = -1.0f;
    
    // Convert to percentage (0-100%)
    float power_percent = fabsf(thrust_value) * 100.0f;
    
    // Apply motor control
    if (thrust_value > 0.0f) {
        thrust_motor2.moveFwd(power_percent);
    } else if (thrust_value < 0.0f) {
        thrust_motor2.moveBckwd(power_percent);
    } else {
        thrust_motor2.stop();
    }
}

void ROV::setThrust(float thrust_value) {
    // Set both thrust motors to the same value
    setThrustMotor1(thrust_value);
    setThrustMotor2(thrust_value);
}

void ROV::setBallast(float ballast_value) {
    // Clamp ballast value to -1.0 to 1.0 range
    if (ballast_value > 1.0f) ballast_value = 1.0f;
    if (ballast_value < -1.0f) ballast_value = -1.0f;
    
    // Apply EXPONENTIAL mapping: power = sign(value) * |value|^2 * 100
    // This provides fine control near zero and more power at extremes
    float sign = (ballast_value >= 0.0f) ? 1.0f : -1.0f;
    float power_percent = sign * (ballast_value * ballast_value) * 100.0f;
    
    // Check limit switches for safety
    bool full_pressed = isFullSwitchPressed();
    bool empty_pressed = isEmptySwitchPressed();
    
    // Forward direction (filling) - positive power
    if (power_percent > 0.0f) {
        if (full_pressed) {
            ballast_motor.stop();
            printf("BALLAST BLOCKED - Full switch pressed, cannot fill!\n");
            return;
        }
        ballast_motor.moveFwd(power_percent);
    }
    // Backward direction (emptying) - negative power
    else if (power_percent < 0.0f) {
        if (empty_pressed) {
            ballast_motor.stop();
            printf("BALLAST BLOCKED - Empty switch pressed, cannot empty!\n");
            return;
        }
        ballast_motor.moveBckwd(fabsf(power_percent));
    }
    // Zero power
    else {
        ballast_motor.stop();
    }
}

void ROV::stop() {
    // Emergency stop - stop all motors immediately
    thrust_motor1.stop();
    thrust_motor2.stop();
    ballast_motor.stop();
    
    printf("ROV EMERGENCY STOP - All motors stopped!\n");
}

void ROV::emergencyEmerge() {
    // Emergency emerge - full ballast in reverse (emptying)
    printf("ROV EMERGENCY EMERGE - Full ballast emptying!\n");
    thrust_motor1.stop();
    thrust_motor2.stop();
    
    // Set ballast motor to full reverse power (100%)
    // Bypass limit switch for emergency
    ballast_motor.moveBckwd(100.0f);
}