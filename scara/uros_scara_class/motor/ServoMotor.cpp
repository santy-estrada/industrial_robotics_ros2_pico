#include "ServoMotor.h"
#include <stdio.h>
#include <cmath>  // For fabs function

// Constructor
ServoMotor::ServoMotor(uint pwm_pin, float min_angle_deg, float max_angle_deg,
                       uint16_t min_pwm, uint16_t max_pwm)
    : PWM_PIN(pwm_pin), min_angle(min_angle_deg), max_angle(max_angle_deg),
      min_pwm_value(min_pwm), max_pwm_value(max_pwm), current_angle(0.0f) {
    
    // Initialize PWM pin
    gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);
    pwmSlice = pwm_gpio_to_slice_num(PWM_PIN);
    
    // Set PWM frequency to 50Hz (20ms period) for servo
    // System clock is 125MHz, so we need: 125MHz / (50Hz * TOP) = prescaler
    // For 50Hz with TOP = 39062: prescaler = 125000000 / (50 * 39062) ≈ 64
    // Actual servo uses PWM range 500-2900 for 0-110° (measured via manual testing)
    pwm_set_clkdiv(pwmSlice, 64.0f);
    pwm_set_wrap(pwmSlice, 39060);  // TOP value for 50Hz with prescaler 64
    
    // Enable PWM
    pwm_set_enabled(pwmSlice, true);
    
    // Initialize servo to middle position
    // float mid_angle = (min_angle + max_angle) / 2.0f;
    // setAngle(mid_angle);
    
    printf("ServoMotor created: Pin=%d, Range=[%.1f°, %.1f°], PWM=[%d, %d], Initial=%.1f°\n",
           PWM_PIN, min_angle, max_angle, min_pwm_value, max_pwm_value, current_angle);
}

// Helper function for mapping values (same as in mv_joints.c)
long ServoMotor::map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Validate and constrain angle within limits
float ServoMotor::constrainAngle(float angle) {
    if (angle < min_angle) {
        printf("WARNING: Angle %.2f° below minimum %.2f°, constraining\n", angle, min_angle);
        return min_angle;
    }
    if (angle > max_angle) {
        printf("WARNING: Angle %.2f° above maximum %.2f°, constraining\n", angle, max_angle);
        return max_angle;
    }
    return angle;
}

// Set servo to specific angle
bool ServoMotor::setAngle(float angle_degrees) {
    // Constrain angle to valid range
    float constrained_angle = constrainAngle(angle_degrees);
    bool was_constrained = (constrained_angle != angle_degrees);
    
    // Map angle to PWM value using the same logic as mv_joints.c
    // Note: mv_joints uses map(angle, 0, 180, 977, 4883)
    uint16_t pwm_value = (uint16_t)map((long)constrained_angle, 
                                       (long)min_angle, (long)max_angle,
                                       (long)min_pwm_value, (long)max_pwm_value);
    
    // Set PWM duty cycle
    pwm_set_chan_level(pwmSlice, pwm_gpio_to_channel(PWM_PIN), pwm_value);
    
    // Update current position
    current_angle = constrained_angle;
    
    printf("Servo moved to %.2f° (PWM: %d)\n", current_angle, pwm_value);
    
    return !was_constrained;  // Return true if angle was not constrained
}


// Stop servo (maintain current position)
void ServoMotor::stop() {
    // For servo, "stop" means maintain current position
    // Re-send the current PWM value to ensure position is held
    setAngle(current_angle);
    printf("Servo holding position at %.2f°\n", current_angle);
}

// Get current servo angle
float ServoMotor::getCurrentAngle() const {
    return current_angle;
}


// Configuration getters
float ServoMotor::getMinAngle() const {
    return min_angle;
}

float ServoMotor::getMaxAngle() const {
    return max_angle;
}

uint16_t ServoMotor::getMinPwmValue() const {
    return min_pwm_value;
}

uint16_t ServoMotor::getMaxPwmValue() const {
    return max_pwm_value;
}

// Configuration setters (for recalibration)
void ServoMotor::setAngleLimits(float min_deg, float max_deg) {
    if (min_deg >= max_deg) {
        printf("ERROR: Invalid angle limits. Min (%.2f) must be less than Max (%.2f)\n", min_deg, max_deg);
        return;
    }
    
    min_angle = min_deg;
    max_angle = max_deg;
    
    // Re-constrain current angle to new limits
    current_angle = constrainAngle(current_angle);
    setAngle(current_angle);  // Update PWM to respect new limits
    
    printf("Servo angle limits updated: [%.2f°, %.2f°]\n", min_angle, max_angle);
}

void ServoMotor::setPwmLimits(uint16_t min_pwm, uint16_t max_pwm) {
    if (min_pwm >= max_pwm) {
        printf("ERROR: Invalid PWM limits. Min (%d) must be less than Max (%d)\n", min_pwm, max_pwm);
        return;
    }
    
    min_pwm_value = min_pwm;
    max_pwm_value = max_pwm;
    
    // Recalculate PWM for current angle with new limits
    setAngle(current_angle);
    
    printf("Servo PWM limits updated: [%d, %d]\n", min_pwm_value, max_pwm_value);
}