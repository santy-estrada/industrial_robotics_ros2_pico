#ifndef SERVO_MOTOR_H
#define SERVO_MOTOR_H

#include <hardware/gpio.h>
#include <hardware/pwm.h>
#include <pico/stdlib.h>

class ServoMotor {
private:
    const uint PWM_PIN;         // PWM pin for servo control
    uint pwmSlice;              // PWM slice for servo control
    
    // Servo configuration
    float min_angle;            // Minimum angle in degrees
    float max_angle;            // Maximum angle in degrees
    uint16_t min_pwm_value;     // PWM value for minimum angle
    uint16_t max_pwm_value;     // PWM value for maximum angle
    
    // Current state
    float current_angle;        // Current servo position in degrees
    
    // Helper function for mapping values
    long map(long x, long in_min, long in_max, long out_min, long out_max);
    
    // Validate and constrain angle within limits
    float constrainAngle(float angle);

public:
    // Constructor
    ServoMotor(uint pwm_pin, float min_angle_deg = 0.0f, float max_angle_deg = 180.0f,
               uint16_t min_pwm = 977, uint16_t max_pwm = 4883);
    
    // Servo control methods
    bool setAngle(float angle_degrees);          // Set servo to specific angle
    void stop();                                 // Stop servo (maintain current position)
    
    // Position methods
    float getCurrentAngle() const;               // Get current servo angle
    
    // Configuration getters
    float getMinAngle() const;
    float getMaxAngle() const;
    uint16_t getMinPwmValue() const;
    uint16_t getMaxPwmValue() const;
    
    // Configuration setters (for recalibration)
    void setAngleLimits(float min_deg, float max_deg);
    void setPwmLimits(uint16_t min_pwm, uint16_t max_pwm);
};

#endif // SERVO_MOTOR_H