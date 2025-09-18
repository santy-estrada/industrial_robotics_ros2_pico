#ifndef MOTOR_H
#define MOTOR_H

#include <hardware/gpio.h>
#include <hardware/pwm.h>
#include <pico/stdlib.h>

class Motor {
protected:
    const uint ENA_PIN;  // PWM pin for speed control
    const uint IN1_PIN;  // Motor direction pin 1
    const uint IN2_PIN;  // Motor direction pin 2
    
    uint pwmSlice;       // PWM slice for motor control
    float PWM_POWER;     // Current PWM power (0-100%)

public:
    // Constructor
    Motor(uint ena_pin, uint in1_pin, uint in2_pin);
    
    // Basic motor control methods
    void moveFwd(float power = 50.0f);
    void moveBckwd(float power = 50.0f);
    void stop();
    
    // Power control methods
    float getPwmPower() const;
    void setPwmPower(float power);

private:
    // Helper method to apply PWM power
    void applyPwmPower();
};

#endif // MOTOR_H