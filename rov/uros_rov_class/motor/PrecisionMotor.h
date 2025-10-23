#ifndef PRECISION_MOTOR_H
#define PRECISION_MOTOR_H

#include "Motor.h"
#include <hardware/irq.h>
#include <math.h>
#include <map>

class PrecisionMotor : public Motor {
private:
    const uint ENCODER_A_PIN;  // Encoder A pin
    const uint ENCODER_B_PIN;  // Encoder B pin
    const int TICKS_PER_REV;   // Encoder resolution (ticks per revolution)
    const float GEAR_RATIO;    // Gear ratio

    // PID controller variables
    float Kp, Ti, Td;
    float dt;                  // Time step for PID calculations
    
    // PID coefficients (calculated from Kp, Ti, Td)
    float q0, q1, q2;

    // PID error history: e(k), e(k-1), e(k-2)
    float error[3];
    float setpoint_percentage; // Setpoint as percentage
    float control_output;      // Store the actual control output (with sign)
    static constexpr float MAX_RPM = 200.0f; // 100% corresponds to 200 RPM

    // Encoder tracking variables
    volatile int32_t encoder_ticks;
    int32_t last_ticks;
    float filtered_rpm;
    static constexpr float alpha = 0.15f;  // Low-pass filter coefficient

    // Static map to route interrupts to correct motor instance
    static std::map<uint, PrecisionMotor*> motor_map;
    static bool interrupt_initialized;

    // Static interrupt handler (called for ALL GPIO interrupts)
    static void encoder_irq_handler(uint gpio, uint32_t events);

    // Instance method to handle this motor's encoder interrupt
    void handle_encoder_interrupt(uint gpio, uint32_t events);

    // PID helper methods
    void calculate_pid_coefficients();
    float calculate_pid_control(float measured_rpm);
    float apply_low_pass_filter(float raw_rpm);
    void calculate_rpm(float* revs, float* rpm);

public:
    // Constructor
    PrecisionMotor(uint ena_pin, uint in1_pin, uint in2_pin,
                   uint enc_a_pin, uint enc_b_pin, 
                   int ticks_per_rev = 28, float gear_ratio = 150.0f,
                   float kp = 0.7458f, float ti = 0.5000f, float td = 2.5000e-05f,
                   float dt = 0.1f);

    // Precision motor control methods
    void set_motor(float desired_speed);
    void set_setpoint(float rpm);
    
    // Getters
    float get_setpoint() const;
    float get_control_output() const;
    float get_filtered_rpm() const;
    float get_error() const;
    int32_t get_encoder_ticks() const;
};

#endif // PRECISION_MOTOR_H