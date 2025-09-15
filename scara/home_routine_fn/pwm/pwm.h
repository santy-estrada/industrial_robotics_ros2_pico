#ifndef PWM_H
#define PWM_H

#include "pico/stdlib.h"

// Initialize PWM for a specific pin
void init_pwm(uint pin, int wrap, int clkdiv);

// Set motor velocity as percentage (0-100%)
void set_vel(uint pin, int percent);

// Stop PWM output on a specific pin
void stop_pwm(uint pin);

#endif // PWM_H
