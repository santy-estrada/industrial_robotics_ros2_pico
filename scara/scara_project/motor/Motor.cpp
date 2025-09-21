#include "Motor.h"

Motor::Motor(uint ena_pin, uint in1_pin, uint in2_pin)
    : ENA_PIN(ena_pin), IN1_PIN(in1_pin), IN2_PIN(in2_pin), PWM_POWER(0.0f) {
    
    // Initialize direction control pins
    gpio_init(IN1_PIN);
    gpio_init(IN2_PIN);
    gpio_set_dir(IN1_PIN, GPIO_OUT);
    gpio_set_dir(IN2_PIN, GPIO_OUT);
    
    // Set up PWM for speed control
    gpio_set_function(ENA_PIN, GPIO_FUNC_PWM);
    pwmSlice = pwm_gpio_to_slice_num(ENA_PIN);
    pwm_set_wrap(pwmSlice, 12499);  // 10kHz PWM frequency
    pwm_set_chan_level(pwmSlice, PWM_CHAN_A, 0);
    pwm_set_enabled(pwmSlice, true);
    
    // Initialize motor in stopped state
    stop();
}

void Motor::moveFwd(float power) {
    setPwmPower(power);
    gpio_put(IN1_PIN, 1);
    gpio_put(IN2_PIN, 0);
    applyPwmPower();
}

void Motor::moveBckwd(float power) {
    setPwmPower(power);
    gpio_put(IN1_PIN, 0);
    gpio_put(IN2_PIN, 1);
    applyPwmPower();
}

void Motor::stop() {
    gpio_put(IN1_PIN, 0);
    gpio_put(IN2_PIN, 0);
    setPwmPower(0.0f);
    applyPwmPower();
}

float Motor::getPwmPower() const {
    return PWM_POWER;
}

void Motor::setPwmPower(float power) {
    // Clamp power to 0-100% range
    if (power < 0.0f) power = 0.0f;
    if (power > 100.0f) power = 100.0f;
    PWM_POWER = power;
}

void Motor::applyPwmPower() {
    uint16_t pwm_level = (uint16_t)(PWM_POWER * 12499.0f / 100.0f);
    pwm_set_gpio_level(ENA_PIN, pwm_level);
}