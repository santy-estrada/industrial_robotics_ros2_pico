#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"


// Store wrap values for each slice (there are 8 slices on RP2040)
static uint16_t slice_wrap_values[8] = {0};

void init_pwm(uint pin, int wrap, int clkdiv) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_OUT);

    gpio_set_function(pin, GPIO_FUNC_PWM);
    // Find which slice is connected to the GPIO pin
    uint slice_num = pwm_gpio_to_slice_num(pin);

    // Store the wrap value for this slice
    slice_wrap_values[slice_num] = wrap;

    // Set the clock divisor (to slow down the PWM clock)
    pwm_set_clkdiv(slice_num, clkdiv);  // Set clock divisor to 1

   // Set the wrap value for a 10 kHz PWM signal
    pwm_set_wrap(slice_num, wrap);

    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(pin), 0);
    pwm_set_enabled(slice_num, true);
}

void set_vel(uint pin, int percent) {
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;

    uint slice_num = pwm_gpio_to_slice_num(pin);
    uint16_t wrap = slice_wrap_values[slice_num];
    uint16_t level = (percent * wrap) / 100;
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(pin), level);
    pwm_set_enabled(slice_num, true);
}

void stop_pwm(uint pin) {
    uint slice_num = pwm_gpio_to_slice_num(pin);
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(pin), 0);
    pwm_set_enabled(slice_num, false);
}
