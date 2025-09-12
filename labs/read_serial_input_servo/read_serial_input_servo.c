#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"

// Define GPIO for the SERVO and UART baud rate
#define SERVO_PIN 12

// Function to initialize PWM on a given GPIO pin
void init_pwm(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    // Find which slice is connected to the GPIO pin
    uint slice_num = pwm_gpio_to_slice_num(pin);

    // Set the clock divisor (to slow down the PWM clock to match servo requirements)
    pwm_set_clkdiv(slice_num, 64.0);  // Set clock divisor to 64
    
    // Set the wrap value for a 50 Hz PWM signal
    // 1 / (50 Hz) = 0.02 s = 20 ms
    // 39060 = 20 ms / (1 / 125 MHz) / 65536
    // 39060 is the maximum value that can be set for the wrap register
    pwm_set_wrap(slice_num, 39060);
    
    // Set the initial position of the servo (0 degrees, corresponding to 1.0 ms pulse width)
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(SERVO_PIN), 1953); 

    // Enable PWM output
    pwm_set_enabled(slice_num, true);
}

// map() function to map a value from one range to another
int map(int x, int in_min, int in_max, int out_min, int out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


int main() {
    // Initialize UART and GPIO
    stdio_init_all();
    init_pwm(SERVO_PIN);

    printf("PWM Servo Position Control via UART\n");
    printf("Enter a value between 0 and 180 to set Servo Position.\n");

    while (1) {
        // Wait for input from the terminal
        int angle;
        if (scanf("%d", &angle) == 1) {
            // Map the angle value from 0-180 to 0-255
            angle = map(angle, 0, 180, 1953, 3906);
            // Ensure angle is within bounds
            // if (angle < 1953) angle = 1953;
            // if (angle > 3906*2) angle = 3906;

            // Set the PWM duty cycle to change Servo Position
            uint slice_num = pwm_gpio_to_slice_num(SERVO_PIN);
            pwm_set_chan_level(slice_num, pwm_gpio_to_channel(SERVO_PIN), angle);

            printf("Servo Position set to: %d\n", angle);
            sleep_ms(1000);
        }
        
    }

    return 0;
}