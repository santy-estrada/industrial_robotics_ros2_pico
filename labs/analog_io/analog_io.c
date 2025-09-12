#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"

// Define the LED pin and the potentiometer pin
#define LED_PIN 25
#define POT_PIN 27 

// map() function to map a value from one range to another
long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


int main()
{
    // Initialize the stdio library
    stdio_init_all();

    // Initialize the LED pin
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // Initialize the potentiometer pin
    adc_init();
    // Set the ADC input to the potentiometer pin
    adc_gpio_init(POT_PIN);
    // select the ADC input to read 
    adc_select_input(1);

    // Initialize the PWM
    gpio_set_function(LED_PIN, GPIO_FUNC_PWM);
    
    // Get the slice number for the LED pin
    uint slice_num = pwm_gpio_to_slice_num(LED_PIN);
    
    // Set the PWM wrap value, which is the maximum value the PWM counter will reach
    // The PWM counter will count from 0 to the wrap value
    pwm_set_wrap(slice_num, 255);

    // Set the PWM clock divider, which is the frequency of the PWM signal
    // The PWM frequency is calculated as follows:  
    // PWM frequency = clock frequency / (wrap value * clock divider)
    // PWM_CHAN_B is the PWM channel that controls the LED pin
    pwm_set_chan_level(slice_num, PWM_CHAN_B, 0);

    // Enable the PWM slice
    pwm_set_enabled(slice_num, true);

    printf("start pico adc and pwm\n");
    while (true) {
        // Read the value from the potentiometer pin
        uint16_t result = adc_read();
        // Map the value from the potentiometer pin to the PWM value
        long pwm_value = map(result, 0, 4095, 0, 255);

        // Set the PWM value to the LED pin
        printf("Raw: %d\t PWM: %d \n", result, pwm_value);

        // Set the PWM value to the LED pin
        pwm_set_chan_level(slice_num, PWM_CHAN_B, pwm_value);

        // Sleep for 100ms
        sleep_ms(100);
    }
    
    return 0;
}