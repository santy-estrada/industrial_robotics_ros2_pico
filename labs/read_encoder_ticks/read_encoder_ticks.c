#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"

#define MOTOR_PIN 8   // GPIO pin connected to the DC motor
#define MOTOR_IN2 9  // GPIO pin connected to the DC motor CCW
#define MOTOR_IN1 11  // GPIO pin connected to the DC motor CW

#define LED_PIN 25    // GPIO pin connected to the onboard LED

// GPIO pins for encoder
const uint ENCODER_A_PIN = 18; // Encoder A signal
const uint ENCODER_B_PIN = 19; // Encoder B signal

volatile int32_t encoder_ticks = 0; // Store the number of encoder tick

// GPIO interrupt handler for Encoder A pin
void encoder_a_irq_handler(uint gpio, uint32_t events) {
    // Read both encoder A and B pins
    bool encoder_a = gpio_get(ENCODER_A_PIN);
    bool encoder_b = gpio_get(ENCODER_B_PIN);

    // Determine direction based on the quadrature signals
    // Count rising and falling edges of both A and B channels
    if (gpio == ENCODER_A_PIN) {
        if ((encoder_a && !encoder_b) || (!encoder_a && encoder_b)) {
            encoder_ticks++;  // Forward direction
        } else {
            encoder_ticks--;  // Reverse direction
        }
    } else {
        if ((encoder_a && !encoder_b) || (!encoder_a && encoder_b)) {
            encoder_ticks--;  // Forward direction
        } else {
            encoder_ticks++;  // Reverse direction
        }
    }

}

// Function to initialize the encoder
void init_encoder() {
    // Initialize the GPIO pins connected to the encoder
    gpio_init(ENCODER_A_PIN);
    gpio_set_dir(ENCODER_A_PIN, GPIO_IN);
    gpio_pull_up(ENCODER_A_PIN);
    // Initialize the GPIO pins connected to the encoder
    gpio_init(ENCODER_B_PIN);
    gpio_set_dir(ENCODER_B_PIN, GPIO_IN);
    gpio_pull_up(ENCODER_B_PIN);

    // Attach interrupt on encoder A pin (rising and falling edge)
    gpio_set_irq_enabled_with_callback(ENCODER_A_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_a_irq_handler);
    gpio_set_irq_enabled_with_callback(ENCODER_B_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_a_irq_handler);
}

// Function to get the current encoder tick count
int32_t get_encoder_ticks() {
    return encoder_ticks;
}

int main() {
    // Initialize the stdio for UART output (optional)
    stdio_init_all();

    // Initialize the onboard LED
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // Initialize the GPIO pins connected to the DC motor
    gpio_init(MOTOR_IN1);
    gpio_set_dir(MOTOR_IN1, GPIO_OUT);
    gpio_put(MOTOR_IN1, 1);

    gpio_init(MOTOR_IN2);
    gpio_set_dir(MOTOR_IN2, GPIO_OUT); 
    gpio_put(MOTOR_IN2, 0);
    
    // Set the GPIO pin function to PWM
    gpio_set_function(MOTOR_PIN, GPIO_FUNC_PWM);
    
    // Get the PWM slice number associated with the GPIO pin
    uint slice_num = pwm_gpio_to_slice_num(MOTOR_PIN);
    
    // Set the wrap value for a 10 kHz PWM signal (calculated wrap = 12499)
    // PWM frequency = 125 MHz / wrap / 2
    // 12499 = 125000000 / 10000 / 2
    // 10 kHz PWM signal for a DC motor driver (L298N)
    pwm_set_wrap(slice_num, 12499);
    
    // Start with a 0 % duty cycle
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(MOTOR_PIN), 0);
    
    // Enable PWM output on the slice
    pwm_set_enabled(slice_num, true);

    // Initialize the encoder
    init_encoder();

    // Blink the onboard LED to indicate the program is running
    gpio_put(LED_PIN, 1);
    
    while (1) {
        // Increase speed to full (100% duty cycle)
        pwm_set_chan_level(slice_num, pwm_gpio_to_channel(MOTOR_PIN), 12499);
        printf("Encoder ticks: %d\n", get_encoder_ticks());
        sleep_ms(2000);  // Run at full speed for 2 seconds
        
        // Decrease speed to half (50% duty cycle)
        pwm_set_chan_level(slice_num, pwm_gpio_to_channel(MOTOR_PIN), 6250);
        printf("Encoder ticks: %d\n", get_encoder_ticks());
        sleep_ms(2000);  // Run at half speed for 2 seconds
        
        // Stop the motor (0% duty cycle)
        pwm_set_chan_level(slice_num, pwm_gpio_to_channel(MOTOR_PIN), 0);
        printf("Encoder ticks: %d\n", get_encoder_ticks());
        gpio_put(MOTOR_IN2, 1);
        gpio_put(MOTOR_IN1, 0);
        sleep_ms(2000);  // Stop for 2 seconds

        // Increase speed to full (100% duty cycle)
        pwm_set_chan_level(slice_num, pwm_gpio_to_channel(MOTOR_PIN), 12499);
        printf("Encoder ticks: %d\n", get_encoder_ticks());
        sleep_ms(2000);  // Run at full speed for 2 seconds
        
        // Decrease speed to half (50% duty cycle)
        pwm_set_chan_level(slice_num, pwm_gpio_to_channel(MOTOR_PIN), 6250);
        printf("Encoder ticks: %d\n", get_encoder_ticks());
        sleep_ms(2000);  // Run at half speed for 2 seconds
        
        // Stop the motor (0% duty cycle)
        pwm_set_chan_level(slice_num, pwm_gpio_to_channel(MOTOR_PIN), 0);
        printf("Encoder ticks: %d\n", get_encoder_ticks());
        gpio_put(MOTOR_IN2, 0);
        gpio_put(MOTOR_IN1, 1);
        sleep_ms(2000);  // Stop for 2 seconds

    }

    return 0;
}