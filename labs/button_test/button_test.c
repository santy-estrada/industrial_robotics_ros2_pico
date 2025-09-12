#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

// Define the GPIO pin that the button is connected to
#define BUTTON_PIN 14
// Define a counter as global variable
int32_t counter = 0;

int main()
{
    // Initialize stdio
    stdio_init_all();

    // Initialize the GPIO pin
    gpio_init(BUTTON_PIN);
    // Set the GPIO pin to input
    gpio_set_dir(BUTTON_PIN, GPIO_IN);
    // Enable the pull-up resistor
    gpio_pull_up(BUTTON_PIN);

    while (true) {
        // Check if the button is pressed
        if (gpio_get(BUTTON_PIN) == 0) {
            // Increment the counter and print the value
            counter++;
            // Print the counter value
            printf("Button pressed!: %d\n", counter);
        }
        // Sleep for 250ms
        sleep_ms(250);
    }
    // Return 0 to indicate success
    return 0;
}