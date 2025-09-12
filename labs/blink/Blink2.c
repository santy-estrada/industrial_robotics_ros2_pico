#include <stdio.h>
#include "pico/stdlib.h"

// blink2.c
// Blink a LED

// define LED_PIN PICO_DEFAULT_LED_PIN
#define LED_PIN 25

int main() {
    // *stdio_init_all()* is a function that initializes the stdio library. 
    // It is required to use printf and other stdio functions.
    stdio_init_all();

    // gpio_init() is a function that initializes a GPIO pin.
    gpio_init(LED_PIN);
    // gpio_set_dir() is a function that sets the direction of a GPIO pin.
    gpio_set_dir(LED_PIN, GPIO_OUT);

    while (true) {
        // printf() is a function that prints a formatted string to stdout.
        printf("Hello, world!\n");
        // gpio_put() is a function that sets the value of a GPIO pin.
        gpio_put(LED_PIN, 1);
        printf("LED on\n");
        // sleep_ms() is a function that sleeps for a given number of milliseconds.
        sleep_ms(250);
        gpio_put(LED_PIN, 0);
        printf("LED off\n");
        sleep_ms(250);
    }
    return 0;
}
