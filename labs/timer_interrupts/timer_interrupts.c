#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/irq.h"

// LED pin is GPIO 25
#define LED_PIN 25 
// #define LED_PIN 25
// Button pin is GPIO 14
#define BUTTON_PIN 14

// LED state, counters
int led_state = 0;
int count = 0;
int interrupt_count = 0;

// Callback function for the repeating timer
bool repeating_timer_callback(struct repeating_timer *t)
{
    // Led state is toggled
    led_state = 1 - led_state;

    // Increase the counter
    count++;
    // Set the LED state
    gpio_put(LED_PIN,led_state);

    // Print the counter
    printf("LED toggled: %d\n",count);

    return true;
}

// Callback function for the button press
void button_pressed(uint gpio, uint32_t events)
{
    // Increase the interrupt counter
    interrupt_count++;

    // Print the button press
    printf("Button pressed on gpio: %d, and the event: %d, and interruptions: %d \n",gpio,events, interrupt_count);
}

int main()
{
    stdio_init_all();

    // Set up our LED
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // Set up our button
    gpio_init(BUTTON_PIN);
    gpio_set_dir(BUTTON_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_PIN);

    struct repeating_timer timer;
    // add_repeating_timer_ms(time_ms, callback_fn, user_data, timer_pointer ) 
    // function is used to add a repeating timer to the system
    add_repeating_timer_ms(500,repeating_timer_callback,NULL,&timer);   
    
    // gpio_set_irq_enabled_with_callback(BUTTON_PIN,GPIO_IRQ_EDGE_RISE || GPIO_IRQ_EDGE_FALL, true, &button_pressed);

    gpio_set_irq_enabled_with_callback(BUTTON_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &button_pressed);


    while (true) {

        // This is a tight loop that does nothing, it is just to keep the program running
        tight_loop_contents();
    }
    return 0;
}