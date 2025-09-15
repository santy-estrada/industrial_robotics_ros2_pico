#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"

// Test definitions - uncomment the test you want to run
// #define T1  // Test all pins as GPIO outputs
// #define T2  // Test FC pins (pull-up switches)
// #define T3  // Test pull-down inputs (channels and ADC pins)
// #define T4  // Test ADC2 as analog input
// #define T5  // Test channels as interruptions
// #define T6  // Test PWM pins as PWM outputs
// #define T7  // Test servo motor with PWM and external power source
// #define T8  // Test DC motors (2) with PWM and direction
// #define T9  // Test motors with encoders - Direction and Interruptions
// #define T10 // Test single encoder channel (CHB_M2) - count ticks only
#define T11 // Test ultrasonic sensor HC-SR05 (Trig -> ADC1, echo -> ADC0)

// Define the GPIO pins
#define LED_PIN 25

#define PWMA_2 0
#define INA2_2 1
#define INA1_2 2
#define PWMB_2 3
#define INB2_2 4
#define INB1_2 5
#define FC_1L 6
#define FC_1R 7
#define FC_2L 8
#define FC_2R 9
#define SDA1 10
#define SCL1 11
#define CHA_M1 12
#define CHB_M1 13
#define CHA_M2 14
#define CHB_M2 15
#define PWMA_1 16
#define INA2_1 17
#define INA1_1 18
#define INB1_1 19
#define INB2_1 20
#define PWMB_1 21
#define PWM_SERVO 22
#define ADC0 26
#define ADC1 27
#define ADC2 28

void init_gpios(){
    // Always initialize LED pin
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

#ifdef T1
    // Test 1: Initialize all pins as OUTPUT for GPIO testing
    gpio_init(PWMA_2);
    gpio_set_dir(PWMA_2, GPIO_OUT);
    gpio_init(INA2_2);
    gpio_set_dir(INA2_2, GPIO_OUT);
    gpio_init(INA1_2);
    gpio_set_dir(INA1_2, GPIO_OUT);
    gpio_init(PWMB_2);
    gpio_set_dir(PWMB_2, GPIO_OUT);
    gpio_init(INB2_2);
    gpio_set_dir(INB2_2, GPIO_OUT);
    gpio_init(INB1_2);
    gpio_set_dir(INB1_2, GPIO_OUT);
    gpio_init(FC_1L);
    gpio_set_dir(FC_1L, GPIO_OUT);
    gpio_init(FC_1R);
    gpio_set_dir(FC_1R, GPIO_OUT);
    gpio_init(FC_2L);
    gpio_set_dir(FC_2L, GPIO_OUT);
    gpio_init(FC_2R);
    gpio_set_dir(FC_2R, GPIO_OUT);
    gpio_init(SDA1);
    gpio_set_dir(SDA1, GPIO_OUT);
    gpio_init(SCL1);
    gpio_set_dir(SCL1, GPIO_OUT);
    gpio_init(CHA_M1);
    gpio_set_dir(CHA_M1, GPIO_OUT);
    gpio_init(CHB_M1);
    gpio_set_dir(CHB_M1, GPIO_OUT);
    gpio_init(CHA_M2);
    gpio_set_dir(CHA_M2, GPIO_OUT);
    gpio_init(CHB_M2);
    gpio_set_dir(CHB_M2, GPIO_OUT);
    gpio_init(PWMA_1);
    gpio_set_dir(PWMA_1, GPIO_OUT);
    gpio_init(INA2_1);
    gpio_set_dir(INA2_1, GPIO_OUT);
    gpio_init(INA1_1);
    gpio_set_dir(INA1_1, GPIO_OUT);
    gpio_init(INB1_1);
    gpio_set_dir(INB1_1, GPIO_OUT);
    gpio_init(INB2_1);
    gpio_set_dir(INB2_1, GPIO_OUT);
    gpio_init(PWMB_1);
    gpio_set_dir(PWMB_1, GPIO_OUT);
    gpio_init(PWM_SERVO);
    gpio_set_dir(PWM_SERVO, GPIO_OUT);
    gpio_init(ADC0);
    gpio_set_dir(ADC0, GPIO_OUT);
    gpio_init(ADC1);
    gpio_set_dir(ADC1, GPIO_OUT);
    gpio_init(ADC2);
    gpio_set_dir(ADC2, GPIO_OUT);
#endif

#ifdef T2
    // Test 2: Initialize FC pins as INPUT (for pull-up switch testing)
    gpio_init(FC_1L);
    gpio_set_dir(FC_1L, GPIO_IN);
    gpio_init(FC_1R);
    gpio_set_dir(FC_1R, GPIO_IN);
    gpio_init(FC_2L);
    gpio_set_dir(FC_2L, GPIO_IN);
    gpio_init(FC_2R);
    gpio_set_dir(FC_2R, GPIO_IN);
#endif

#ifdef T3
    // Test 3: Initialize channels and ADC pins as INPUT (for pull-down testing)
    gpio_init(CHA_M1);
    gpio_set_dir(CHA_M1, GPIO_IN);
    gpio_init(CHB_M1);
    gpio_set_dir(CHB_M1, GPIO_IN);
    gpio_init(CHA_M2);
    gpio_set_dir(CHA_M2, GPIO_IN);
    gpio_init(CHB_M2);
    gpio_set_dir(CHB_M2, GPIO_IN);
    gpio_init(ADC0);
    gpio_set_dir(ADC0, GPIO_IN);
    gpio_init(ADC1);
    gpio_set_dir(ADC1, GPIO_IN);
    gpio_init(ADC2);
    gpio_set_dir(ADC2, GPIO_IN);
#endif

#ifdef T4
    // Test 4: ADC2 will be initialized as analog input in init_analog()
    // No GPIO initialization for ADC2 here
#endif

#if defined T5 || defined T9
    //Test 5, 9: Channels as interruptions
    gpio_init(CHA_M1);
    gpio_set_dir(CHA_M1, GPIO_IN);
    gpio_pull_down(CHA_M1);
    gpio_init(CHB_M1);
    gpio_set_dir(CHB_M1, GPIO_IN);
    gpio_pull_down(CHB_M1);
    gpio_init(CHA_M2);
    gpio_set_dir(CHA_M2, GPIO_IN);
    gpio_pull_down(CHA_M2);
    gpio_init(CHB_M2);
    gpio_set_dir(CHB_M2, GPIO_IN);
    gpio_pull_down(CHB_M2);
#endif

#ifdef T10
    // Test 10: Single encoder channel (CHB_M2) for tick counting
    gpio_init(CHB_M2);
    gpio_set_dir(CHB_M2, GPIO_IN);
    gpio_pull_down(CHB_M2);
#endif

#ifdef T11
    // Test 11: HC-SR05 ultrasonic sensor (Trig -> ADC1, Echo -> ADC0)
    gpio_init(ADC1);  // Trigger pin
    gpio_set_dir(ADC1, GPIO_OUT);
    gpio_put(ADC1, 0);  // Initialize trigger low
    
    gpio_init(ADC0);  // Echo pin
    gpio_set_dir(ADC0, GPIO_IN);
#endif

#ifdef T6
    //Test 6: PWM pins as PWM outputs
    // No GPIOs are set. They will be handled by the init_pwm function
#endif

#if defined T8 || defined T9
    // Test 8: DC motors with PWM and direction control
    gpio_init(INA1_1);
    gpio_set_dir(INA1_1, GPIO_OUT);
    gpio_init(INB1_1);
    gpio_set_dir(INB1_1, GPIO_OUT);
    gpio_init(INA2_1);
    gpio_set_dir(INA2_1, GPIO_OUT);
    gpio_init(INB2_1);
    gpio_set_dir(INB2_1, GPIO_OUT);
    gpio_init(PWMA_1);
    gpio_set_dir(PWMA_1, GPIO_OUT);
    gpio_init(PWMB_1);
    gpio_set_dir(PWMB_1, GPIO_OUT);

    gpio_init(INA1_2);
    gpio_set_dir(INA1_2, GPIO_OUT);
    gpio_init(INB1_2);
    gpio_set_dir(INB1_2, GPIO_OUT);
    gpio_init(INA2_2);
    gpio_set_dir(INA2_2, GPIO_OUT);
    gpio_init(INB2_2);
    gpio_set_dir(INB2_2, GPIO_OUT);
    gpio_init(PWMA_2);
    gpio_set_dir(PWMA_2, GPIO_OUT);
    gpio_init(PWMB_2);
    gpio_set_dir(PWMB_2, GPIO_OUT);
#endif
}

#ifdef T7
void init_pwm_servo(uint pin) {
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
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(pin), 1953); 

    // Enable PWM output
    pwm_set_enabled(slice_num, true);
}
#endif

#if defined(T6) || defined(T8) || defined(T9)
void init_pwm(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    // Find which slice is connected to the GPIO pin
    uint slice_num = pwm_gpio_to_slice_num(pin);

    // Set the clock divisor (to slow down the PWM clock)
    pwm_set_clkdiv(slice_num, 1);  // Set clock divisor to 1

   // Set the wrap value for a 10 kHz PWM signal (calculated wrap = 12499)
    // PWM frequency = 125 MHz / wrap / 2
    // 12499 = 125000000 / 10000 / 2
    // 10 kHz PWM signal for a DC motor driver
    pwm_set_wrap(slice_num, 12499);
}
#endif

#ifdef T5
void interruption_test(uint gpio, uint32_t events)
{
    // Print the button press
    printf("Button pressed on gpio: %d, and the event: %d \n",gpio,events);
}

void init_interruption(){
    gpio_set_irq_enabled_with_callback(CHA_M1, GPIO_IRQ_EDGE_RISE, true, &interruption_test);
    gpio_set_irq_enabled_with_callback(CHB_M1, GPIO_IRQ_EDGE_RISE, true, &interruption_test);
    gpio_set_irq_enabled_with_callback(CHA_M2, GPIO_IRQ_EDGE_RISE, true, &interruption_test);
    gpio_set_irq_enabled_with_callback(CHB_M2, GPIO_IRQ_EDGE_RISE, true, &interruption_test);
}
#endif

#ifdef T9
volatile int32_t encoderM1_ticks = 0; // Store the number of encoder ticks
volatile int32_t encoderM2_ticks = 0; // Store the number of encoder ticks

// Function to get the current encoder tick count
int32_t get_encoderM1_ticks() {
    return encoderM1_ticks;
}

int32_t get_encoderM2_ticks() {
    return encoderM2_ticks;
}

void interruption(uint gpio, uint32_t events)
{
    if (gpio == CHA_M1 || gpio == CHB_M1){
        // Read both encoder A and B pins
        bool encoder_a = gpio_get(CHA_M1);
        bool encoder_b = gpio_get(CHB_M1);

        // Determine direction based on the quadrature signals
        // Count rising and falling edges of both A and B channels
        if (gpio == CHA_M1) {
            if ((encoder_a && !encoder_b) || (!encoder_a && encoder_b)) {
                encoderM1_ticks++;  // Forward direction
            } else {
                encoderM1_ticks--;  // Reverse direction
            }
        } else {
            if ((encoder_a && !encoder_b) || (!encoder_a && encoder_b)) {
                encoderM1_ticks--;  // Forward direction
            } else {
                encoderM1_ticks++;  // Reverse direction
            }
        }
    }else{
        // Read both encoder A and B pins
        bool encoder_a = gpio_get(CHA_M2);
        bool encoder_b = gpio_get(CHB_M2);

        // Determine direction based on the quadrature signals
        // Count rising and falling edges of both A and B channels
        if (gpio == CHA_M2) {
            if ((encoder_a && !encoder_b) || (!encoder_a && encoder_b)) {
                encoderM2_ticks++;  // Forward direction
            } else {
                encoderM2_ticks--;  // Reverse direction
            }
        } else {
            if ((encoder_a && !encoder_b) || (!encoder_a && encoder_b)) {
                encoderM2_ticks--;  // Forward direction
            } else {
                encoderM2_ticks++;  // Reverse direction
            }
        }
    }

}


void init_interruption(){
    gpio_set_irq_enabled_with_callback(CHA_M1, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &interruption);
    gpio_set_irq_enabled_with_callback(CHB_M1, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &interruption);
    gpio_set_irq_enabled_with_callback(CHA_M2, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &interruption);
    gpio_set_irq_enabled_with_callback(CHB_M2, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &interruption);
}
#endif

#ifdef T10
volatile uint32_t encoder_ticks = 0; // Store the number of encoder ticks

// Function to get the current encoder tick count
uint32_t get_encoder_ticks() {
    return encoder_ticks;
}

// Reset the encoder tick count
void reset_encoder_ticks() {
    encoder_ticks = 0;
}

void interruption_single_channel(uint gpio, uint32_t events)
{
    // Simple tick counting - no direction detection possible with single channel
    encoder_ticks++;
}

void init_interruption(){
    gpio_set_irq_enabled_with_callback(CHB_M2, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &interruption_single_channel);
}
#endif

#ifdef T11
// Function to measure distance using HC-SR05 ultrasonic sensor
long measure_distance() {
    // Send trigger pulse (HC-SR05 needs at least 10us trigger pulse)
    gpio_put(ADC1, 1);  // Set trigger high
    sleep_us(10);       // Wait 10 microseconds (optimal for HC-SR05)
    gpio_put(ADC1, 0);  // Set trigger low
    
    // Wait for echo to go high (start of pulse)
    // HC-SR05 is faster and more reliable than HC-SR04
    uint32_t timeout = time_us_32() + 20000; // 20ms timeout (reduced for HC-SR05)
    while (!gpio_get(ADC0)) {
        if (time_us_32() > timeout) {
            return -1; // Timeout - no echo received
        }
    }
    
    // Measure pulse duration
    uint32_t start_time = time_us_32();
    
    // Wait for echo to go low (end of pulse)
    // HC-SR05 has more stable echo signal
    timeout = start_time + 20000; // 20ms timeout for echo pulse (reduced for HC-SR05)
    while (gpio_get(ADC0)) {
        if (time_us_32() > timeout) {
            return -1; // Timeout - echo stuck high
        }
    }
    
    uint32_t end_time = time_us_32();
    
    // Calculate distance in cm
    // Speed of sound = 343 m/s = 0.0343 cm/us
    // Distance = (time * speed) / 2 (divide by 2 for round trip)
    // HC-SR05 uses same calculation: pulse_time_us / 58
    long pulse_time = end_time - start_time;
    long distance = pulse_time / 58;
    
    return distance;
}
#endif



long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#ifdef T4
void init_analog(){
    adc_init();
    adc_gpio_init(ADC2);
    adc_select_input(2);
}
#endif

#ifdef T2
void set_pull_up_FC(){
    gpio_pull_up(FC_1L);
    gpio_pull_up(FC_1R);
    gpio_pull_up(FC_2L);
    gpio_pull_up(FC_2R);
}
#endif

#ifdef T3
void set_pull_down_CH(){
    gpio_pull_down(CHA_M1);
    gpio_pull_down(CHB_M1);
    gpio_pull_down(CHA_M2);
    gpio_pull_down(CHB_M2);
    gpio_pull_down(ADC0);
    gpio_pull_down(ADC1);
    gpio_pull_down(ADC2);
}
#endif

int main()
{
    // Initialize stdio
    stdio_init_all();

    // Initialize the GPIO pins
    init_gpios();

    // Initialize specific test setups
#ifdef T2
    set_pull_up_FC();
#endif
#ifdef T3
    set_pull_down_CH();
#endif
#ifdef T4
    init_analog();
#endif
#ifdef T5
    init_interruption();
#endif

#ifdef T7
    init_pwm_servo(PWM_SERVO);
#endif

#if defined T8 || defined T9
    init_pwm(PWMA_1);
    init_pwm(PWMB_1);
    init_pwm(PWMA_2);
    init_pwm(PWMB_2);
    
    // Initialize all PWM slices but keep them disabled initially
    uint slice_num = pwm_gpio_to_slice_num(PWMA_1);
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(PWMA_1), 0);
    pwm_set_enabled(slice_num, false); // Keep slice 0 disabled initially

    slice_num = pwm_gpio_to_slice_num(PWMB_1);
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(PWMB_1), 0);
    pwm_set_enabled(slice_num, true);

    slice_num = pwm_gpio_to_slice_num(PWMB_2);
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(PWMB_2), 0);
    pwm_set_enabled(slice_num, true);
#endif

#ifdef T9
    init_interruption();
#endif

#ifdef T10
    init_interruption();
#endif

    gpio_put(LED_PIN, 1); // Turn on the onboard LED to indicate the program is running

    while (true) {

#ifdef T1
        // Test 1: All pins as GPIO outputs - cycle through all pins
        for (int i = 1; i < 29; i++) {
            gpio_put(i, 1);
            printf("GPIO pin %d is ON\n", i);
            sleep_ms(2000);
            gpio_put(i, 0);
            printf("GPIO pin %d is OFF\n", i);
            sleep_ms(1000);
        }
#endif

#ifdef T2
        // Test 2: FC pins (pull-up switches) - check if connected to ground
        if (!gpio_get(FC_1L)) {
            printf("FC_1L pressed\n");
        }
        if (!gpio_get(FC_1R)) {
            printf("FC_1R pressed\n");
        }
        if (!gpio_get(FC_2L)) {
            printf("FC_2L pressed\n");
        }
        if (!gpio_get(FC_2R)) {
            printf("FC_2R pressed\n");
        }
        sleep_ms(500);
#endif

#ifdef T3
        // Test 3: Pull-down inputs (channels and ADC pins) - check if connected to 3V3
        if (gpio_get(ADC0)) {
            printf("ADC0 pressed\n");
        }
        if (gpio_get(ADC1)) {
            printf("ADC1 pressed\n");
        }
        if (gpio_get(ADC2)) {
            printf("ADC2 pressed\n");
        }
        if(gpio_get(CHA_M1)){
            printf("CHA_M1 pressed\n");
        }
        if(gpio_get(CHB_M1)){
            printf("CHB_M1 pressed\n");
        }
        if(gpio_get(CHA_M2)){
            printf("CHA_M2 pressed\n");
        }
        if(gpio_get(CHB_M2)){
            printf("CHB_M2 pressed\n");
        }
        sleep_ms(500);
#endif

#ifdef T4
        // Test 4: ADC2 as analog input - read and map to 0-255
        uint16_t result = adc_read();
        long pwm_value = map(result, 0, 4095, 0, 255);
        printf("Raw: %d\t PWM: %d \n", result, pwm_value);
        sleep_ms(100);
#endif

#ifdef T5
        // Test 5: Channels as interruptions - handled in interruption_test()
        tight_loop_contents();
#endif

#ifdef T6
        // Test 6: PWM pins as PWM outputs - Cycle each pin through 100%, 75%, 25%, 0%
        uint pwm_pins[] = {PWMA_1, PWMB_1, PWM_SERVO, PWMA_2, PWMB_2};
        uint levels[] = {12499, 9374, 3124, 0}; // 100%, 75%, 25%, 0%
        const char *percentages[] = {"100%", "75%", "25%", "0%"};
        
        for (int i = 0; i < 5; i++) {
            // Reset all pins to GPIO function first to avoid conflicts
            for (int k = 0; k < 5; k++) {
                gpio_set_function(pwm_pins[k], GPIO_FUNC_SIO);
                gpio_set_dir(pwm_pins[k], GPIO_OUT);
                gpio_put(pwm_pins[k], 0);
            }
            
            // Now set only the current pin to PWM function
            init_pwm(pwm_pins[i]);
            uint slice_num = pwm_gpio_to_slice_num(pwm_pins[i]);
            uint channel = pwm_gpio_to_channel(pwm_pins[i]);
            
            // Enable PWM output for this slice
            pwm_set_enabled(slice_num, true);
            
            for (int j = 0; j < 4; j++) {
                pwm_set_chan_level(slice_num, channel, levels[j]);
                printf("Pin %d at %s\n", pwm_pins[i], percentages[j]);
                sleep_ms(10000);
            }
            
            // Disable PWM output
            pwm_set_enabled(slice_num, false);
        }
#endif

#ifdef T7
        // Test 7: Servo motor with PWM and external power source from 0 to 180 in 3 steps
        int angle = 0;
        // for (int i = 0; i <= 180; i += 60) {
        //     angle = map(i, 0, 180, 977, 4883);
        //     pwm_set_chan_level(pwm_gpio_to_slice_num(PWM_SERVO), pwm_gpio_to_channel(PWM_SERVO), angle);
        //     printf("Servo motor angle set to %d\n", i);
        //     sleep_ms(2000);
        // }
        if (scanf("%d", &angle) == 1) {
            // Map the angle value from 0-180 to 0-255
            angle = map(angle, 0, 180, 977, 4883);

            // Set the PWM duty cycle to change Servo Position
            uint slice_num = pwm_gpio_to_slice_num(PWM_SERVO);
            pwm_set_chan_level(slice_num, pwm_gpio_to_channel(PWM_SERVO), angle);

            printf("Servo Position set to: %d\n", angle);
            sleep_ms(1000);
        }
#endif

#ifdef T8
        // Test 8: DC motors with PWM - Handle GPIO 0 and 16 conflict (same PWM slice/channel)
        int levels[] = {0, 3124, 6250, 11249}; // 0%, 25%, 50%, 90%
        const char *percentages[] = {"0%", "25%", "50%", "90%"};
        
        // Test Motor 1 (using GPIO 16 - PWMA_1)
        printf("=== Testing Motor 1 (GPIO 16 - PWMA_1) ===\n");
        
        // Set GPIO 0 (PWMA_2) to regular GPIO output to prevent interference
        gpio_set_function(PWMA_2, GPIO_FUNC_SIO);
        gpio_set_dir(PWMA_2, GPIO_OUT);
        gpio_put(PWMA_2, 0);
        
        // Set GPIO 16 (PWMA_1) to PWM function
        gpio_set_function(PWMA_1, GPIO_FUNC_PWM);
        uint slice_num = pwm_gpio_to_slice_num(PWMA_1);
        pwm_set_enabled(slice_num, true);
        
        // Direction 1 for Motor 1
        printf("Motor 1 - Direction 1\n");
        gpio_put(INA1_1, 1);
        gpio_put(INA2_1, 0);
        gpio_put(INB1_1, 1);
        gpio_put(INB2_1, 0);

        for (int i = 0; i < 4; i++) {
            pwm_set_chan_level(slice_num, pwm_gpio_to_channel(PWMA_1), levels[i]);
            pwm_set_chan_level(pwm_gpio_to_slice_num(PWMB_1), pwm_gpio_to_channel(PWMB_1), levels[i]);
            printf("Motor 1 set to %s\n", percentages[i]);
            sleep_ms(3000);
        }
        
        // Stop Motor 1
        pwm_set_chan_level(slice_num, pwm_gpio_to_channel(PWMA_1), 0);
        pwm_set_chan_level(pwm_gpio_to_slice_num(PWMB_1), pwm_gpio_to_channel(PWMB_1), 0);
        sleep_ms(1000);
        
        // Direction 2 for Motor 1
        printf("Motor 1 - Direction 2\n");
        gpio_put(INA1_1, 0);
        gpio_put(INA2_1, 1);
        gpio_put(INB1_1, 0);
        gpio_put(INB2_1, 1);

        for (int i = 0; i < 4; i++) {
            pwm_set_chan_level(slice_num, pwm_gpio_to_channel(PWMA_1), levels[i]);
            pwm_set_chan_level(pwm_gpio_to_slice_num(PWMB_1), pwm_gpio_to_channel(PWMB_1), levels[i]);
            printf("Motor 1 set to %s\n", percentages[i]);
            sleep_ms(3000);
        }
        
        // Stop Motor 1 and disable its PWM
        pwm_set_chan_level(slice_num, pwm_gpio_to_channel(PWMA_1), 0);
        pwm_set_chan_level(pwm_gpio_to_slice_num(PWMB_1), pwm_gpio_to_channel(PWMB_1), 0);
        pwm_set_enabled(slice_num, false);
        gpio_set_function(PWMA_1, GPIO_FUNC_SIO);
        gpio_set_dir(PWMA_1, GPIO_OUT);
        gpio_put(PWMA_1, 0);
        sleep_ms(2000);
        
        // Test Motor 2 (using GPIO 0 - PWMA_2)
        printf("=== Testing Motor 2 (GPIO 0 - PWMA_2) ===\n");
        
        // Set GPIO 0 (PWMA_2) to PWM function
        gpio_set_function(PWMA_2, GPIO_FUNC_PWM);
        slice_num = pwm_gpio_to_slice_num(PWMA_2);
        pwm_set_enabled(slice_num, true);
        
        // Direction 1 for Motor 2
        printf("Motor 2 - Direction 1\n");
        gpio_put(INA1_2, 1);
        gpio_put(INA2_2, 0);
        gpio_put(INB1_2, 1);
        gpio_put(INB2_2, 0);

        for (int i = 0; i < 4; i++) {
            pwm_set_chan_level(slice_num, pwm_gpio_to_channel(PWMA_2), levels[i]);
            pwm_set_chan_level(pwm_gpio_to_slice_num(PWMB_2), pwm_gpio_to_channel(PWMB_2), levels[i]);
            printf("Motor 2 set to %s\n", percentages[i]);
            sleep_ms(3000);
        }
        
        // Stop Motor 2
        pwm_set_chan_level(slice_num, pwm_gpio_to_channel(PWMA_2), 0);
        pwm_set_chan_level(pwm_gpio_to_slice_num(PWMB_2), pwm_gpio_to_channel(PWMB_2), 0);
        sleep_ms(1000);
        
        // Direction 2 for Motor 2
        printf("Motor 2 - Direction 2\n");
        gpio_put(INA1_2, 0);
        gpio_put(INA2_2, 1);
        gpio_put(INB1_2, 0);
        gpio_put(INB2_2, 1);

        for (int i = 0; i < 4; i++) {
            pwm_set_chan_level(slice_num, pwm_gpio_to_channel(PWMA_2), levels[i]);
            pwm_set_chan_level(pwm_gpio_to_slice_num(PWMB_2), pwm_gpio_to_channel(PWMB_2), levels[i]);
            printf("Motor 2 set to %s\n", percentages[i]);
            sleep_ms(3000);
        }
        
        // Stop Motor 2 and disable its PWM
        pwm_set_chan_level(slice_num, pwm_gpio_to_channel(PWMA_2), 0);
        pwm_set_chan_level(pwm_gpio_to_slice_num(PWMB_2), pwm_gpio_to_channel(PWMB_2), 0);
        pwm_set_enabled(slice_num, false);
        gpio_set_function(PWMA_2, GPIO_FUNC_SIO);
        gpio_set_dir(PWMA_2, GPIO_OUT);
        gpio_put(PWMA_2, 0);
        sleep_ms(2000);

#endif

#ifdef T9
        // Test 9: DC motors with PWM, direction and encoders
        // int levels[] = {0, 3124, 6250, 11249}; // 0%, 25%, 50%, 90%
        int levels[] = {0, 625, 1250}; // 0%, 5%, 10%
        // const char *percentages[] = {"0%", "25%", "50%", "90%"};
        const char *percentages[] = {"0%", "5%", "10%"};
        // Set GPIO 16 (PWMA_1) to PWM function
        gpio_set_function(PWMA_1, GPIO_FUNC_PWM);
        uint slice_num = pwm_gpio_to_slice_num(PWMA_1);
        pwm_set_enabled(slice_num, true);
        
        // Direction 1 for Motor 1
        printf("Motor 1 - Direction 1\n");
        gpio_put(INA1_1, 1);
        gpio_put(INA2_1, 0);
        gpio_put(INB1_1, 1);
        gpio_put(INB2_1, 0);

        for (int i = 0; i < 3; i++) {       //i < 4
            pwm_set_chan_level(slice_num, pwm_gpio_to_channel(PWMA_1), levels[i]);
            pwm_set_chan_level(pwm_gpio_to_slice_num(PWMB_1), pwm_gpio_to_channel(PWMB_1), levels[i]);
            printf("Motors set to %s\n", percentages[i]);
            printf("Encoder ticks M1: %d\n", get_encoderM1_ticks());
            printf("Encoder ticks M2: %d\n", get_encoderM2_ticks());
            sleep_ms(1000);
        }
        
        // Stop Motor 1
        pwm_set_chan_level(slice_num, pwm_gpio_to_channel(PWMA_1), 0);
        pwm_set_chan_level(pwm_gpio_to_slice_num(PWMB_1), pwm_gpio_to_channel(PWMB_1), 0);
        printf("Encoder ticks M1: %d\n", get_encoderM1_ticks());
        printf("Encoder ticks M2: %d\n", get_encoderM2_ticks());
        sleep_ms(1000);
        
        // Direction 2 for Motor 1
        printf("Motor 1 - Direction 2\n");
        gpio_put(INA1_1, 0);
        gpio_put(INA2_1, 1);
        gpio_put(INB1_1, 0);
        gpio_put(INB2_1, 1);

        for (int i = 0; i < 3; i++) {   // i < 4
            pwm_set_chan_level(slice_num, pwm_gpio_to_channel(PWMA_1), levels[i]);
            pwm_set_chan_level(pwm_gpio_to_slice_num(PWMB_1), pwm_gpio_to_channel(PWMB_1), levels[i]);
            printf("Motors set to %s\n", percentages[i]);
            printf("Encoder ticks M1: %d\n", get_encoderM1_ticks());
            printf("Encoder ticks M2: %d\n", get_encoderM2_ticks());
            sleep_ms(1000);
        }
        
        // Stop Motor 1 and disable its PWM
        pwm_set_chan_level(slice_num, pwm_gpio_to_channel(PWMA_1), 0);
        pwm_set_chan_level(pwm_gpio_to_slice_num(PWMB_1), pwm_gpio_to_channel(PWMB_1), 0);
        pwm_set_enabled(slice_num, false);
        gpio_set_function(PWMA_1, GPIO_FUNC_SIO);
        gpio_set_dir(PWMA_1, GPIO_OUT);
        gpio_put(PWMA_1, 0);
        printf("Encoder ticks M1: %d\n", get_encoderM1_ticks());
        printf("Encoder ticks M2: %d\n", get_encoderM2_ticks());
        sleep_ms(2000);
#endif

#ifdef T10
        // Test 10: Single encoder channel (CHB_M2) - count ticks only
        // Display current tick count every 2 seconds
        printf("CHB_M2 Encoder Ticks: %d\n", get_encoder_ticks());
        printf("Connect a signal to CHB_M2 (GPIO %d) to see tick counting...\n", CHB_M2);
        printf("Note: Only tick counting, no direction detection possible with single channel.\n");
        
        sleep_ms(2000);
#endif

#ifdef T11
        // Test 11: HC-SR05 ultrasonic sensor (3.3V compatible)
        long distance = measure_distance();
        
        if (distance >= 0 && distance <= 400) { // HC-SR05 max range ~4m
            printf("Distancia: %ld cm\n", distance);
        } else if (distance > 400) {
            printf("Objeto demasiado lejos (>4m) o sin obstáculo\n");
        } else {
            printf("Error: Sensor no responde - Verificar conexiones\n");
            printf("HC-SR05: VCC->3V3, GND->GND, Trig->GPIO27, Echo->GPIO26\n");
        }
        
        sleep_ms(150);  // 150ms delay - optimal for HC-SR05 continuous measurements
#endif

    }
    return 0;
}