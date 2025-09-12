#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"

// Test definitions - uncomment the test you want to run
// #define T1  // Home routine: Touch the 4 FC
#define M1
#define M2
#define T2  // Home routine: Stop in the middle

// Define the GPIO pins
#define LED_PIN 25

#define FC_1L 6
#define FC_1R 7
#define FC_2L 8
#define FC_2R 9
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
#define BTN 


volatile int32_t encoderM1_ticks = 0; // Store the number of encoder ticks
volatile int32_t encoderM2_ticks = 0; // Store the number of encoder ticks
int32_t rangeM1 =0;
int32_t rangeM2 =0;
int32_t midPoint1 =0;
int32_t midPoint2 =0;

bool stop = false; // Flag to indicate if the motors should stop

// Function to get the current encoder tick count
int32_t get_encoderM1_ticks() {
    return encoderM1_ticks;
}

int32_t get_encoderM2_ticks() {
    return encoderM2_ticks;
}

void set_pull_up_FC(){
    gpio_pull_up(FC_1L);
    gpio_pull_up(FC_1R);
    gpio_pull_up(FC_2L);
    gpio_pull_up(FC_2R);
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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

void init_limit_sw(){
    gpio_init(FC_1L);
    gpio_set_dir(FC_1L, GPIO_IN);
    gpio_init(FC_1R);
    gpio_set_dir(FC_1R, GPIO_IN);
    gpio_init(FC_2L);
    gpio_set_dir(FC_2L, GPIO_IN);
    gpio_init(FC_2R);
    gpio_set_dir(FC_2R, GPIO_IN);
    gpio_pull_up(FC_1L);
    gpio_pull_up(FC_1R);
    gpio_pull_up(FC_2L);
    gpio_pull_up(FC_2R);
}

void init_channels(){
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
}

void init_interruption(){
    gpio_set_irq_enabled_with_callback(CHA_M1, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &interruption);
    gpio_set_irq_enabled_with_callback(CHB_M1, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &interruption);
    gpio_set_irq_enabled_with_callback(CHA_M2, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &interruption);
    gpio_set_irq_enabled_with_callback(CHB_M2, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &interruption);
}

void init_motors(){
    gpio_init(INA1_1);
    gpio_set_dir(INA1_1, GPIO_OUT);
    gpio_init(INB1_1);
    gpio_set_dir(INB1_1, GPIO_OUT);
    gpio_init(INA2_1);
    gpio_set_dir(INA2_1, GPIO_OUT);
    gpio_init(INB2_1);
    gpio_set_dir(INB2_1, GPIO_OUT);
    // gpio_init(PWMA_1);
    // gpio_set_dir(PWMA_1, GPIO_OUT);
    // gpio_init(PWMB_1);
    // gpio_set_dir(PWMB_1, GPIO_OUT);
}

void init_gpios(){
    // Always initialize LED pin
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

#if defined T1 || defined T2
    // Test 1: Home routine with limit switches
    init_limit_sw();

    // Channels as interruptions
    init_channels();
    init_interruption();

    // Motors
    init_motors();
#endif
}

// void init_pwm(uint pin) {
//     gpio_set_function(pin, GPIO_FUNC_PWM);
//     // Find which slice is connected to the GPIO pin
//     uint slice_num = pwm_gpio_to_slice_num(pin);

//     // Set the clock divisor (to slow down the PWM clock)
//     pwm_set_clkdiv(slice_num, 1);  // Set clock divisor to 1

//    // Set the wrap value for a 10 kHz PWM signal (calculated wrap = 12499)
//     // PWM frequency = 125 MHz / wrap / 2
//     // 12499 = 125000000 / 10000 / 2
//     // 10 kHz PWM signal for a DC motor driver
//     pwm_set_wrap(slice_num, 12499);
// }


int main()
{
    // Initialize stdio
    stdio_init_all();

    // Initialize the GPIO pins
    init_gpios();

    // Initialize specific test setups
#if defined T1 || defined T2
    // init_pwm(PWMA_1);
    // init_pwm(PWMB_1);
    
    // // Initialize PWM slices
    // uint slice_num = pwm_gpio_to_slice_num(PWMA_1);
    // pwm_set_chan_level(slice_num, pwm_gpio_to_channel(PWMA_1), 0);
    // pwm_set_enabled(slice_num, true);

    // slice_num = pwm_gpio_to_slice_num(PWMB_1);
    // pwm_set_chan_level(slice_num, pwm_gpio_to_channel(PWMB_1), 0);
    // pwm_set_enabled(slice_num, true);

    // Home Routine: Move motors to touch all limit switches
    // 5% PWM = 0.05 * 12499 = ~625
    uint16_t pwm_percent = (uint16_t)(0.08 * 12499);
#endif

    gpio_put(LED_PIN, 1); // Turn on the onboard LED to indicate the program is running

    printf("Starting Home Routine...\n");

    while (true) {
#ifdef T1
        // Set PWM to 5% - ONLY Motor 1 (PWMA_1)
        uint slice_num_A = pwm_gpio_to_slice_num(PWMA_1);
        pwm_set_chan_level(slice_num_A, pwm_gpio_to_channel(PWMA_1), pwm_percent);
        // Keep Motor 2 (PWMB_1) stopped during Motor 1 homing
        uint slice_num_B = pwm_gpio_to_slice_num(PWMB_1);
    #ifdef M1
        printf("=== Starting Motor 1 Homing ===\n");
        
        // Reset encoder count for Motor 1
        encoderM1_ticks = 0;
        
        // Motor 1 - Direction 1 (towards FC_1L)
        printf("Motor 1: Moving towards FC_1L...\n");
        gpio_put(INA1_1, 0);  // Motor 1 Direction 1
        gpio_put(INA2_1, 1);
        // Keep Motor 2 direction pins inactive
        gpio_put(INB1_1, 0);
        gpio_put(INB2_1, 0);
        
        pwm_set_chan_level(slice_num_B, pwm_gpio_to_channel(PWMB_1), 0);
        
        // Move until FC_1L is touched
        while (gpio_get(FC_1L)) {  // FC switches are pulled up, so they read 1 when not pressed
            printf("Motor 1 ticks: %d\n", get_encoderM1_ticks());
            sleep_ms(100);
        }
        encoderM1_ticks = 0;
        rangeM1 = 0;
        printf("Range M1: %d\n", rangeM1);
        // Stop Motor 1 - FC_1L touched
        pwm_set_chan_level(slice_num_A, pwm_gpio_to_channel(PWMA_1), 0);
        // Keep Motor 2 stopped
        pwm_set_chan_level(slice_num_B, pwm_gpio_to_channel(PWMB_1), 0);
        printf("FC_1L ACTIVATED Motor 1 stopped. Ticks: %d\n", get_encoderM1_ticks());
        sleep_ms(2000);
        
        // Motor 1 - Direction 2 (towards FC_1R)
        printf("Motor 1: Moving towards FC_1R...\n");
        gpio_put(INA1_1, 1);  // Motor 1 Direction 2
        gpio_put(INA2_1, 0);
        // Keep Motor 2 direction pins inactive
        gpio_put(INB1_1, 0);
        gpio_put(INB2_1, 0);
        
        // Set PWM to 5% - ONLY Motor 1 (PWMA_1) 
        pwm_set_chan_level(slice_num_A, pwm_gpio_to_channel(PWMA_1), pwm_percent);
        // Keep Motor 2 stopped
        pwm_set_chan_level(slice_num_B, pwm_gpio_to_channel(PWMB_1), 0);
        
        // Move until FC_1R is touched
        while (gpio_get(FC_1R)) {  // FC switches are pulled up, so they read 1 when not pressed
            printf("Motor 1 ticks: %d\n", get_encoderM1_ticks());
            sleep_ms(100);
        }

        rangeM1 = get_encoderM1_ticks();
        printf("Range M1: %d\n", rangeM1);
        // Stop Motor 1 - FC_1R touched
        pwm_set_chan_level(slice_num_A, pwm_gpio_to_channel(PWMA_1), 0);
        // Keep Motor 2 stopped  
        pwm_set_chan_level(slice_num_B, pwm_gpio_to_channel(PWMB_1), 0);
        printf("*** FC_1R ACTIVATED *** Motor 1 stopped. Final ticks: %d\n", get_encoderM1_ticks());
        printf("=== Motor 1 Homing Complete ===\n\n");
        sleep_ms(2000);
    #endif
    #ifdef M2
        // Now start Motor 2 homing
        printf("=== Starting Motor 2 Homing ===\n");
        
        // Reset encoder count for Motor 2
        encoderM2_ticks = 0;
        
        // Motor 2 - Direction 1 (towards FC_2L)
        printf("Motor 2: Moving towards FC_2L...\n");
        // Keep Motor 1 direction pins inactive
        gpio_put(INA1_1, 0);
        gpio_put(INA2_1, 0);
        // Motor 2 Direction 1
        gpio_put(INB1_1, 0);  
        gpio_put(INB2_1, 1);
        
        // Set PWM to 5% - ONLY Motor 2 (PWMB_1), keep Motor 1 stopped
        pwm_set_chan_level(slice_num_A, pwm_gpio_to_channel(PWMA_1), 0);  // Motor 1 OFF
        pwm_set_chan_level(slice_num_B, pwm_gpio_to_channel(PWMB_1), pwm_percent);  // Motor 2 ON
        
        // Move until FC_2L is touched
        while (gpio_get(FC_2L)) {  // FC switches are pulled up, so they read 1 when not pressed
            printf("Motor 2 ticks: %d\n", get_encoderM2_ticks());
            sleep_ms(100);
        }
        encoderM2_ticks = 0;
        rangeM2 = 0;
        printf("Range M2: %d\n", rangeM2);
        // Stop Motor 2 - FC_2L touched
        pwm_set_chan_level(slice_num_A, pwm_gpio_to_channel(PWMA_1), 0);  // Motor 1 OFF
        pwm_set_chan_level(slice_num_B, pwm_gpio_to_channel(PWMB_1), 0);  // Motor 2 OFF
        printf("*** FC_2L ACTIVATED *** Motor 2 stopped. Ticks: %d\n", get_encoderM2_ticks());
        sleep_ms(2000);
        
        // Motor 2 - Direction 2 (towards FC_2R)
        printf("Motor 2: Moving towards FC_2R...\n");
        // Keep Motor 1 direction pins inactive
        gpio_put(INA1_1, 0);
        gpio_put(INA2_1, 0);
        // Motor 2 Direction 2
        gpio_put(INB1_1, 1);
        gpio_put(INB2_1, 0);
        
        // Set PWM to 5% - ONLY Motor 2 (PWMB_1), keep Motor 1 stopped
        pwm_set_chan_level(slice_num_A, pwm_gpio_to_channel(PWMA_1), 0);  // Motor 1 OFF
        pwm_set_chan_level(slice_num_B, pwm_gpio_to_channel(PWMB_1), pwm_percent);  // Motor 2 ON
        
        // Move until FC_2R is touched
        while (gpio_get(FC_2R)) {  // FC switches are pulled up, so they read 1 when not pressed
            printf("Motor 2 ticks: %d\n", get_encoderM2_ticks());
            sleep_ms(100);
        }

        rangeM2 = get_encoderM2_ticks();
        printf("Range M2: %d\n", rangeM2);
        
        // Stop Motor 2 - FC_2R touched
        pwm_set_chan_level(slice_num_A, pwm_gpio_to_channel(PWMA_1), 0);  // Motor 1 OFF
        pwm_set_chan_level(slice_num_B, pwm_gpio_to_channel(PWMB_1), 0);  // Motor 2 OFF
        printf("*** FC_2R ACTIVATED *** Motor 2 stopped. Final ticks: %d\n", get_encoderM2_ticks());
        printf("=== Motor 2 Homing Complete ===\n\n");
    #endif
        printf("*** HOME ROUTINE COMPLETE ***\n");
        printf("Motor 1 final position: %d ticks\n", get_encoderM1_ticks());
        printf("Motor 2 final position: %d ticks\n", get_encoderM2_ticks());
        printf("Waiting 10 seconds before restarting...\n\n");
        sleep_ms(10000);
#endif

#ifdef T2
        // Set PWM to 5%
        uint slice_num_A = pwm_gpio_to_slice_num(PWMA_1);
        uint slice_num_B = pwm_gpio_to_slice_num(PWMB_1);

    #ifdef M2
        // Now start Motor 2 homing
        printf("=== Starting Motor 2 Homing ===\n");
        
        // Reset encoder count for Motor 2
        encoderM2_ticks = 0;
        
        // Motor 2 - Direction 1 (towards FC_2L)
        printf("Motor 2: Moving towards FC_2L...\n");
        // Keep Motor 1 direction pins inactive
        gpio_put(INA1_1, 0);
        gpio_put(INA2_1, 0);
        // Motor 2 Direction 1
        gpio_put(INB1_1, 0);  
        gpio_put(INB2_1, 1);
        
        // Set PWM to 5% - ONLY Motor 2 (PWMB_1), keep Motor 1 stopped
        pwm_set_chan_level(slice_num_A, pwm_gpio_to_channel(PWMA_1), 0);  // Motor 1 OFF
        pwm_set_chan_level(slice_num_B, pwm_gpio_to_channel(PWMB_1), pwm_percent);  // Motor 2 ON
        
        // Move until FC_2L is touched
        while (gpio_get(FC_2L)) {  // FC switches are pulled up, so they read 1 when not pressed
            printf("Motor 2 ticks: %d\n", get_encoderM2_ticks());
            sleep_ms(100);
        }
        encoderM2_ticks = 0;
        rangeM2 = 0;
        printf("Range M2: %d\n", rangeM2);
        // Stop Motor 2 - FC_2L touched
        pwm_set_chan_level(slice_num_A, pwm_gpio_to_channel(PWMA_1), 0);  // Motor 1 OFF
        pwm_set_chan_level(slice_num_B, pwm_gpio_to_channel(PWMB_1), 0);  // Motor 2 OFF
        printf("*** FC_2L ACTIVATED *** Motor 2 stopped. Ticks: %d\n", get_encoderM2_ticks());
        sleep_ms(2000);
        
        // Motor 2 - Direction 2 (towards FC_2R)
        printf("Motor 2: Moving towards FC_2R...\n");
        // Keep Motor 1 direction pins inactive
        gpio_put(INA1_1, 0);
        gpio_put(INA2_1, 0);
        // Motor 2 Direction 2
        gpio_put(INB1_1, 1);
        gpio_put(INB2_1, 0);
        
        // Set PWM to 5% - ONLY Motor 2 (PWMB_1), keep Motor 1 stopped
        pwm_set_chan_level(slice_num_A, pwm_gpio_to_channel(PWMA_1), 0);  // Motor 1 OFF
        pwm_set_chan_level(slice_num_B, pwm_gpio_to_channel(PWMB_1), pwm_percent);  // Motor 2 ON
        
        // Move until FC_2R is touched
        while (gpio_get(FC_2R)) {  // FC switches are pulled up, so they read 1 when not pressed
            printf("Motor 2 ticks: %d\n", get_encoderM2_ticks());
            sleep_ms(100);
        }

        rangeM2 = get_encoderM2_ticks();
        midPoint2 = rangeM2 / 2;
        printf("Range M2: %d\n", rangeM2);
        
        // Stop Motor 2 - FC_2R touched
        pwm_set_chan_level(slice_num_A, pwm_gpio_to_channel(PWMA_1), 0);  // Motor 1 OFF
        pwm_set_chan_level(slice_num_B, pwm_gpio_to_channel(PWMB_1), 0);  // Motor 2 OFF
        printf("*** FC_2R ACTIVATED *** Motor 2 stopped. Final ticks: %d\n", get_encoderM2_ticks());
        sleep_ms(2000);

        // Move the arm towards the center
        encoderM2_ticks = 0;  // Reset encoder count
        printf("Motor 2: Moving towards center...\n");
        gpio_put(INA1_1, 0);
        gpio_put(INA2_1, 0);
        gpio_put(INB1_1, 0);
        gpio_put(INB2_1, 1);
        pwm_set_chan_level(slice_num_A, pwm_gpio_to_channel(PWMA_1), 0);
        pwm_set_chan_level(slice_num_B, pwm_gpio_to_channel(PWMB_1), pwm_percent);

        int32_t cont = 0;
        while ((cont < labs(midPoint2) - 100) && gpio_get(FC_2L)) {
            printf("Motor 2 ticks: %d\n", get_encoderM2_ticks());
            sleep_ms(100);
            cont = labs(get_encoderM2_ticks());
        }
        // Stop Motor 2 - FC_2R touched
        pwm_set_chan_level(slice_num_A, pwm_gpio_to_channel(PWMA_1), 0);  // Motor 1 OFF
        pwm_set_chan_level(slice_num_B, pwm_gpio_to_channel(PWMB_1), 0);  // Motor 2 OFF
        sleep_ms(2000);

        printf("=== Motor 2 Homing Complete ===\n\n");
    #endif

    #ifdef M1
        printf("=== Starting Motor 1 Homing ===\n");
        
        // Reset encoder count for Motor 1
        encoderM1_ticks = 0;
        
        // Motor 1 - Direction 1 (towards FC_1L)
        printf("Motor 1: Moving towards FC_1L...\n");
        gpio_put(INA1_1, 0);  // Motor 1 Direction 1
        gpio_put(INA2_1, 1);
        // Keep Motor 2 direction pins inactive
        gpio_put(INB1_1, 0);
        gpio_put(INB2_1, 0);
        
        pwm_set_chan_level(slice_num_B, pwm_gpio_to_channel(PWMB_1), 0);
        pwm_set_chan_level(slice_num_A, pwm_gpio_to_channel(PWMA_1), pwm_percent);

        // Move until FC_1L is touched
        while (gpio_get(FC_1L)) {  // FC switches are pulled up, so they read 1 when not pressed
            printf("Motor 1 ticks: %d\n", get_encoderM1_ticks());
            sleep_ms(100);
        }
        encoderM1_ticks = 0;
        rangeM1 = 0;
        printf("Range M1: %d\n", rangeM1);
        // Stop Motor 1 - FC_1L touched
        pwm_set_chan_level(slice_num_A, pwm_gpio_to_channel(PWMA_1), 0);
        // Keep Motor 2 stopped
        pwm_set_chan_level(slice_num_B, pwm_gpio_to_channel(PWMB_1), 0);
        printf("FC_1L ACTIVATED Motor 1 stopped. Ticks: %d\n", get_encoderM1_ticks());
        sleep_ms(2000);
        
        // Motor 1 - Direction 2 (towards FC_1R)
        printf("Motor 1: Moving towards FC_1R...\n");
        gpio_put(INA1_1, 1);  // Motor 1 Direction 2
        gpio_put(INA2_1, 0);
        // Keep Motor 2 direction pins inactive
        gpio_put(INB1_1, 0);
        gpio_put(INB2_1, 0);
        
        // Set PWM to 5% - ONLY Motor 1 (PWMA_1) 
        pwm_set_chan_level(slice_num_A, pwm_gpio_to_channel(PWMA_1), pwm_percent);
        // Keep Motor 2 stopped
        pwm_set_chan_level(slice_num_B, pwm_gpio_to_channel(PWMB_1), 0);
        
        // Move until FC_1R is touched
        while (gpio_get(FC_1R)) {  // FC switches are pulled up, so they read 1 when not pressed
            printf("Motor 1 ticks: %d\n", get_encoderM1_ticks());
            sleep_ms(100);
        }

        rangeM1 = get_encoderM1_ticks();
        midPoint1 = rangeM1 / 2;

        printf("Range M1: %d\n", rangeM1);


        // Stop Motor 1 - FC_1R touched
        pwm_set_chan_level(slice_num_A, pwm_gpio_to_channel(PWMA_1), 0);  // Motor 1 OFF
        pwm_set_chan_level(slice_num_B, pwm_gpio_to_channel(PWMB_1), 0);  // Motor 2 OFF
        printf("*** FC_1R ACTIVATED *** Motor 1 stopped. Final ticks: %d\n", get_encoderM1_ticks());
        sleep_ms(2000);

        // Move the arm towards the center
        encoderM1_ticks = 0;  // Reset encoder count
        printf("Motor 1: Moving towards center...\n");
        gpio_put(INA1_1, 0);
        gpio_put(INA2_1, 1);
        gpio_put(INB1_1, 0);
        gpio_put(INB2_1, 0);
        pwm_set_chan_level(slice_num_A, pwm_gpio_to_channel(PWMA_1), pwm_percent);
        pwm_set_chan_level(slice_num_B, pwm_gpio_to_channel(PWMB_1), 0);

        #ifndef M2
        int32_t cont = 0;
        #else
        cont = 0;
        #endif
        while ((cont < labs(midPoint1)) && gpio_get(FC_1L)) {
            printf("Motor 1 ticks: %d\n", get_encoderM1_ticks());
            sleep_ms(100);
            cont = labs(get_encoderM1_ticks());
        }
        // Stop Motor 1 - FC_1R touched
        pwm_set_chan_level(slice_num_A, pwm_gpio_to_channel(PWMA_1), 0);  // Motor 1 OFF
        pwm_set_chan_level(slice_num_B, pwm_gpio_to_channel(PWMB_1), 0);  // Motor 2 OFF

        printf("=== Motor 1 Homing Complete ===\n\n");
        sleep_ms(2000);
    #endif

        printf("*** HOME ROUTINE COMPLETE ***\n");
        printf("Motor 1 final position: %d ticks\n", get_encoderM1_ticks());
        printf("Motor 2 final position: %d ticks\n", get_encoderM2_ticks());
        printf("Waiting 10 seconds before restarting...\n\n");
        sleep_ms(10000);
#endif
    }
    return 0;
}