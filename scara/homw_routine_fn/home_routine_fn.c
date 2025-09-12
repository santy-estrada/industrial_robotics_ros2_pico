#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "pwm/pwm.h"
#include "motor/motor.h"

// Test definitions - uncomment the test you want to run
// #define T1  // Home routine: Touch the 4 FC
#define M1
#define M2
// #define T2  // Home routine: Stop in the middle
#define T3  // Home routine: Toggle motor direction
#define T4  // Abilitate safe pendant

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
#define INA1_1 17
#define INA2_1 18
#define INB1_1 19
#define INB2_1 20
#define PWMB_1 21
#define BTN 28

volatile int32_t encoderM1_ticks = 0; // Store the number of encoder ticks
volatile int32_t encoderM2_ticks = 0; // Store the number of encoder ticks
int32_t rangeM1 =0;
int32_t rangeM2 =0;
int32_t midPoint1 =0;
int32_t midPoint2 =0;

const uint motor1[3] ={INA1_1, INA2_1, 0};
const uint motor2[3] ={INB1_1, INB2_1, 1};

bool dirM1 = false;
bool dirM2 = true;

// Function to get the current encoder tick count
int32_t get_encoderM1_ticks() {
    return encoderM1_ticks;
}

int32_t get_encoderM2_ticks() {
    return encoderM2_ticks;
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

#ifdef T4
void init_dead_man_switch(){
    gpio_init(BTN);
    gpio_set_dir(BTN, GPIO_IN);
    gpio_pull_up(BTN);
}
#endif

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


void init_gpios(){
    // Always initialize LED pin
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

#if defined T1 || defined T2 || defined T3
    // Test 1: Home routine with limit switches
    init_limit_sw();

    // Channels as interruptions
    init_channels();
    init_interruption();

    // Motors - use the modular motor functions
    init_motor(INA1_1, INA2_1);  // Initialize Motor 1
    init_motor(INB1_1, INB2_1);  // Initialize Motor 2
#endif

#ifdef T4
    //Dead man switch
    init_dead_man_switch();
#endif
}


int main()
{
    // Initialize stdio
    stdio_init_all();

    // Initialize the GPIO pins
    init_gpios();

    // Initialize specific test setups
#if defined T1 || defined T2 || defined T3
    // Initialize PWM for both motors
    init_pwm(PWMA_1, 12499, 1);  // 10 kHz PWM for Motor 1
    init_pwm(PWMB_1, 12499, 1);  // 10 kHz PWM for Motor 2

    // Home Routine: Move motors to touch all limit switches
    // 8% PWM speed
    int pwm_speed_percent = 6;
#endif

    gpio_put(LED_PIN, 1); // Turn on the onboard LED to indicate the program is running

    printf("Starting Home Routine...\n");

    while (true) {
#ifdef T1
    #ifdef M1
        printf("=== Starting Motor 1 Homing ===\n");
        
        // Reset encoder count for Motor 1
        encoderM1_ticks = 0;
        
        // Motor 1 - Direction 1 (towards FC_1L)
        printf("Motor 1: Moving towards FC_1L...\n");
        mv_cw(motor1);  // Motor 1 Direction 1
        stop_motor(motor2);  // Keep Motor 2 stopped
        
        // Start Motor 1, stop Motor 2
        set_vel(PWMA_1, pwm_speed_percent);
        stop_pwm(PWMB_1);
        
        // Move until FC_1L is touched
        while (gpio_get(FC_1L)) {  // FC switches are pulled up, so they read 1 when not pressed
            printf("Motor 1 ticks: %d\n", get_encoderM1_ticks());
            sleep_ms(100);
        }
        encoderM1_ticks = 0;
        rangeM1 = 0;
        printf("Range M1: %d\n", rangeM1);
        // Stop Motor 1 - FC_1L touched
        stop_pwm(PWMA_1);
        stop_pwm(PWMB_1);
        printf("FC_1L ACTIVATED Motor 1 stopped. Ticks: %d\n", get_encoderM1_ticks());
        sleep_ms(1000);
        
        // Motor 1 - Direction 2 (towards FC_1R)
        printf("Motor 1: Moving towards FC_1R...\n");
        mv_ccw(motor1);  // Motor 1 Direction 2
        stop_motor(motor2);  // Keep Motor 2 stopped
        
        // Start Motor 1, keep Motor 2 stopped
        set_vel(PWMA_1, pwm_speed_percent);
        stop_pwm(PWMB_1);
        
        // Move until FC_1R is touched
        while (gpio_get(FC_1R)) {  // FC switches are pulled up, so they read 1 when not pressed
            printf("Motor 1 ticks: %d\n", get_encoderM1_ticks());
            sleep_ms(100);
        }

        rangeM1 = get_encoderM1_ticks();
        printf("Range M1: %d\n", rangeM1);
        // Stop Motor 1 - FC_1R touched
        stop_pwm(PWMA_1);
        stop_pwm(PWMB_1);
        printf("*** FC_1R ACTIVATED *** Motor 1 stopped. Final ticks: %d\n", get_encoderM1_ticks());
        printf("=== Motor 1 Homing Complete ===\n\n");
        sleep_ms(1000);
    #endif
    #ifdef M2
        // Now start Motor 2 homing
        printf("=== Starting Motor 2 Homing ===\n");
        
        // Reset encoder count for Motor 2
        encoderM2_ticks = 0;
        
        // Motor 2 - Direction 1 (towards FC_2L)
        printf("Motor 2: Moving towards FC_2L...\n");
        stop_motor(motor1);  // Keep Motor 1 stopped
        mv_ccw(motor2);  // Motor 2 Direction 1
        
        // Start Motor 2, keep Motor 1 stopped
        stop_pwm(PWMA_1);
        set_vel(PWMB_1, pwm_speed_percent);
        
        // Move until FC_2L is touched
        while (gpio_get(FC_2L)) {  // FC switches are pulled up, so they read 1 when not pressed
            printf("Motor 2 ticks: %d\n", get_encoderM2_ticks());
            sleep_ms(100);
        }
        encoderM2_ticks = 0;
        rangeM2 = 0;
        printf("Range M2: %d\n", rangeM2);
        // Stop Motor 2 - FC_2L touched
        stop_pwm(PWMA_1);
        stop_pwm(PWMB_1);
        printf("*** FC_2L ACTIVATED *** Motor 2 stopped. Ticks: %d\n", get_encoderM2_ticks());
        sleep_ms(1000);
        
        // Motor 2 - Direction 2 (towards FC_2R)
        printf("Motor 2: Moving towards FC_2R...\n");
        stop_motor(motor1);  // Keep Motor 1 stopped
        mv_cw(motor2);  // Motor 2 Direction 2
        
        // Start Motor 2, keep Motor 1 stopped
        stop_pwm(PWMA_1);
        set_vel(PWMB_1, pwm_speed_percent);
        
        // Move until FC_2R is touched
        while (gpio_get(FC_2R)) {  // FC switches are pulled up, so they read 1 when not pressed
            printf("Motor 2 ticks: %d\n", get_encoderM2_ticks());
            sleep_ms(100);
        }

        rangeM2 = get_encoderM2_ticks();
        printf("Range M2: %d\n", rangeM2);
        
        // Stop Motor 2 - FC_2R touched
        stop_pwm(PWMA_1);
        stop_pwm(PWMB_1);
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
    #ifdef M2
        // Now start Motor 2 homing
        printf("=== Starting Motor 2 Homing ===\n");
        
        // Reset encoder count for Motor 2
        encoderM2_ticks = 0;
        
        // Motor 2 - Direction 1 (towards FC_2L)
        printf("Motor 2: Moving towards FC_2L...\n");
        stop_motor(motor1);  // Keep Motor 1 stopped
        mv_ccw(motor2);  // Motor 2 Direction 1
        
        // Start Motor 2, keep Motor 1 stopped
        stop_pwm(PWMA_1);
        set_vel(PWMB_1, pwm_speed_percent);
        
        // Move until FC_2L is touched
        while (gpio_get(FC_2L)) {  // FC switches are pulled up, so they read 1 when not pressed
            printf("Motor 2 ticks: %d\n", get_encoderM2_ticks());
            sleep_ms(100);
        }
        encoderM2_ticks = 0;
        rangeM2 = 0;
        printf("Range M2: %d\n", rangeM2);
        // Stop Motor 2 - FC_2L touched
        stop_pwm(PWMA_1);
        stop_pwm(PWMB_1);
        printf("*** FC_2L ACTIVATED *** Motor 2 stopped. Ticks: %d\n", get_encoderM2_ticks());
        sleep_ms(1000);
        
        // Motor 2 - Direction 2 (towards FC_2R)
        printf("Motor 2: Moving towards FC_2R...\n");
        stop_motor(motor1);  // Keep Motor 1 stopped  
        mv_cw(motor2);  // Motor 2 Direction 2
        
        // Start Motor 2, keep Motor 1 stopped
        stop_pwm(PWMA_1);
        set_vel(PWMB_1, pwm_speed_percent);
        
        // Move until FC_2R is touched
        while (gpio_get(FC_2R)) {  // FC switches are pulled up, so they read 1 when not pressed
            printf("Motor 2 ticks: %d\n", get_encoderM2_ticks());
            sleep_ms(100);
        }

        rangeM2 = get_encoderM2_ticks();
        midPoint2 = rangeM2 / 2;
        printf("Range M2: %d\n", rangeM2);
        
        // Stop Motor 2 - FC_2R touched
        stop_pwm(PWMA_1);
        stop_pwm(PWMB_1);
        printf("*** FC_2R ACTIVATED *** Motor 2 stopped. Final ticks: %d\n", get_encoderM2_ticks());
        sleep_ms(1000);

        // Move the arm towards the center
        encoderM2_ticks = 0;  // Reset encoder count
        printf("Motor 2: Moving towards center...\n");
        mv_ccw(motor2);  // Move back towards center
        stop_pwm(PWMA_1);
        set_vel(PWMB_1, pwm_speed_percent);

        int32_t cont = 0;
        while ((cont < labs(midPoint2) - 100) && gpio_get(FC_2L)) {
            printf("Motor 2 ticks: %d\n", get_encoderM2_ticks());
            cont = labs(get_encoderM2_ticks());
            sleep_ms(100);
        }
        // Stop Motor 2 - Center reached
        stop_pwm(PWMA_1);
        stop_pwm(PWMB_1);
        sleep_ms(1000);

        printf("=== Motor 2 Homing Complete ===\n\n");
    #endif

    #ifdef M1
        printf("=== Starting Motor 1 Homing ===\n");
        
        // Reset encoder count for Motor 1
        encoderM1_ticks = 0;
        
        // Motor 1 - Direction 1 (towards FC_1L)
        printf("Motor 1: Moving towards FC_1L...\n");
        mv_cw(motor1);  // Motor 1 Direction 1
        stop_motor(motor2);  // Keep Motor 2 stopped
        
        stop_pwm(PWMB_1);
        set_vel(PWMA_1, pwm_speed_percent);

        // Move until FC_1L is touched
        while (gpio_get(FC_1L)) {  // FC switches are pulled up, so they read 1 when not pressed
            printf("Motor 1 ticks: %d\n", get_encoderM1_ticks());
            sleep_ms(100);
        }
        encoderM1_ticks = 0;
        rangeM1 = 0;
        printf("Range M1: %d\n", rangeM1);
        // Stop Motor 1 - FC_1L touched
        stop_pwm(PWMA_1);
        stop_pwm(PWMB_1);
        printf("FC_1L ACTIVATED Motor 1 stopped. Ticks: %d\n", get_encoderM1_ticks());
        sleep_ms(1000);
        
        // Motor 1 - Direction 2 (towards FC_1R)
        printf("Motor 1: Moving towards FC_1R...\n");
        mv_ccw(motor1);  // Motor 1 Direction 2
        stop_motor(motor2);  // Keep Motor 2 stopped
        
        // Start Motor 1, keep Motor 2 stopped
        set_vel(PWMA_1, pwm_speed_percent);
        stop_pwm(PWMB_1);
        
        // Move until FC_1R is touched
        while (gpio_get(FC_1R)) {  // FC switches are pulled up, so they read 1 when not pressed
            printf("Motor 1 ticks: %d\n", get_encoderM1_ticks());
            sleep_ms(100);
        }

        rangeM1 = get_encoderM1_ticks();
        midPoint1 = rangeM1 / 2;

        printf("Range M1: %d\n", rangeM1);

        // Stop Motor 1 - FC_1R touched
        stop_pwm(PWMA_1);
        stop_pwm(PWMB_1);
        printf("*** FC_1R ACTIVATED *** Motor 1 stopped. Final ticks: %d\n", get_encoderM1_ticks());
        sleep_ms(1000);

        // Move the arm towards the center
        encoderM1_ticks = 0;  // Reset encoder count
        printf("Motor 1: Moving towards center...\n");
        mv_cw(motor1);  // Move back towards center
        stop_motor(motor2);
        set_vel(PWMA_1, pwm_speed_percent);
        stop_pwm(PWMB_1);

        #ifndef M2
        int32_t cont = 0;
        #else
        cont = 0;
        #endif
        while ((cont < labs(midPoint1)) && gpio_get(FC_1L)) {
            printf("Motor 1 ticks: %d\n", get_encoderM1_ticks());
            cont = labs(get_encoderM1_ticks());
            sleep_ms(100)
        }
        // Stop Motor 1 - Center reached
        stop_pwm(PWMA_1);
        stop_pwm(PWMB_1);

        printf("=== Motor 1 Homing Complete ===\n\n");
        sleep_ms(1000);
    #endif

        printf("*** HOME ROUTINE COMPLETE ***\n");
        printf("Motor 1 final position: %d ticks\n", get_encoderM1_ticks());
        printf("Motor 2 final position: %d ticks\n", get_encoderM2_ticks());
        printf("Waiting 10 seconds before restarting...\n\n");
        sleep_ms(10000);
#endif

#ifdef T3
    #ifdef M2
        // Motor 2 homing with toggle_dir
        printf("=== Starting Motor 2 Homing (T3) ===\n");
        
        // Reset encoder count for Motor 2
        encoderM2_ticks = 0;
        
        // Motor 2 - Direction 1 (towards FC_2L)
        printf("Motor 2: Moving towards FC_2L...\n");
        stop_motor(motor1);  // Keep Motor 1 stopped
        // Set initial direction for motor2 (ccw towards FC_2L)
        mv_ccw(motor2);
        
        // Start Motor 2, keep Motor 1 stopped
        stop_pwm(PWMA_1);
        set_vel(PWMB_1, pwm_speed_percent);
        
        // Move until FC_2L is touched
        while (gpio_get(FC_2L)) {
            #ifdef T4
            // Wait until dead_man switch is pressed
            while(gpio_get(BTN)){
                stop_motor(motor2);
                stop_pwm(PWMB_1);
                sleep_ms(50);
            }
            resume_motor(motor2);
            set_vel(PWMB_1, pwm_speed_percent);
            #endif
            printf("Motor 2 ticks: %d\n", get_encoderM2_ticks());
            sleep_ms(100);
        }
        encoderM2_ticks = 0;
        rangeM2 = 0;
        printf("Range M2: %d\n", rangeM2);
        // Stop Motor 2 - FC_2L touched
        stop_pwm(PWMA_1);
        stop_pwm(PWMB_1);
        printf("*** FC_2L ACTIVATED *** Motor 2 stopped. Ticks: %d\n", get_encoderM2_ticks());
        sleep_ms(1000);
        
        // Motor 2 - Toggle direction (towards FC_2R)
        printf("Motor 2: Toggling direction towards FC_2R...\n");
        stop_motor(motor1);  // Keep Motor 1 stopped  
        toggle_dir(motor2);  // Toggle Motor 2 direction (motor index 1)
        
        // Start Motor 2, keep Motor 1 stopped
        stop_pwm(PWMA_1);
        set_vel(PWMB_1, pwm_speed_percent);
        
        // Move until FC_2R is touched
        while (gpio_get(FC_2R)) {
            #ifdef T4
            // Wait until dead_man switch is pressed
            while(gpio_get(BTN)){
                stop_motor(motor2);
                stop_pwm(PWMB_1);
                sleep_ms(50);
            }
            resume_motor(motor2);
            set_vel(PWMB_1, pwm_speed_percent);
            #endif
            printf("Motor 2 ticks: %d\n", get_encoderM2_ticks());
            sleep_ms(100);
        }

        rangeM2 = get_encoderM2_ticks();
        midPoint2 = rangeM2 / 2;
        printf("Range M2: %d\n", rangeM2);
        
        // Stop Motor 2 - FC_2R touched
        stop_pwm(PWMA_1);
        stop_pwm(PWMB_1);
        printf("*** FC_2R ACTIVATED *** Motor 2 stopped. Final ticks: %d\n", get_encoderM2_ticks());
        sleep_ms(1000);

        // Move the arm towards the center
        encoderM2_ticks = 0;  // Reset encoder count
        printf("Motor 2: Moving towards center...\n");
        toggle_dir(motor2);  // Toggle back to ccw direction
        stop_pwm(PWMA_1);
        set_vel(PWMB_1, pwm_speed_percent);

        int32_t cont = 0;
        while ((cont < labs(midPoint2) - 100) && gpio_get(FC_2L)) {
            #ifdef T4
            // Wait until dead_man switch is pressed
            while(gpio_get(BTN)){
                stop_motor(motor2);
                stop_pwm(PWMB_1);
                sleep_ms(50);
            }
            resume_motor(motor2);
            set_vel(PWMB_1, pwm_speed_percent);
            #endif
            printf("Motor 2 ticks: %d\n", get_encoderM2_ticks());
            cont = labs(get_encoderM2_ticks());
            sleep_ms(100);
        }
        printf("Reached cont: %d\n", cont);

        // Stop Motor 2 - Center reached
        stop_pwm(PWMA_1);
        stop_pwm(PWMB_1);
        sleep_ms(1000);

        printf("=== Motor 2 Homing Complete (T3) ===\n\n");
    #endif

    #ifdef M1
        printf("=== Starting Motor 1 Homing (T3) ===\n");
        
        // Reset encoder count for Motor 1
        encoderM1_ticks = 0;
        
        // Motor 1 - Direction 1 (towards FC_1L)
        printf("Motor 1: Moving towards FC_1L...\n");
        // Set initial direction for motor1 (cw towards FC_1L)
        mv_cw(motor1);
        stop_motor(motor2);  // Keep Motor 2 stopped
        
        stop_pwm(PWMB_1);
        set_vel(PWMA_1, pwm_speed_percent);

        // Move until FC_1L is touched
        while (gpio_get(FC_1L)) {
            #ifdef T4
            // Wait until dead_man switch is pressed
            while(gpio_get(BTN)){
                stop_motor(motor1);
                stop_pwm(PWMA_1);
                sleep_ms(50);
            }
            resume_motor(motor1);
            set_vel(PWMA_1, pwm_speed_percent);
            #endif
            printf("Motor 1 ticks: %d\n", get_encoderM1_ticks());
            sleep_ms(100);
        }
        // Stop Motor 1 - FC_1L touched
        stop_pwm(PWMA_1);
        stop_pwm(PWMB_1);
        encoderM1_ticks = 0;
        rangeM1 = 0;
        printf("Range M1: %d\n", rangeM1);
        
        printf("*** FC_1L ACTIVATED *** Motor 1 stopped. Ticks: %d\n", get_encoderM1_ticks());
        sleep_ms(1000);
        
        // Motor 1 - Toggle direction (towards FC_1R)
        printf("Motor 1: Toggling direction towards FC_1R...\n");
        toggle_dir(motor1);  // Toggle Motor 1 direction (motor index 0)
        stop_motor(motor2);  // Keep Motor 2 stopped
        
        // Start Motor 1, keep Motor 2 stopped
        set_vel(PWMA_1, pwm_speed_percent);
        stop_pwm(PWMB_1);
        
        // Move until FC_1R is touched
        while (gpio_get(FC_1R)) {
            #ifdef T4
            // Wait until dead_man switch is pressed
            while(gpio_get(BTN)){
                stop_motor(motor1);
                stop_pwm(PWMA_1);
                sleep_ms(50);
            }
            resume_motor(motor1);
            set_vel(PWMA_1, pwm_speed_percent);
            #endif
            printf("Motor 1 ticks: %d\n", get_encoderM1_ticks());
            sleep_ms(100);
        }
        // Stop Motor 1 - FC_1R touched
        stop_pwm(PWMA_1);
        stop_pwm(PWMB_1);

        rangeM1 = get_encoderM1_ticks();
        midPoint1 = rangeM1 / 2;

        printf("Range M1: %d\n", rangeM1);

        printf("*** FC_1R ACTIVATED *** Motor 1 stopped. Final ticks: %d\n", get_encoderM1_ticks());
        sleep_ms(1000);

        // Move the arm towards the center
        encoderM1_ticks = 0;  // Reset encoder count
        printf("Motor 1: Moving towards center...\n");
        toggle_dir(motor1);  // Toggle back to cw direction
        stop_motor(motor2);
        set_vel(PWMA_1, pwm_speed_percent);
        stop_pwm(PWMB_1);

        #ifndef M2
        int32_t cont = 0;
        #else
        cont = 0;
        #endif
        while ((cont < labs(midPoint1)) && gpio_get(FC_1L)) {
            #ifdef T4
            // Wait until dead_man switch is pressed
            while(gpio_get(BTN)){
                stop_motor(motor1);
                stop_pwm(PWMA_1);
                sleep_ms(50);
            }
            resume_motor(motor1);
            set_vel(PWMA_1, pwm_speed_percent);
            #endif
            printf("Motor 1 ticks: %d\n", get_encoderM1_ticks());
            cont = labs(get_encoderM1_ticks());
            sleep_ms(100);
        }
        printf("Reached cont: %d\n", cont);
        // Stop Motor 1 - Center reached
        stop_pwm(PWMA_1);
        stop_pwm(PWMB_1);

        printf("=== Motor 1 Homing Complete (T3) ===\n\n");
        sleep_ms(1000);
    #endif

        printf("*** HOME ROUTINE COMPLETE (T3) ***\n");
        printf("Motor 1 final position: %d ticks\n", get_encoderM1_ticks());
        printf("Motor 2 final position: %d ticks\n", get_encoderM2_ticks());
        printf("Waiting 10 seconds before restarting...\n\n");
        sleep_ms(10000);
#endif
    }
    return 0;
}
