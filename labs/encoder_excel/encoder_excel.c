#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include <math.h>

// #define debug
#define motor
// #define pot
// #define reaction_curve
#define pololu_test
// #define microgear_test

// #define CHA_M1 12
// #define CHB_M1 13
// #define CHA_M2 14
// #define CHB_M2 15
// #define PWMA_1 16
// #define INA1_1 17
// #define INA2_1 18
// #define INB1_1 19
// #define INB2_1 20
// #define PWMB_1 21

#define MOTOR_PIN 21   // GPIO pin connected to the DC motor
#define MOTOR_IN2 20  // GPIO pin connected to the DC motor CCW
#define MOTOR_IN1 19  // GPIO pin connected to the DC motor CW

#define LED_PIN 25    // GPIO pin connected to the onboard LED

// GPIO pins for encoder
const uint ENCODER_A_PIN = 14; // Encoder A signal
const uint ENCODER_B_PIN = 15; // Encoder B signal

// Potentiometer pin
const uint POT_PIN = 26; // GPIO pin connected to the potentiometer

// Encoder and motor parameters
#ifdef microgear_test
const int TICKS_PER_REV = 28; // Encoder resolution for microgear motor
const float GEAR_RATIO = 150.0f; // Gear ratio for microgear motor
#elif defined(pololu_test)
const int TICKS_PER_REV = 64; // Encoder resolution for Pololu motor  
const float GEAR_RATIO = 50.0f; // Gear ratio for Pololu motor
#else
const int TICKS_PER_REV = 64; // Encoder resolution (ticks per revolution) 28 for microgear; 64 for pololu
const float GEAR_RATIO = 50.0f; // Gear ratio 150.0f for microgear; 50.0f for pololu
#endif

volatile int32_t encoder_ticks = 0; // Store the number of encoder tick

// Speed calculation variables
int32_t last_ticks = 0;
float dt = 0.05f; // Time interval in seconds (adjust as needed)
float rpm = 0.0f;
int percentage = 0; // Percentage of speed (0-100)
// Low-pass filter variables (same as uros code)
static float filtered_rpm = 0.0f;
static const float alpha = 0.15f;  // Low-pass filter coefficient

// Low-pass filter function (same as uros code)
float apply_low_pass_filter(float raw_rpm) {
    filtered_rpm = alpha * raw_rpm + (1.0f - alpha) * filtered_rpm;
    return filtered_rpm;
}

// Map helper function: converts value from [in_min, in_max] to [out_min, out_max]
long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// GPIO interrupt handler for Encoder A pin
void encoder_a_irq_handler(uint gpio, uint32_t events) {
    // Read both encoder A and B pins
    bool encoder_a = gpio_get(ENCODER_A_PIN);
    bool encoder_b = gpio_get(ENCODER_B_PIN);

    // Determine direction based on the quadrature signals
    // Count rising and falling edges of both A and B channels
    if (gpio == ENCODER_A_PIN) {
        if ((encoder_a && !encoder_b) || (!encoder_a && encoder_b)) {
            encoder_ticks--;  // Forward direction
        } else {
            encoder_ticks++;  // Reverse direction
        }
    } else {
        if ((encoder_a && !encoder_b) || (!encoder_a && encoder_b)) {
            encoder_ticks++;  // Forward direction
        } else {
            encoder_ticks--;  // Reverse direction
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

#ifdef pot
    // Initialize ADC for potentiometer input
    adc_init();
    adc_gpio_init(POT_PIN);  // Enable GPIO pin for ADC
    adc_select_input(0);                  // ADC input channel for POT1
#endif

#ifdef motor
    // Initialize the GPIO pins connected to the DC motor
    gpio_init(MOTOR_IN1);
    gpio_set_dir(MOTOR_IN1, GPIO_OUT);
    

    gpio_init(MOTOR_IN2);
    gpio_set_dir(MOTOR_IN2, GPIO_OUT); 
    
    
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
#endif

    // Blink the onboard LED to indicate the program is running
    gpio_put(LED_PIN, 1);
    
    uint64_t last_time_step = to_ms_since_boot(get_absolute_time());
    int step = 0;
#ifdef motor
    uint64_t last_time_vel = to_ms_since_boot(get_absolute_time());
    int32_t ticks_since_last = 0;
#endif
    uint64_t current_time = 0;

    //Potentiometer reading variables
#ifdef pot
    uint16_t result_pot1 = 0;    // Raw 12-bit ADC value [0–4095]
    uint64_t last_time_pot = to_ms_since_boot(get_absolute_time());
    int pot_value = 0;
#endif

#ifdef microgear_test
    int step_duration = 15000; // Duration of each step in milliseconds (15 seconds for microgear)
#elif defined(pololu_test)
    int step_duration = 10000; // Duration of each step in milliseconds (10 seconds for Pololu)
#elif defined(reaction_curve)
    int step_duration = 15000; // Duration of each step in milliseconds
#else
    int duration = 2000; // Duration of each step in milliseconds
#endif

#ifdef microgear_test
    sleep_ms(20000);    // Wait 20 seconds (time to connect everything)
#elif defined(pololu_test)
    sleep_ms(15000);    // Wait 15 seconds (time to connect everything) 
#endif
    while (1) {
        current_time = to_ms_since_boot(get_absolute_time());
        if (current_time - last_time_step >= step_duration) {
            step++; // Wait until 2000 milliseconds have passed
            last_time_step = current_time;
        }

#ifdef pot
        if (step == 1) {
            result_pot1 = adc_read();    // Raw 12-bit ADC value [0–4095]
            pot_value = map(result_pot1, 0, 4095, 0, 100);  // Map ADC value to [0, 100]
        }else if (step == 2)
        {
            result_pot1 = adc_read();    // Raw 12-bit ADC value [0–4095]
            pot_value = map(result_pot1, 0, 4095, -100, 0);  // Map ADC value to [-100, 0]
        }else if (step == 3)
        {
            result_pot1 = adc_read();    // Raw 12-bit ADC value [0–4095]
            pot_value = map(result_pot1, 0, 4095, -100, 100);  // Map ADC value to [-100, 100]
        }else if (step >= 4)
        {
            step = 1;
        }
        
        if (current_time - last_time_pot >= dt*2000) {
    #ifdef debug
            printf("Step: %d Potentiometer value: %d\n", step, pot_value);
    #endif
    #ifndef debug
            printf("%u,%d,%d\n", result_pot1, pot_value, step);
    #endif
        last_time_pot = current_time;
        }

#endif

#ifdef motor
        if (current_time - last_time_vel >= dt*1000) {

        #ifdef microgear_test
        //Reaction curve of the microgear motor: 8 Steps (0->25->50->75->100->75->50->25->0). Each step lasts 15 seconds.
            gpio_put(MOTOR_IN1, 1);
            gpio_put(MOTOR_IN2, 0);
            if (step == 1){
                // Increase speed to 25% duty cycle
                pwm_set_chan_level(slice_num, pwm_gpio_to_channel(MOTOR_PIN), 3125);
                percentage = 25; // Set percentage to 25%
                #ifdef debug
                    printf("Running motor forward\n");
                    printf("Encoder ticks: %d\n", get_encoder_ticks());
                #endif
                // Calculate speed (RPM) - same as uros implementation
                ticks_since_last = encoder_ticks - last_ticks;
                last_ticks = encoder_ticks;
                float raw_rpm = ((float)ticks_since_last / TICKS_PER_REV) * (60.0f / dt) * (1.0f / GEAR_RATIO);
                rpm = apply_low_pass_filter(raw_rpm);
                #ifdef debug
                    printf("Speed (RPM): %.2f\n", rpm);
                #endif
            }else if (step == 2)
            {
                // Increase speed to 50% duty cycle
                pwm_set_chan_level(slice_num, pwm_gpio_to_channel(MOTOR_PIN), 6250);
                percentage = 50; // Set percentage to 50%
                #ifdef debug
                    printf("Encoder ticks: %d\n", get_encoder_ticks());
                #endif
                // Calculate speed (RPM) - same as uros implementation
                ticks_since_last = encoder_ticks - last_ticks;
                last_ticks = encoder_ticks;
                float raw_rpm = ((float)ticks_since_last / TICKS_PER_REV) * (60.0f / dt) * (1.0f / GEAR_RATIO);
                rpm = apply_low_pass_filter(raw_rpm);
                #ifdef debug
                    printf("Speed (RPM): %.2f\n", rpm);
                #endif
            }else if (step == 3)
            {
                // Increase speed to 75% duty cycle
                pwm_set_chan_level(slice_num, pwm_gpio_to_channel(MOTOR_PIN), 9375);
                percentage = 75; // Set percentage to 75%
                #ifdef debug
                    printf("Encoder ticks: %d\n", get_encoder_ticks());
                #endif
                // Calculate speed (RPM) - same as uros implementation
                ticks_since_last = encoder_ticks - last_ticks;
                last_ticks = encoder_ticks;
                float raw_rpm = ((float)ticks_since_last / TICKS_PER_REV) * (60.0f / dt) * (1.0f / GEAR_RATIO);
                rpm = apply_low_pass_filter(raw_rpm);
                #ifdef debug
                    printf("Speed (RPM): %.2f\n", rpm);
                #endif
            }else if (step == 4)
            {
                // Increase speed to almost full (95% duty cycle)
                pwm_set_chan_level(slice_num, pwm_gpio_to_channel(MOTOR_PIN), 11874);
                percentage = 95; // Set percentage to 95%
                #ifdef debug
                    printf("Encoder ticks: %d\n", get_encoder_ticks());
                #endif
                // Calculate speed (RPM) - same as uros implementation
                ticks_since_last = encoder_ticks - last_ticks;
                last_ticks = encoder_ticks;
                float raw_rpm = ((float)ticks_since_last / TICKS_PER_REV) * (60.0f / dt) * (1.0f / GEAR_RATIO);
                rpm = apply_low_pass_filter(raw_rpm);
                #ifdef debug
                    printf("Speed (RPM): %.2f\n", rpm);
                #endif
            } else if (step == 5)
            {
                // Decrease to 75% duty cycle
                pwm_set_chan_level(slice_num, pwm_gpio_to_channel(MOTOR_PIN), 9375);
                percentage = 75; // Set percentage to 75%
                #ifdef debug
                    printf("Encoder ticks: %d\n", get_encoder_ticks());
                #endif
                // Calculate speed (RPM)
                ticks_since_last = encoder_ticks - last_ticks;
                last_ticks = encoder_ticks;
                float raw_rpm = ((float)ticks_since_last / TICKS_PER_REV) * (60.0f / dt) * (1.0f / GEAR_RATIO);
                rpm = apply_low_pass_filter(raw_rpm);
                #ifdef debug
                    printf("Speed (RPM): %.2f\n", rpm);
                #endif
            } else if (step == 6)
            {
                // Decrease to 50% duty cycle
                pwm_set_chan_level(slice_num, pwm_gpio_to_channel(MOTOR_PIN), 6250);
                percentage = 50; // Set percentage to 50%
                #ifdef debug
                    printf("Encoder ticks: %d\n", get_encoder_ticks());
                #endif
                // Calculate speed (RPM)
                ticks_since_last = encoder_ticks - last_ticks;
                last_ticks = encoder_ticks;
                float raw_rpm = ((float)ticks_since_last / TICKS_PER_REV) * (60.0f / dt) * (1.0f / GEAR_RATIO);
                rpm = apply_low_pass_filter(raw_rpm);
                #ifdef debug
                    printf("Speed (RPM): %.2f\n", rpm);
                #endif
            } else if (step == 7)
            {
                // Decrease to 25% duty cycle
                pwm_set_chan_level(slice_num, pwm_gpio_to_channel(MOTOR_PIN), 3125);
                percentage = 25; // Set percentage to 25%
                #ifdef debug
                    printf("Encoder ticks: %d\n", get_encoder_ticks());
                #endif
                // Calculate speed (RPM)
                ticks_since_last = encoder_ticks - last_ticks;
                last_ticks = encoder_ticks;
                float raw_rpm = ((float)ticks_since_last / TICKS_PER_REV) * (60.0f / dt) * (1.0f / GEAR_RATIO);
                rpm = apply_low_pass_filter(raw_rpm);
                #ifdef debug
                    printf("Speed (RPM): %.2f\n", rpm);
                #endif
            } else if (step == 8)
            {
                //Set to 0% duty cycle
                pwm_set_chan_level(slice_num, pwm_gpio_to_channel(MOTOR_PIN), 0);
                percentage = 0; // Set percentage to 0%
                #ifdef debug
                    printf("Encoder ticks: %d\n", get_encoder_ticks());
                #endif
                // Calculate speed (RPM)
                ticks_since_last = encoder_ticks - last_ticks;
                last_ticks = encoder_ticks;
                float raw_rpm = ((float)ticks_since_last / TICKS_PER_REV) * (60.0f / dt) * (1.0f / GEAR_RATIO);
                rpm = apply_low_pass_filter(raw_rpm);
                #ifdef debug
                    printf("Speed (RPM): %.2f\n", rpm);
                #endif
            }else{
                sleep_ms(5000);
                step = 1;
            }
        
        #elif defined(pololu_test)
        //Pololu motor test: Steps of 10% from 0 to 60% and back to 0 (0->10->20->30->40->50->60->50->40->30->20->10->0). Each step lasts 10 seconds.
            gpio_put(MOTOR_IN1, 1);
            gpio_put(MOTOR_IN2, 0);
            
            int pwm_levels[] = {0, 1250, 2500, 3750, 5000, 6250, 7500, 6250, 5000, 3750, 2500, 1250, 0}; // PWM values for 0%, 10%, 20%, ..., 60%, ..., 10%, 0%
            int percentages[] = {0, 10, 20, 30, 40, 50, 60, 50, 40, 30, 20, 10, 0}; // Corresponding percentages
            
            if (step >= 1 && step <= 13) {
                int index = step - 1; // Convert step to array index (0-12)
                pwm_set_chan_level(slice_num, pwm_gpio_to_channel(MOTOR_PIN), pwm_levels[index]);
                percentage = percentages[index];
                
                #ifdef debug
                    printf("Step %d: %d%% duty cycle\n", step, percentage);
                    printf("Encoder ticks: %d\n", get_encoder_ticks());
                #endif
                
                // Calculate speed (RPM) - same as uros implementation
                ticks_since_last = encoder_ticks - last_ticks;
                last_ticks = encoder_ticks;
                float raw_rpm = ((float)ticks_since_last / TICKS_PER_REV) * (60.0f / dt) * (1.0f / GEAR_RATIO);
                rpm = apply_low_pass_filter(raw_rpm);
                
                #ifdef debug
                    printf("Speed (RPM): %.2f\n", rpm);
                #endif
            } else {
                sleep_ms(5000);
                step = 1;
            }

        #else
        // Default test (original bidirectional test)
            if (step == 1){
                gpio_put(MOTOR_IN1, 1);
                gpio_put(MOTOR_IN2, 0);
                // Increase speed to full (100% duty cycle)
                pwm_set_chan_level(slice_num, pwm_gpio_to_channel(MOTOR_PIN), 12499);
                percentage = 100; // Set percentage to 100%
                #ifdef debug
                    printf("Running motor forward\n");
                    printf("Encoder ticks: %d\n", get_encoder_ticks());
                #endif
                // Calculate speed (RPM)
                ticks_since_last = encoder_ticks - last_ticks;
                last_ticks = encoder_ticks;
                rpm = ((float)ticks_since_last / TICKS_PER_REV) * (60.0f / dt) * (1.0f / GEAR_RATIO);
                #ifdef debug
                    printf("Speed (RPM): %.2f\n", rpm);
                #endif
            } else if (step == 2)
            {
            // Decrease speed to half (50% duty cycle)
                pwm_set_chan_level(slice_num, pwm_gpio_to_channel(MOTOR_PIN), 6250);
                percentage = 50; // Set percentage to 50%
                #ifdef debug
                    printf("Encoder ticks: %d\n", get_encoder_ticks());
                #endif
                // Calculate speed (RPM)
                ticks_since_last = encoder_ticks - last_ticks;
                last_ticks = encoder_ticks;
                rpm = ((float)ticks_since_last / TICKS_PER_REV) * (60.0f / dt) * (1.0f / GEAR_RATIO);
                #ifdef debug
                    printf("Speed (RPM): %.2f\n", rpm);
                #endif
            } else if (step == 3)
            {
                // Stop the motor (0% duty cycle)
                pwm_set_chan_level(slice_num, pwm_gpio_to_channel(MOTOR_PIN), 0);
                percentage = 0; // Set percentage to 0%
                #ifdef debug
                    printf("Encoder ticks: %d\n", get_encoder_ticks());
                #endif  
                // Calculate speed (RPM)
                ticks_since_last = encoder_ticks - last_ticks;
                last_ticks = encoder_ticks;
                rpm = ((float)ticks_since_last / TICKS_PER_REV) * (60.0f / dt) * (1.0f / GEAR_RATIO);
                #ifdef debug
                    printf("Speed (RPM): %.2f\n", rpm);
                #endif
               
            }else if (step == 4)
            {
                gpio_put(MOTOR_IN2, 1);
                gpio_put(MOTOR_IN1, 0);
                // Increase speed to full (100% duty cycle)
                pwm_set_chan_level(slice_num, pwm_gpio_to_channel(MOTOR_PIN), 12499);
                percentage = -100; // Set percentage to -100%
                #ifdef debug
                    printf("Running motor backward\n");
                    printf("Encoder ticks: %d\n", get_encoder_ticks());
                #endif
                // Calculate speed (RPM)
                ticks_since_last = encoder_ticks - last_ticks;
                last_ticks = encoder_ticks;
                rpm = ((float)ticks_since_last / TICKS_PER_REV) * (60.0f / dt) * (1.0f / GEAR_RATIO);
                #ifdef debug
                    printf("Speed (RPM): %.2f\n", rpm);
                #endif
            } else if (step == 5)
            {
                // Decrease speed to half (50% duty cycle)
                pwm_set_chan_level(slice_num, pwm_gpio_to_channel(MOTOR_PIN), 6250);
                percentage = -50; // Set percentage to -50%
                #ifdef debug
                    printf("Encoder ticks: %d\n", get_encoder_ticks());
                #endif
                // Calculate speed (RPM)
                ticks_since_last = encoder_ticks - last_ticks;
                last_ticks = encoder_ticks;
                rpm = ((float)ticks_since_last / TICKS_PER_REV) * (60.0f / dt) * (1.0f / GEAR_RATIO);
                #ifdef debug
                    printf("Speed (RPM): %.2f\n", rpm);
                #endif  
            } else if (step == 6)
            {
                // Stop the motor (0% duty cycle)
                pwm_set_chan_level(slice_num, pwm_gpio_to_channel(MOTOR_PIN), 0);
                percentage = 0; // Set percentage to 0%
                #ifdef debug
                    printf("Encoder ticks: %d\n", get_encoder_ticks());
                #endif
                // Calculate speed (RPM)
                ticks_since_last = encoder_ticks - last_ticks;
                last_ticks = encoder_ticks;
                rpm = ((float)ticks_since_last / TICKS_PER_REV) * (60.0f / dt) * (1.0f / GEAR_RATIO);
                #ifdef debug
                    printf("Speed (RPM): %.2f\n", rpm);
                #endif
            } else if (step >= 7)
            {
                step = 0;
            }
        #endif // End of test type conditions
            
            last_time_vel = current_time;
            
        #ifndef debug
            printf("%llu, %d, %.2f, %d, %d\n", current_time, encoder_ticks, rpm, percentage, step);
        #endif
        }
#endif
      
    }

    return 0;
}