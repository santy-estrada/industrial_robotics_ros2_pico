#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include <math.h>

#define CHA_M1 12
#define CHB_M1 13
#define PWMA_1 16
#define INA2_1 17
#define INA1_1 18

#define debug
#define motor
#define pid_controller
// #define step

#define MOTOR_PIN 21   // GPIO pin connected to the DC motor
#define MOTOR_IN2 20  // GPIO pin connected to the DC motor CCW
#define MOTOR_IN1 19  // GPIO pin connected to the DC motor CW

// HC-SR05 Ultrasonic sensor pins
#define TRIG_PIN 27   // ADC1 - Trigger pin for HC-SR05
#define ECHO_PIN 26   // ADC0 - Echo pin for HC-SR05

#define LED_PIN 25    // GPIO pin connected to the onboard LED

// GPIO pins for encoders
const uint ENCODER_A_PIN = 14; // Encoder A signal Motor 1 (CHA_M2)
const uint ENCODER_B_PIN = 15; // Encoder B signal Motor 1 (CHB_M2)
const uint ENCODER2_A_PIN = 12; // Encoder A signal Motor 2 (CHA_M1)
const uint ENCODER2_B_PIN = 13; // Encoder B signal Motor 2 (CHB_M1)

// Encoder and motor parameters
const int TICKS_PER_REV = 28; // Encoder resolution (ticks per revolution)
const float GEAR_RATIO = 150.0f; // Gear ratio

volatile int32_t encoder_ticks = 0; // Store the number of encoder ticks Motor 1
volatile int32_t encoder2_ticks = 0; // Store the number of encoder ticks Motor 2

// Speed calculation variables Motor 1
int32_t last_ticks = 0;
float rpm = 0.0f;
int percentage = 0; // Percentage of speed (0-100)
static float filtered_rpm = 0.0f;

// Speed calculation variables Motor 2
int32_t last_ticks2 = 0;
float rpm2 = 0.0f;
int percentage2 = 0; // Percentage of speed (0-100)
static float filtered_rpm2 = 0.0f;

// Common timing and filter variables
float dt = 0.1f; // Time interval in seconds (adjust as needed)
static const float alpha = 0.15f;  // Low-pass filter coefficient

// PID Controller variables Motor 1
static const float q0 = 0.8206f;
static const float q1 = -0.6716f;
static const float q2 = 0.00018646f;
static float error[3] = {0.0f, 0.0f, 0.0f}; // e(k), e(k-1), e(k-2)
static float control_output = 0.0f;

// PID Controller variables Motor 2
static float error2[3] = {0.0f, 0.0f, 0.0f}; // e(k), e(k-1), e(k-2)
static float control_output2 = 0.0f;

// Common PID variables
static float setpoint_percentage = 20.0f; // Static setpoint
static const float MAX_RPM = 200.0f; // 100% corresponds to 200 RPM

// Obstacle detection variables
static bool obstacle_detected = false;
static bool obstacle_detected_prev = false;
static const float OBSTACLE_DISTANCE_CM = 10.0f; // Stop if object closer than 10cm
static long last_distance = 100; // Initialize to safe distance
static bool ultrasonic_measuring = false;

// Low-pass filter function (takes filter state as parameter)
float apply_low_pass_filter(float raw_rpm, float *filtered_state) {
    *filtered_state = alpha * raw_rpm + (1.0f - alpha) * (*filtered_state);
    return *filtered_state;
}

// PID Controller function Motor 1
float calculate_pid_control(float setpoint_rpm, float measured_rpm) {
    // Convert RPM to percentage for error calculation
    float setpoint_percent = (setpoint_rpm / MAX_RPM) * 100.0f;
    float measured_percent = (measured_rpm / MAX_RPM) * 100.0f;
    
    // Shift error history
    error[2] = error[1];  // e(k-2) = e(k-1)
    error[1] = error[0];  // e(k-1) = e(k)
    error[0] = setpoint_percent - measured_percent;  // e(k) = setpoint - measurement
    
    // Calculate control output using difference equation
    // u(k) = q0*e(k) + q1*e(k-1) + q2*e(k-2) + u(k-1)
    control_output = q0 * error[0] + q1 * error[1] + q2 * error[2] + control_output;

    // Limit control output to [0, 80] percentage
    if (control_output > 80.0f) control_output = 80.0f;
    if (control_output < 0.0f) control_output = 0.0f;
    
    return control_output;
}

// PID Controller function Motor 2
float calculate_pid_control2(float setpoint_rpm, float measured_rpm) {
    // Convert RPM to percentage for error calculation
    float setpoint_percent = (setpoint_rpm / MAX_RPM) * 100.0f;
    float measured_percent = (measured_rpm / MAX_RPM) * 100.0f;
    
    // Shift error history
    error2[2] = error2[1];  // e(k-2) = e(k-1)
    error2[1] = error2[0];  // e(k-1) = e(k)
    error2[0] = setpoint_percent - measured_percent;  // e(k) = setpoint - measurement
    
    // Calculate control output using difference equation
    control_output2 = q0 * error2[0] + q1 * error2[1] + q2 * error2[2] + control_output2;

    // Limit control output to [0, 80] percentage
    if (control_output2 > 80.0f) control_output2 = 80.0f;
    if (control_output2 < 0.0f) control_output2 = 0.0f;
    
    return control_output2;
}

// Function to measure distance using HC-SR05 ultrasonic sensor with fast timeout
long measure_distance_fast() {
    // Send trigger pulse (HC-SR05 needs at least 10us trigger pulse)
    gpio_put(TRIG_PIN, 1);  // Set trigger high
    sleep_us(10);           // Wait 10 microseconds (optimal for HC-SR05)
    gpio_put(TRIG_PIN, 0);  // Set trigger low
    
    // Wait for echo to go high (start of pulse) - REDUCED timeout for timing control
    uint32_t timeout = time_us_32() + 3000; // 3ms timeout (much faster)
    while (!gpio_get(ECHO_PIN)) {
        if (time_us_32() > timeout) {
            return last_distance; // Return last known distance on timeout
        }
    }
    
    // Measure pulse duration
    uint32_t start_time = time_us_32();
    
    // Wait for echo to go low (end of pulse) - REDUCED timeout
    timeout = start_time + 3000; // 3ms timeout for echo pulse
    while (gpio_get(ECHO_PIN)) {
        if (time_us_32() > timeout) {
            return last_distance; // Return last known distance on timeout
        }
    }
    
    uint32_t end_time = time_us_32();
    
    // Calculate distance in cm
    long pulse_time = end_time - start_time;
    long distance = pulse_time / 58;
    
    // Validate distance (HC-SR05 range: 2cm to 400cm)
    if (distance >= 2) {
        last_distance = distance; // Update last known good distance
        return distance;
    } else {
        return last_distance; // Return last known distance if invalid reading
    }
}

// Convert percentage to PWM value
uint16_t percentage_to_pwm(float percentage) {
    if (percentage < 0.0f) percentage = 0.0f;
    if (percentage > 100.0f) percentage = 100.0f;
    return (uint16_t)((percentage / 100.0f) * 12499.0f);
}

// Map helper function: converts value from [in_min, in_max] to [out_min, out_max]
long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// GPIO interrupt handler for both motor encoders (single function)
void encoder_a_irq_handler(uint gpio, uint32_t events) {
    if (gpio == ENCODER_A_PIN || gpio == ENCODER_B_PIN){
        // Motor 1 encoder handling
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
    } else {
        // Motor 2 encoder handling
        bool encoder2_a = gpio_get(ENCODER2_A_PIN);
        bool encoder2_b = gpio_get(ENCODER2_B_PIN);

        // Determine direction based on the quadrature signals
        // Count rising and falling edges of both A and B channels
        if (gpio == ENCODER2_A_PIN) {
            if ((encoder2_a && !encoder2_b) || (!encoder2_a && encoder2_b)) {
                encoder2_ticks++;  // Forward direction
            } else {
                encoder2_ticks--;  // Reverse direction
            }
        } else {
            if ((encoder2_a && !encoder2_b) || (!encoder2_a && encoder2_b)) {
                encoder2_ticks--;  // Forward direction
            } else {
                encoder2_ticks++;  // Reverse direction
            }
        }
    }
}

// Function to initialize both motor encoders
void init_encoder() {
    // Initialize Motor 1 encoder pins
    gpio_init(ENCODER_A_PIN);
    gpio_set_dir(ENCODER_A_PIN, GPIO_IN);
    gpio_pull_up(ENCODER_A_PIN);
    gpio_init(ENCODER_B_PIN);
    gpio_set_dir(ENCODER_B_PIN, GPIO_IN);
    gpio_pull_up(ENCODER_B_PIN);

    // Initialize Motor 2 encoder pins
    gpio_init(ENCODER2_A_PIN);
    gpio_set_dir(ENCODER2_A_PIN, GPIO_IN);
    gpio_pull_up(ENCODER2_A_PIN);
    gpio_init(ENCODER2_B_PIN);
    gpio_set_dir(ENCODER2_B_PIN, GPIO_IN);
    gpio_pull_up(ENCODER2_B_PIN);

    // Initialize additional pins (legacy - can be removed if not needed)
    gpio_init(CHA_M1);
    gpio_set_dir(CHA_M1, GPIO_IN);
    gpio_pull_up(CHA_M1);
    gpio_init(CHB_M1);
    gpio_set_dir(CHB_M1, GPIO_IN);
    gpio_pull_up(CHB_M1);

    // Attach interrupts for Motor 1 encoder
    gpio_set_irq_enabled_with_callback(ENCODER_A_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_a_irq_handler);
    gpio_set_irq_enabled_with_callback(ENCODER_B_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_a_irq_handler);
    
    // Attach interrupts for Motor 2 encoder
    gpio_set_irq_enabled_with_callback(ENCODER2_A_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_a_irq_handler);
    gpio_set_irq_enabled_with_callback(ENCODER2_B_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_a_irq_handler);
}

// Function to initialize HC-SR05 ultrasonic sensor
void init_ultrasonic() {
    // Initialize trigger pin (output)
    gpio_init(TRIG_PIN);
    gpio_set_dir(TRIG_PIN, GPIO_OUT);
    gpio_put(TRIG_PIN, 0);  // Initialize trigger low
    
    // Initialize echo pin (input)
    gpio_init(ECHO_PIN);
    gpio_set_dir(ECHO_PIN, GPIO_IN);
}

// Function to get the current encoder tick counts
int32_t get_encoder_ticks() {
    return encoder_ticks;
}

int32_t get_encoder2_ticks() {
    return encoder2_ticks;
}

int main() {
    // Initialize the stdio for UART output (optional)
    stdio_init_all();

    // Initialize the onboard LED
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

#ifdef motor
    // Initialize the GPIO pins connected to the DC motor
    gpio_init(MOTOR_IN1);
    gpio_set_dir(MOTOR_IN1, GPIO_OUT);
    

    gpio_init(MOTOR_IN2);
    gpio_set_dir(MOTOR_IN2, GPIO_OUT); 

    gpio_init(INA2_1);
    gpio_set_dir(INA2_1, GPIO_OUT);
    

    gpio_init(INA1_1);
    gpio_set_dir(INA1_1, GPIO_OUT); 
    
    
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

        // Set the GPIO pin function to PWM
    gpio_set_function(PWMA_1, GPIO_FUNC_PWM);
    
    // Get the PWM slice number associated with the GPIO pin
    uint slice_num2 = pwm_gpio_to_slice_num(PWMA_1);
    
    // Set the wrap value for a 10 kHz PWM signal (calculated wrap = 12499)
    // PWM frequency = 125 MHz / wrap / 2
    // 12499 = 125000000 / 10000 / 2
    // 10 kHz PWM signal for a DC motor driver (L298N)
    pwm_set_wrap(slice_num2, 12499);
    
    // Start with a 0 % duty cycle
    pwm_set_chan_level(slice_num2, pwm_gpio_to_channel(PWMA_1), 0);
    
    // Enable PWM output on the slice
    pwm_set_enabled(slice_num2, true);

    // Initialize the encoder
    init_encoder();
    
    // Initialize the ultrasonic sensor
    init_ultrasonic();
#endif

    // Blink the onboard LED to indicate the program is running
    gpio_put(LED_PIN, 1);
    
#ifdef motor
    uint64_t last_time_vel = to_ms_since_boot(get_absolute_time());
    uint64_t last_time_ultrasonic = to_ms_since_boot(get_absolute_time());
    #ifdef step
    uint64_t last_time_step = to_ms_since_boot(get_absolute_time());
    int phase = 0;
    int step_duration = 10000; // Duration of each step in milliseconds
    #endif
    int32_t ticks_since_last = 0;
    int32_t ticks_since_last2 = 0;
#endif
    uint64_t current_time = 0;

    sleep_ms(7000);     // Wait 7 seconds for system to stabilize
    while (1) {
        current_time = to_ms_since_boot(get_absolute_time());

        // Ultrasonic sensor measurement (separate timing - every 300ms)
        if (current_time - last_time_ultrasonic >= 300) {
            long distance = measure_distance_fast();
            
            // Update obstacle detection state
            obstacle_detected_prev = obstacle_detected;
            if (distance >= 2 && distance <= OBSTACLE_DISTANCE_CM) { // Valid distance and obstacle detected
                obstacle_detected = true;
            } else {
                obstacle_detected = false;
            }
            
            // Print obstacle detection messages (only once per state change)
            if (obstacle_detected && !obstacle_detected_prev) {
                printf("STOPPED - Obstacle detected at %ld cm\n", distance);
            } else if (!obstacle_detected && obstacle_detected_prev) {
                printf("CONTINUE - Obstacle cleared\n");
            }
            
            last_time_ultrasonic = current_time;
        }

#ifdef step
        if (current_time - last_time_step >= step_duration){
            phase++;
            if (phase > 3) phase = 0;
            last_time_step = current_time;

            setpoint_percentage = phase * 5.0f + 10.0f; // Setpoint changes in steps of 5% (10%, 15%, 20%, 25%)
        }
#endif

#ifdef motor
        // PID Control Loop (every 100ms - maintains precise timing)
        if (current_time - last_time_vel >= dt*1000) {

            // PID Controller implementation (only run if no obstacle)
            if (!obstacle_detected) {
                // Set motor directions (both motors forward)
                gpio_put(MOTOR_IN1, 0);
                gpio_put(MOTOR_IN2, 1);
                gpio_put(INA1_1, 0);
                gpio_put(INA2_1, 1);

                // Calculate setpoint in RPM (same for both motors)
                float setpoint_rpm = (setpoint_percentage / 100.0f) * MAX_RPM;

                // Motor 1 Control
                ticks_since_last = encoder_ticks - last_ticks;
                last_ticks = encoder_ticks;
                float raw_rpm = ((float)ticks_since_last / TICKS_PER_REV) * (60.0f / dt) * (1.0f / GEAR_RATIO);
                rpm = apply_low_pass_filter(raw_rpm, &filtered_rpm);
                
                float pid_output = calculate_pid_control(setpoint_rpm, rpm);
                uint16_t pwm_value = percentage_to_pwm(pid_output);
                pwm_set_chan_level(slice_num, pwm_gpio_to_channel(MOTOR_PIN), pwm_value);
                percentage = (int)pid_output;

                // Motor 2 Control
                ticks_since_last2 = encoder2_ticks - last_ticks2;
                last_ticks2 = encoder2_ticks;
                float raw_rpm2 = ((float)ticks_since_last2 / TICKS_PER_REV) * (60.0f / dt) * (1.0f / GEAR_RATIO);
                rpm2 = apply_low_pass_filter(raw_rpm2, &filtered_rpm2);
                
                float pid_output2 = calculate_pid_control2(setpoint_rpm, rpm2);
                uint16_t pwm_value2 = percentage_to_pwm(pid_output2);
                pwm_set_chan_level(slice_num2, pwm_gpio_to_channel(PWMA_1), pwm_value2);
                percentage2 = (int)pid_output2;
                
            } else {
                // Stop both motors when obstacle detected
                pwm_set_chan_level(slice_num, pwm_gpio_to_channel(MOTOR_PIN), 0);
                pwm_set_chan_level(slice_num2, pwm_gpio_to_channel(PWMA_1), 0);
                
                // Update variables to maintain continuity when resuming
                // Motor 1
                ticks_since_last = encoder_ticks - last_ticks;
                last_ticks = encoder_ticks;
                rpm = 0.0f;
                percentage = 0;
                
                // Motor 2
                ticks_since_last2 = encoder2_ticks - last_ticks2;
                last_ticks2 = encoder2_ticks;
                rpm2 = 0.0f;
                percentage2 = 0;
            }
            
            #ifdef debug
                if (obstacle_detected) {
                    printf("OBSTACLE DETECTED - Motors stopped. Distance: %ld cm\n", last_distance);
                } else {
                    printf("M1: %.1f%% (%.1f RPM), Measured: %.2f RPM, PID: %.2f%% | M2: %.2f RPM, PID: %.2f%% | Distance: %ld cm\n", 
                           setpoint_percentage, (setpoint_percentage / 100.0f) * MAX_RPM, rpm, (float)percentage, rpm2, (float)percentage2, last_distance);
                }
            #endif

            last_time_vel = current_time;
            
        #ifndef debug
            printf("%llu, %d, %.2f, %d, %d, %.2f, %d, %.1f, %.2f, %.2f, %ld, %d\n", 
                   current_time, encoder_ticks, rpm, percentage, encoder2_ticks, rpm2, percentage2, setpoint_percentage, error[0], error2[0], last_distance, obstacle_detected ? 1 : 0);
        #endif
        }
#endif
      
    }

    return 0;
}