#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include <math.h>

#define debug
#define motor
#define pid_controller
#define step

#define MOTOR_PIN 17   // GPIO pin connected to the DC motor
#define MOTOR_IN2 20  // GPIO pin connected to the DC motor CCW
#define MOTOR_IN1 21  // GPIO pin connected to the DC motor CW

#define LED_PIN 25    // GPIO pin connected to the onboard LED

// GPIO pins for encoder
const uint ENCODER_A_PIN = 15; // Encoder A signal
const uint ENCODER_B_PIN = 14; // Encoder B signal

// Encoder and motor parameters
const int TICKS_PER_REV = 28; // Encoder resolution (ticks per revolution)
const float GEAR_RATIO = 150.0f; // Gear ratio

volatile int32_t encoder_ticks = 0; // Store the number of encoder tick

// Speed calculation variables
int32_t last_ticks = 0;
float dt = 0.1f; // Time interval in seconds (adjust as needed)
float rpm = 0.0f;
int percentage = 0; // Percentage of speed (0-100)
// Low-pass filter variables (same as uros code)
static float filtered_rpm = 0.0f;
static const float alpha = 0.15f;  // Low-pass filter coefficient

// PID Controller variables
static const float q0 = 0.8206f;   // PID coefficient
static const float q1 = -0.6716f;  // PID coefficient  
static const float q2 = 0.0001865f; // PID coefficient
static float error[3] = {0.0f, 0.0f, 0.0f}; // e(k), e(k-1), e(k-2)
static float control_output = 0.0f;
static float setpoint_percentage = 50.0f; // Static setpoint (50% = 100 RPM)
static const float MAX_RPM = 200.0f; // 100% corresponds to 200 RPM

// Low-pass filter function (same as uros code)
float apply_low_pass_filter(float raw_rpm) {
    filtered_rpm = alpha * raw_rpm + (1.0f - alpha) * filtered_rpm;
    return filtered_rpm;
}

// PID Controller function
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

    // Limit control output to [0, 100] percentage
    if (control_output > 100.0f) control_output = 100.0f;
    if (control_output < 0.0f) control_output = 0.0f;
    
    return control_output;
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
    
#ifdef motor
    uint64_t last_time_vel = to_ms_since_boot(get_absolute_time());
    #ifdef step
    uint64_t last_time_step = to_ms_since_boot(get_absolute_time());
    int phase = 0;
    int step_duration = 10000; // Duration of each step in milliseconds
    #endif
    int32_t ticks_since_last = 0;
#endif
    uint64_t current_time = 0;

    sleep_ms(7000);     // Wait 7 seconds for system to stabilize
    while (1) {
        current_time = to_ms_since_boot(get_absolute_time());

#ifdef step
        if (current_time - last_time_step >= step_duration){
            phase++;
            if (phase > 3) phase = 0;
            last_time_step = current_time;

            setpoint_percentage = phase * 15.0f + 10.0f; // Setpoint changes in steps of 15% (10%, 25%, 40%, 55%)
        }
#endif

#ifdef motor
        if (current_time - last_time_vel >= dt*1000) {

            // PID Controller implementation
            gpio_put(MOTOR_IN1, 1);
            gpio_put(MOTOR_IN2, 0);
            
            // Calculate speed (RPM) - same as previous implementation
            ticks_since_last = encoder_ticks - last_ticks;
            last_ticks = encoder_ticks;
            float raw_rpm = ((float)ticks_since_last / TICKS_PER_REV) * (60.0f / dt) * (1.0f / GEAR_RATIO);
            rpm = apply_low_pass_filter(raw_rpm);
            
            // Calculate setpoint in RPM (setpoint_percentage% of MAX_RPM)
            float setpoint_rpm = (setpoint_percentage / 100.0f) * MAX_RPM;
            
            // Calculate PID control output
            float pid_output = calculate_pid_control(setpoint_rpm, rpm);
            
            // Apply control output to motor
            uint16_t pwm_value = percentage_to_pwm(pid_output);
            pwm_set_chan_level(slice_num, pwm_gpio_to_channel(MOTOR_PIN), pwm_value);
            
            percentage = (int)pid_output; // Store for logging
            
            #ifdef debug
                printf("Setpoint: %.1f%% (%.1f RPM), Measured: %.2f RPM, Error: %.2f%%, PID Output: %.2f%%\n", 
                       setpoint_percentage, setpoint_rpm, rpm, error[0], pid_output);
            #endif

            last_time_vel = current_time;
            
        #ifndef debug
            printf("%llu, %d, %.2f, %d, %.1f, %.2f\n", current_time, encoder_ticks, rpm, percentage, setpoint_percentage, error[0]);
        #endif
        }
#endif
      
    }

    return 0;
}