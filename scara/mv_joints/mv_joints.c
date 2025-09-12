#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "pwm/pwm.h"
#include "motor/motor.h"

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
#define PWM_SERVO 22
#define BTN 28

// Control modes
#define MODE_MOTOR1 1
#define MODE_MOTOR2 2
#define MODE_SERVO 3

// Servo limits (in degrees)
#define SERVO_MIN_ANGLE -5
#define SERVO_MAX_ANGLE 20

volatile int32_t encoderM1_ticks = 0; // Store the number of encoder ticks
volatile int32_t encoderM2_ticks = 0; // Store the number of encoder ticks

const uint motor1[3] ={INA1_1, INA2_1, 0};
const uint motor2[3] ={INB1_1, INB2_1, 1};

// Global variables for control
int current_mode = 0;  // 0 = no mode selected
int current_servo_angle = 7;  // Start at middle position (7.5 degrees average)
int pwm_speed_percent = 5;  // 5% PWM speed for motors

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

// Function to set servo angle (in degrees)
void set_servo_angle(int angle) {
    if (angle < SERVO_MIN_ANGLE) angle = SERVO_MIN_ANGLE;
    if (angle > SERVO_MAX_ANGLE) angle = SERVO_MAX_ANGLE;
    
    // Map angle using the provided mapping function
    // Map from -5° to 20° range to the PWM values
    int pwm_value = map(angle, 0, 180, 977, 4883);
    
    uint slice_num = pwm_gpio_to_slice_num(PWM_SERVO);
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(PWM_SERVO), pwm_value);
    
    current_servo_angle = angle;
}

// Function to move Motor 1 in specified direction with limit switch checking
bool move_motor1(char direction) {
    if (direction == 'L') {  // Left - towards FC_1L
        if (gpio_get(FC_1L) == 0) {  // FC_1L is pressed (pulled up, so 0 = pressed)
            printf("WARNING: FC_1L limit switch activated! Motor 1 stopped.\n");
            stop_pwm(PWMA_1);
            stop_motor(motor1);
            return false;
        }
        mv_cw(motor1);
        set_vel(PWMA_1, pwm_speed_percent);
        printf("Motor 1 moving LEFT (towards FC_1L)\n");
        return true;
    } else if (direction == 'R') {  // Right - towards FC_1R
        if (gpio_get(FC_1R) == 0) {  // FC_1R is pressed
            printf("WARNING: FC_1R limit switch activated! Motor 1 stopped.\n");
            stop_pwm(PWMA_1);
            stop_motor(motor1);
            return false;
        }
        mv_ccw(motor1);
        set_vel(PWMA_1, pwm_speed_percent);
        printf("Motor 1 moving RIGHT (towards FC_1R)\n");
        return true;
    }
    return false;
}

// Function to move Motor 2 in specified direction with limit switch checking
bool move_motor2(char direction) {
    if (direction == 'L') {  // Left - towards FC_2L
        if (gpio_get(FC_2L) == 0) {  // FC_2L is pressed
            printf("WARNING: FC_2L limit switch activated! Motor 2 stopped.\n");
            stop_pwm(PWMB_1);
            stop_motor(motor2);
            return false;
        }
        mv_ccw(motor2);
        set_vel(PWMB_1, pwm_speed_percent);
        printf("Motor 2 moving LEFT (towards FC_2L)\n");
        return true;
    } else if (direction == 'R') {  // Right - towards FC_2R
        if (gpio_get(FC_2R) == 0) {  // FC_2R is pressed
            printf("WARNING: FC_2R limit switch activated! Motor 2 stopped.\n");
            stop_pwm(PWMB_1);
            stop_motor(motor2);
            return false;
        }
        mv_cw(motor2);
        set_vel(PWMB_1, pwm_speed_percent);
        printf("Motor 2 moving RIGHT (towards FC_2R)\n");
        return true;
    }
    return false;
}

// Function to stop all motors
void stop_all_motors() {
    stop_pwm(PWMA_1);
    stop_pwm(PWMB_1);
    stop_motor(motor1);
    stop_motor(motor2);
}

// Function to move servo up/down
void move_servo(char direction) {
    int new_angle = current_servo_angle;
    
    if (direction == 'U') {  // Up - increase angle
        new_angle += 1;
        if (new_angle > SERVO_MAX_ANGLE) {
            printf("WARNING: Servo at maximum angle (%d degrees)\n", SERVO_MAX_ANGLE);
            return;
        }
        printf("Servo moving UP to %d degrees\n", new_angle);
    } else if (direction == 'D') {  // Down - decrease angle
        new_angle -= 1;
        if (new_angle < SERVO_MIN_ANGLE) {
            printf("WARNING: Servo at minimum angle (%d degrees)\n", SERVO_MIN_ANGLE);
            return;
        }
        printf("Servo moving DOWN to %d degrees\n", new_angle);
    }
    
    set_servo_angle(new_angle);
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

void init_gpios(){
    // Always initialize LED pin
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // Initialize limit switches
    init_limit_sw();

    // Channels as interruptions
    init_channels();
    init_interruption();

    // Motors - use the modular motor functions
    init_motor(INA1_1, INA2_1);  // Initialize Motor 1
    init_motor(INB1_1, INB2_1);  // Initialize Motor 2
}

int main()
{
    // Initialize stdio
    stdio_init_all();

    // Initialize the GPIO pins
    init_gpios();

    // Initialize PWM for both motors and servo
    init_pwm(PWMA_1, 12499, 1);  // 10 kHz PWM for Motor 1
    init_pwm(PWMB_1, 12499, 1);  // 10 kHz PWM for Motor 2
    init_pwm(PWM_SERVO, 39060, 64); // 50 Hz for servo motor

    // Initialize servo to middle position
    set_servo_angle(current_servo_angle);

    gpio_put(LED_PIN, 1); // Turn on the onboard LED to indicate the program is running

    printf("=== Individual Actuator Control System ===\n");
    printf("Commands:\n");
    printf("1 - Select Motor 1\n");
    printf("2 - Select Motor 2\n");
    printf("3 - Select Servo\n");
    printf("\nControls:\n");
    printf("Motors: 'a' = Left, 'd' = Right\n");
    printf("Servo: 'w' = Up, 's' = Down\n");
    printf("Any other key = Stop current actuator\n");
    printf("\nEnter command: ");

    while (true) {
        char input_buffer[10];
        
        // Get input as string to handle both numbers and characters
        if (scanf("%9s", input_buffer) == 1) {
            // Check if input is a single digit for mode selection
            if (strlen(input_buffer) == 1 && input_buffer[0] >= '1' && input_buffer[0] <= '3') {
                int mode = input_buffer[0] - '0';  // Convert char to int
                current_mode = mode;
                stop_all_motors();  // Stop all motors when switching modes
                
                switch (current_mode) {
                    case MODE_MOTOR1:
                        printf("\n*** MOTOR 1 SELECTED ***\n");
                        printf("Use 'a' (left) and 'd' (right) to control Motor 1\n");
                        printf("Motor 1 ready. Press keys to move: ");
                        break;
                    case MODE_MOTOR2:
                        printf("\n*** MOTOR 2 SELECTED ***\n");
                        printf("Use 'a' (left) and 'd' (right) to control Motor 2\n");
                        printf("Motor 2 ready. Press keys to move: ");
                        break;
                    case MODE_SERVO:
                        printf("\n*** SERVO SELECTED ***\n");
                        printf("Use 'w' (up) and 's' (down) to control Servo\n");
                        printf("Current servo angle: %d degrees\n", current_servo_angle);
                        printf("Servo ready. Press keys to move: ");
                        break;
                }
            }
            // Check if input is a movement command
            else if (strlen(input_buffer) == 1) {
                char ch = input_buffer[0];
                
                switch (current_mode) {
                    case MODE_MOTOR1:
                        if (ch == 'a' || ch == 'A') {
                            if (move_motor1('L')) {
                                // Keep moving while not hitting limit switch
                                sleep_ms(100);
                                stop_all_motors();
                                printf("Motor 1 stopped.\n");
                            }
                        } else if (ch == 'd' || ch == 'D') {
                            if (move_motor1('R')) {
                                // Keep moving while not hitting limit switch
                                sleep_ms(100);
                                stop_all_motors();
                                printf("Motor 1 stopped.\n");
                            }
                        } else {
                            printf("Invalid command. Use 'a' (left) or 'd' (right) for Motor 1.\n");
                        }
                        break;
                        
                    case MODE_MOTOR2:
                        if (ch == 'a' || ch == 'A') {
                            if (move_motor2('L')) {
                                // Keep moving while not hitting limit switch
                                sleep_ms(100);
                                stop_all_motors();
                                printf("Motor 2 stopped.\n");
                            }
                        } else if (ch == 'd' || ch == 'D') {
                            if (move_motor2('R')) {
                                // Keep moving while not hitting limit switch
                                sleep_ms(100);
                                stop_all_motors();
                                printf("Motor 2 stopped.\n");
                            }
                        } else {
                            printf("Invalid command. Use 'a' (left) or 'd' (right) for Motor 2.\n");
                        }
                        break;
                        
                    case MODE_SERVO:
                        if (ch == 'w' || ch == 'W') {
                            move_servo('U');
                            sleep_ms(100);  // Small delay for servo movement
                        } else if (ch == 's' || ch == 'S') {
                            move_servo('D');
                            sleep_ms(100);  // Small delay for servo movement
                        } else {
                            printf("Invalid command. Use 'w' (up) or 's' (down) for Servo.\n");
                        }
                        break;
                        
                    default:
                        printf("No actuator selected. Please enter 1, 2, or 3 first.\n");
                        break;
                }
            } else {
                printf("Invalid input. Enter:\n");
                printf("- '1', '2', or '3' to select actuator\n");
                printf("- 'a'/'d' for motors, 'w'/'s' for servo\n");
            }
            
            printf("Ready for next command: ");
        }
        
        sleep_ms(10);  // Small delay to prevent busy waiting
    }
    
    return 0;
}
