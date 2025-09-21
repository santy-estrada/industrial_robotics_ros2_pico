#include "motor/ServoMotor.h"
#include <stdio.h>

/*
 * Simple demonstration of ServoMotor class that replicates the 
 * servo functionality from mv_joints.c
 */

int main() {
    stdio_init_all();
    
    printf("=== ServoMotor Demo (replicating mv_joints.c servo) ===\n");
    
    // Create servo with same configuration as mv_joints.c
    const uint PWM_SERVO_PIN = 22;  // Same pin as mv_joints.c
    ServoMotor servo(PWM_SERVO_PIN, -5.0f, 20.0f, 977, 4883);
    
    printf("Servo initialized - replicating mv_joints.c behavior\n");
    printf("Range: -5° to 20° (same as SERVO_MIN_ANGLE/SERVO_MAX_ANGLE)\n");
    printf("PWM values: 977 to 4883 (same as in set_servo_angle function)\n");
    printf("Starting at middle position: %.2f°\n", servo.getCurrentAngle());
    
    sleep_ms(2000);
    
    // Demonstrate movement sequence similar to mv_joints.c servo control
    printf("\nDemonstrating servo movement:\n");
    
    // Move up (like 'w' command in mv_joints.c)
    printf("Moving UP (like 'w' command):\n");
    for (int i = 0; i < 5; i++) {
        float current = servo.getCurrentAngle();
        float new_angle = current + 1.0f;  // Increment by 1 degree
        
        if (servo.setAngle(new_angle)) {
            printf("  Moved to: %.1f°\n", servo.getCurrentAngle());
        } else {
            printf("  At maximum limit: %.1f°\n", servo.getCurrentAngle());
            break;
        }
        sleep_ms(500);
    }
    
    sleep_ms(1000);
    
    // Move down (like 's' command in mv_joints.c)  
    printf("\nMoving DOWN (like 's' command):\n");
    for (int i = 0; i < 10; i++) {
        float current = servo.getCurrentAngle();
        float new_angle = current - 1.0f;  // Decrement by 1 degree
        
        if (servo.setAngle(new_angle)) {
            printf("  Moved to: %.1f°\n", servo.getCurrentAngle());
        } else {
            printf("  At minimum limit: %.1f°\n", servo.getCurrentAngle());
            break;
        }
        sleep_ms(500);
    }
    
    sleep_ms(1000);
    
    // Return to middle position (7° like mv_joints.c initial position)
    printf("\nReturning to middle position (7°):\n");
    servo.setAngle(7.0f);
    printf("Final position: %.1f°\n", servo.getCurrentAngle());
    
    printf("\n=== Demo Complete ===\n");
    
    return 0;
}

/*
 * Comparison between mv_joints.c and ServoMotor class:
 * 
 * mv_joints.c implementation:
 * ---------------------------
 * #define SERVO_MIN_ANGLE -5
 * #define SERVO_MAX_ANGLE 20
 * int current_servo_angle = 7;
 * 
 * void set_servo_angle(int angle) {
 *     if (angle < SERVO_MIN_ANGLE) angle = SERVO_MIN_ANGLE;
 *     if (angle > SERVO_MAX_ANGLE) angle = SERVO_MAX_ANGLE;
 *     int pwm_value = map(angle, 0, 180, 977, 4883);
 *     pwm_set_chan_level(slice_num, channel, pwm_value);
 *     current_servo_angle = angle;
 * }
 * 
 * ServoMotor class equivalent:
 * ----------------------------
 * ServoMotor servo(PWM_PIN, -5.0f, 20.0f, 977, 4883);
 * 
 * servo.setAngle(7.0f);              // Same as set_servo_angle(7)
 * float angle = servo.getCurrentAngle(); // Same as current_servo_angle
 * 
 * Advantages of ServoMotor class:
 * - Automatic limit checking
 * - Float precision for angles
 * - Multiple servo support
 * - Built-in PWM initialization
 * - Consistent interface with other motor classes
 * - Runtime reconfiguration capability
 */