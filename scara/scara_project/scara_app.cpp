#include "motor/PrecisionMotor.h"
#include "motor/ServoMotor.h"
#include "sensor/LimitSwitch.h"
#include "scara/Joint.h"
#include "scara/ServoJoint.h"
#include "scara/ScaraRobot.h"
#include <stdio.h>

// Test configuration defines - comment/uncomment to enable/disable tests
// NOTE: TEST_FULL_SCARA is mutually exclusive with individual joint tests
// #define TEST_FULL_SCARA
// #define TEST_JOINT1
#define TEST_JOINT2  
// #define TEST_JOINT3

// SCARA Robot configuration
#define LED_PIN 25

// Joint configuration defines - Updated to match tested hardware pins
#define JOINT1_MOTOR_ENA 16    // PWMA_1 from C code
#define JOINT1_MOTOR_IN1 17    // INA1_1 from C code  
#define JOINT1_MOTOR_IN2 18    // INA2_1 from C code
#define JOINT1_ENCODER_A 13    // CHA_M1 from C code
#define JOINT1_ENCODER_B 12    // CHB_M1 from C code
#define JOINT1_LIMIT_MIN 7     // FC_1L from C code
#define JOINT1_LIMIT_MAX 6     // FC_1R from C code

#define JOINT2_MOTOR_ENA 21    // PWMB_1 from C code
#define JOINT2_MOTOR_IN1 19    // INB1_1 from C code
#define JOINT2_MOTOR_IN2 20    // INB2_1 from C code
#define JOINT2_ENCODER_A 15    // CHA_M2 from C code
#define JOINT2_ENCODER_B 14    // CHB_M2 from C code
#define JOINT2_LIMIT_MIN 8     // FC_2L from C code
#define JOINT2_LIMIT_MAX 9     // FC_2R from C code

#define JOINT3_SERVO_PWM 22    // PWM_SERVO from C code (servo motor)

#define SAFETY_PENDANT_PIN 28  // BTN from C code (safety pendant/dead man's switch)

/*
 * SCARA Robot Test Application
 * 
 * This application supports two mutually exclusive test modes:
 *
 * === FULL SCARA MODE (TEST_FULL_SCARA) ===
 * Complete 3-DOF SCARA robot with unified control using ScaraRobot class:
 * - Joint 1: Revolute, ±90°, 4:1 gear ratio, limit switches, precision motor
 * - Joint 2: Revolute, ±120°, 4:1 gear ratio, limit switches, precision motor  
 * - Joint 3: Servo motor, 0°-180° range, 0.5:1 gear ratio, no limit switches
 * - Safety Pendant: Dead man's switch on pin 28 - robot only moves when pressed
 * 
 * Features:
 * - Unified calibration (joint2 then joint1, servo origin setting)
 * - Coordinated motion commands (moveToConfiguration)
 * - Safety pendant prevents movement when not pressed (motors stop, encoders still read)
 * - Comprehensive status reporting including pendant status
 * - 5-phase test sequence (15 second intervals)
 * 
 * === INDIVIDUAL JOINT MODE (TEST_JOINTx) ===
 * Individual joint testing for development and debugging:
 * - TEST_JOINT1: Test revolute joint 1 only
 * - TEST_JOINT2: Test revolute joint 2 only  
 * - TEST_JOINT3: Test servo joint 3 only
 * - No safety pendant in individual mode
 * - 4-phase test sequence (10 second intervals)
 * 
 * Usage: Comment/uncomment the defines below to select test mode.
 * NOTE: TEST_FULL_SCARA takes precedence over individual joint tests.
 */

// SCARA Robot Application
int main() {
    stdio_init_all();
    
    // Initialize LED pin
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1); // Turn on LED to indicate startup
    
#ifdef TEST_FULL_SCARA
    printf("=== FULL SCARA Robot Test Mode ===\n");
    
    // Create the complete SCARA robot
    ScaraRobot scara_robot(
        // Joint 1 pins
        JOINT1_MOTOR_ENA, JOINT1_MOTOR_IN1, JOINT1_MOTOR_IN2,
        JOINT1_ENCODER_A, JOINT1_ENCODER_B, JOINT1_LIMIT_MIN, JOINT1_LIMIT_MAX,
        // Joint 2 pins
        JOINT2_MOTOR_ENA, JOINT2_MOTOR_IN1, JOINT2_MOTOR_IN2,
        JOINT2_ENCODER_A, JOINT2_ENCODER_B, JOINT2_LIMIT_MIN, JOINT2_LIMIT_MAX,
        // Joint 3 pins
        JOINT3_SERVO_PWM
        // Safety pendant pin
        // SAFETY_PENDANT_PIN       // Enable by uncommenting this line (add a comma after JOINT3_SERVO_PWM)
    );
    
    sleep_ms(5000);
    
    // Calibrate the robot
    printf("=== Calibrating SCARA Robot ===\n");
    scara_robot.calibrateRevoluteJoints();  // Calibrates joint2 then joint1
    scara_robot.calibrateServoJoint(90.0f); // Set servo 90° as joint origin (0°)
    
    printf("\n=== SCARA Control Loop Starting ===\n");
    
    // Control loop timing
    uint64_t last_time_fast = to_ms_since_boot(get_absolute_time());    // Fast loop: 50ms
    uint64_t last_time_status = to_ms_since_boot(get_absolute_time());  // Status: 1000ms
    
    // Test sequence variables
    uint64_t test_sequence_time = to_ms_since_boot(get_absolute_time());
    int test_phase = 0;
    
    while (true) {
        uint64_t current_time = to_ms_since_boot(get_absolute_time());
        
        // FAST CONTROL LOOP - 50ms (20Hz)
        if (current_time - last_time_fast >= 50) {
            // Update all joint states
            scara_robot.updateJoints();
            last_time_fast = current_time;
        }
        
        // STATUS DISPLAY - 1000ms (1Hz)  
        if (current_time - last_time_status >= 1000) {
            scara_robot.printStatus();
            last_time_status = current_time;
        }
        
        // TEST SEQUENCE - Move robot through predefined configurations
        if (current_time - test_sequence_time >= 15000) {  // Change every 15 seconds
            
            switch (test_phase) {
                case 0:
                    printf("\n=== SCARA Test Phase 1: Home Configuration ===\n");
                    scara_robot.moveToConfiguration(0.0f, 0.0f, 0.0f);
                    break;
                    
                case 1:
                    printf("\n=== SCARA Test Phase 2: Extended Configuration ===\n");
                    scara_robot.moveToConfiguration(45.0f, 30.0f, 60.0f);
                    break;
                    
                case 2:
                    printf("\n=== SCARA Test Phase 3: Retracted Configuration ===\n");
                    scara_robot.moveToConfiguration(-30.0f, -45.0f, -60.0f);
                    break;
                    
                case 3:
                    printf("\n=== SCARA Test Phase 4: Mixed Configuration ===\n");
                    scara_robot.moveToConfiguration(60.0f, -60.0f, 45.0f);
                    break;
                    
                case 4:
                    printf("\n=== SCARA Test Phase 5: Return to Home ===\n");
                    scara_robot.moveToConfiguration(0.0f, 0.0f, 0.0f);
                    break;

                case 5:
                    printf("\n=== SCARA Test Phase 6: Move only Joint 1 to 30°===\n");
                    scara_robot.setJoint1(30.0f);  // Move joint 1 to 30°
                    break;
                
                case 6:
                    printf("\n=== SCARA Test Phase 7: Move only Joint1 to 60°===\n");
                    scara_robot.setJoint1(60.0f);  // Move joint 1 to 60°
                    break;
                
                case 7:
                    printf("\n=== SCARA Test Phase 8: Move only Joint 1 to 75°===\n");
                    scara_robot.setJoint1(75.0f);  // Move joint 1 to 75°
                    break;
                
                case 8:
                    printf("\n=== SCARA Test Phase 9: Move only Joint 1 to -75°===\n");
                    scara_robot.setJoint1(-75.0f);  // Move joint 1 to -75°
                    break;
                
                case 9:
                    printf("\n=== SCARA Test Phase 10: Move only Joint 1 to -45°===\n");
                    scara_robot.setJoint1(-45.0f);  // Move joint 1 to -45°
                    break;

                case 10:
                    printf("\n=== SCARA Test Phase 11: Move only Joint 1 to -15°===\n");
                    scara_robot.setJoint1(-15.0f);  // Move joint 1 to -15°
                    break;

                case 11:
                    printf("\n=== SCARA Test Phase 12: Move only Joint 1 to 0°===\n");
                    scara_robot.setJoint1(0.0f);  // Move joint 1 to 0°
                    break;
                    
                default:
                    printf("\n=== SCARA Test Sequence Complete - Continuous Operation ===\n");
                    test_phase = -1; // Reset to -1 since it will be incremented to 0 at end of switch
                    break;
            }
            
            test_phase++;
            test_sequence_time = current_time;
        }
        
        // Small sleep to prevent excessive CPU usage
        sleep_ms(10);
    }

#else
    printf("=== Individual Joint Test Mode ===\n");
    
#ifdef TEST_JOINT1
    // Create precision motor for joint 1
    printf("Creating Joint 1 precision motor...\n");
    PrecisionMotor joint1_motor(JOINT1_MOTOR_ENA, JOINT1_MOTOR_IN1, JOINT1_MOTOR_IN2,
                               JOINT1_ENCODER_A, JOINT1_ENCODER_B, 
                               64, 50.0f, 0.05f, 0.2332f, 0.35f, 0.0000125f);  // 64 ticks/rev, 50:1 gear ratio
    
    // Create limit switches for joint 1
    printf("Creating Joint 1 limit switches...\n");
    LimitSwitch joint1_limit_min(JOINT1_LIMIT_MIN);
    LimitSwitch joint1_limit_max(JOINT1_LIMIT_MAX);
    
    // Create SCARA joint 1
    printf("Creating SCARA Joint 1...\n");
    Joint joint1('R', -85.0f, 85.0f, &joint1_motor, 4.0f, 
                 &joint1_limit_min, &joint1_limit_max, 1.0f);  // Joint 1: ±85° with 4:1 joint ratio
#endif

#ifdef TEST_JOINT2
    // Create precision motor for joint 2
    printf("Creating Joint 2 precision motor...\n");
    PrecisionMotor joint2_motor(JOINT2_MOTOR_ENA, JOINT2_MOTOR_IN1, JOINT2_MOTOR_IN2,
                               JOINT2_ENCODER_A, JOINT2_ENCODER_B, 
                               64, 50.0f, 0.025f, 0.3f, 0.8f, 0.0000125f,
                            10.0f, 5.2f, 0.001f);   // 64 ticks/rev, 50:1 gear ratio
    
    // Create limit switches for joint 2
    printf("Creating Joint 2 limit switches...\n");
    LimitSwitch joint2_limit_min(JOINT2_LIMIT_MIN);
    LimitSwitch joint2_limit_max(JOINT2_LIMIT_MAX);
    
    // Create SCARA joint 2
    printf("Creating SCARA Joint 2...\n");
    Joint joint2('R', -120.0f, 120.0f, &joint2_motor, 4.0f, 
                 &joint2_limit_min, &joint2_limit_max,  1.0f);   // Joint 2: ±120° with 4:1 joint ratio
#endif

#ifdef TEST_JOINT3
    // Create servo motor and servo joint for joint 3
    printf("Creating Joint 3 servo motor and servo joint...\n");
    ServoMotor joint3_servo_motor(JOINT3_SERVO_PWM);  // Using default values (0° to 110°)
    ServoJoint joint3('P', -50.0f, 50.0f, &joint3_servo_motor, -1.0f);  // Prismatic joint, -50 to 50 mm, 1:-1 gear ratio
#endif
    
    printf("Individual joints initialized successfully!\n");
    
    sleep_ms(5000);
    
    printf("Setting joint origins...\n");
#ifdef TEST_JOINT1
    joint1.calibrateOrigin();  // Set current position as 0°
#endif
#ifdef TEST_JOINT2
    joint2.calibrateOrigin();  // Set current position as 0°
#endif
#ifdef TEST_JOINT3
    joint3.setOriginAt(55.0f);  // Set servo angle 55° as joint origin (0°)
#endif
    
    printf("\n=== Individual Joint Control Loop Starting ===\n");
    
    // Control loop timing
    uint64_t last_time_fast = to_ms_since_boot(get_absolute_time());    // Fast loop: 50ms
    uint64_t last_time_status = to_ms_since_boot(get_absolute_time());  // Status: 1000ms
    
    // Test sequence variables
    uint64_t test_sequence_time = to_ms_since_boot(get_absolute_time());
    int test_phase = 0;
    
    while (true) {
        uint64_t current_time = to_ms_since_boot(get_absolute_time());
        
        // FAST CONTROL LOOP - 50ms (20Hz)
        if (current_time - last_time_fast >= 25) {
            
            // Update joint states (this will update position, speed, and run control)
#ifdef TEST_JOINT1
            joint1.set_joint();
#endif
#ifdef TEST_JOINT2
            joint2.set_joint();
#endif
            
            last_time_fast = current_time;
        }

        // STATUS DISPLAY - 100ms (10Hz)
        if (current_time - last_time_status >= 200) {

            printf("\n=== Individual Joint Status ===\n");
#ifdef TEST_JOINT1
            printf("Joint1: Pos=%.2f°, Speed=%.2f°/s, Target=%.2f°, Error=%.2f°\n",
                   joint1.getCurrentPosition(), joint1.getCurrentSpeed(), joint1.getDesiredPosition(), joint1.getErrorPosition());
#endif
#ifdef TEST_JOINT2
            printf("Joint2: Pos=%.2f°, Speed=%.2f°/s, Target=%.2f°, Error=%.2f°\n",
                   joint2.getCurrentPosition(), joint2.getCurrentSpeed(), joint2.getDesiredPosition(), joint2.getErrorPosition());
#endif
#ifdef TEST_JOINT3
            printf("Joint3 (ServoJoint): Pos=%.2f°, Range=[%.1f°, %.1f°]\n",
                   joint3.getCurrentPosition(), joint3.getMinLimit(), joint3.getMaxLimit());
#endif
            // Check safety status
#ifdef TEST_JOINT1
            if (!joint1.isWithinLimits()) {
                printf("WARNING: Joint1 outside limits!\n");
            }
#endif
#ifdef TEST_JOINT2
            if (!joint2.isWithinLimits()) {
                printf("WARNING: Joint2 outside limits!\n");
            }
#endif
            
            last_time_status = current_time;
        }
        
        // TEST SEQUENCE - Move joints through predefined positions
        if (current_time - test_sequence_time >= 10000) {  // Change every 10 seconds
            
            switch (test_phase) {
                case 0:
                    printf("\n=== Test Phase 1: Home Position ===\n");
#ifdef TEST_JOINT1
                    joint1.set_joint(0.0f);
#endif
#ifdef TEST_JOINT2
                    joint2.set_joint(0.0f);
#endif
#ifdef TEST_JOINT3
                    joint3.setPosition(0.0f);
#endif
                    break;
                    
                case 1:
                    printf("\n=== Test Phase 2: Extended Position ===\n");
#ifdef TEST_JOINT1
                    joint1.set_joint(45.0f);
#endif
#ifdef TEST_JOINT2
                    joint2.set_joint(100.0f);
#endif
#ifdef TEST_JOINT3
                    joint3.setPosition(-10.0f);
#endif
                    break;
                    
                case 2:
                    printf("\n=== Test Phase 3: Retracted Position ===\n");
#ifdef TEST_JOINT1
                    joint1.set_joint(-30.0f);
#endif
#ifdef TEST_JOINT2
                    joint2.set_joint(-100.0f);
#endif
#ifdef TEST_JOINT3
                    joint3.setPosition(-50.0f);
#endif
                    break;
                    
                case 3:
                    printf("\n=== Test Phase 4: Return to Home ===\n");
#ifdef TEST_JOINT1
                    joint1.set_joint(0.0f);
#endif
#ifdef TEST_JOINT2
                    joint2.set_joint(0.0f);
#endif
#ifdef TEST_JOINT3
                    joint3.setPosition(30.0f);
#endif
                    break;
                    
                default:
                    printf("\n=== Test Sequence Complete - Continuous Operation ===\n");
                    test_phase = -1; // Reset to -1 since it will be incremented to 0 at end of switch
                    break;
            }
            
            test_phase++;
            test_sequence_time = current_time;
        }
        
        // Small sleep to prevent excessive CPU usage
        sleep_ms(10);
    }
#endif
    
    return 0;
}