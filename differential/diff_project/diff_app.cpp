#include "motor/PrecisionMotor.h"
#include "sensor/ultrasonic_sensor.hpp"
#include "sensor/mpu9250.hpp"
#include "sensor/bmi160.hpp"
#include "sensor/madgwick_filter.hpp"
#include "robot/DiffDrive.h"
#include <stdio.h>
#include <hardware/i2c.h>

// Test configuration defines - comment/uncomment to enable/disable tests
// NOTE: Only enable one test at a time
// #define TEST_PRECISION_MOTORS  // Test 1: Two precision motors with PID control
// #define TEST_IMU_MPU          // Test 2: MPU9250 IMU with Madgwick filter
// #define TEST_IMU_BMI          // Test 3: BMI160 IMU with Madgwick filter
// #define TEST_ULTRASONIC       // Test 4: HC-SR05 ultrasonic sensor (both libraries)
// #define TEST_OBSTACLE_AVOID   // Test 5: Motors + ultrasonic obstacle avoidance
#define TEST_DIFF_DRIVE       // Test 6: Complete differential drive robot class

// LED Pin for status indication
#define LED_PIN 25

// === DIFFERENTIAL DRIVE MOTOR PINS ===
// Left Motor (Motor 1) - GPIO pins 16, 17, 18 + encoders 13, 12
#define MOTOR_L_ENA  16  // PWM enable pin for left motor
#define MOTOR_L_IN1  18  // Direction pin 1 for left motor
#define MOTOR_L_IN2  17  // Direction pin 2 for left motor
#define ENCODER_L_A  13  // Encoder A pin for left motor
#define ENCODER_L_B  12  // Encoder B pin for left motor

// Right Motor (Motor 2) - GPIO pins 21, 19, 20 + encoders 15, 14
#define MOTOR_R_ENA  21  // PWM enable pin for right motor
#define MOTOR_R_IN1  19  // Direction pin 1 for right motor
#define MOTOR_R_IN2  20  // Direction pin 2 for right motor
#define ENCODER_R_A  14  // Encoder A pin for right motor
#define ENCODER_R_B  15  // Encoder B pin for right motor

// === IMU PINS ===
#define IMU_SDA      10  // I2C SDA pin for IMU
#define IMU_SCL      11  // I2C SCL pin for IMU

// === ULTRASONIC SENSOR PINS ===
#define ULTRASONIC_TRIG  27  // Trigger pin for ultrasonic sensor
#define ULTRASONIC_ECHO  26  // Echo pin for ultrasonic sensor

// === MOTOR PARAMETERS ===
#define ENCODER_TICKS_PER_REV  28    // Encoder ticks per motor revolution
#define GEAR_RATIO             150.0f // 150:1 gear ratio
#define MAX_SPEED_RPM          200.0f // Maximum speed in RPM

// === PID CONTROLLER PARAMETERS ===
#define PID_KP  0.9696f    // Proportional gain
#define PID_TI  0.6f       // Integral time constant
#define PID_TD  2.5e-05f   // Derivative time constant
#define dt     0.1f       // Control loop time interval in seconds

/**
 * === DIFFERENTIAL DRIVE ROBOT TEST SUITE ===
 * 
 * This application supports six mutually exclusive test modes:
 *
 * === TEST 1: PRECISION MOTORS (TEST_PRECISION_MOTORS) ===
 * Tests two precision motors with individual PID control:
 * - Motor 1 (Left): GPIO 16,17,18 + encoders 13,12
 * - Motor 2 (Right): GPIO 21,19,20 + encoders 15,14
 * - Speed cycle: 0→50→100→150→0 RPM
 * - Prints: desired speed, current speed, control law, error
 *
 * === TEST 2: MPU9250 IMU SENSOR (TEST_IMU_MPU) ===
 * Tests MPU9250 IMU with Madgwick filter:
 * - I2C on pins 10 (SDA) and 11 (SCL)
 * - Prints: accelerometer, gyroscope, magnetometer, quaternion
 *
 * === TEST 3: BMI160 IMU SENSOR (TEST_IMU_BMI) ===
 * Tests BMI160 IMU with Madgwick filter:
 * - I2C on pins 10 (SDA) and 11 (SCL)
 * - Prints: accelerometer, gyroscope, quaternion
 *
 * === TEST 4: ULTRASONIC SENSOR (TEST_ULTRASONIC) ===  
 * Tests HC-SR05 ultrasonic sensor with UltrasonicSensor library:
 * - Uses ultrasonic_sensor.cpp library
 * - Trig pin: GPIO 27, Echo pin: GPIO 26
 * - Prints: distance in cm
 *
 * === TEST 5: OBSTACLE AVOIDANCE (TEST_OBSTACLE_AVOID) ===
 * Combined motors + ultrasonic for obstacle avoidance:
 * - Both motors move forward at 100 RPM
 * - Stop when obstacle ≤ 10cm detected
 * - Resume when obstacle cleared
 * - Same motor/sensor parameters as individual tests
 *
 * === TEST 6: DIFFERENTIAL DRIVE ROBOT (TEST_DIFF_DRIVE) ===
 * Complete differential drive robot class with integrated sensors:
 * - DiffDrive class with 2 precision motors, BMI160 IMU, ultrasonic sensor
 * - Speed ramp test: 0→20→40→60→80→100 RPM (10 seconds each)
 * - Automatic obstacle detection and emergency stop
 * - Prints: motor status, sensor readings, warnings, emergency stops
 */

// Differential Drive Robot Application
int main() {
    stdio_init_all();
    
    // Initialize LED pin
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1); // Turn on LED to indicate startup
    
    printf("=== Differential Drive Robot Test Suite ===\n");
    
#ifdef TEST_PRECISION_MOTORS
    printf("=== TEST 1: PRECISION MOTORS ===\n");
    
    // Create two precision motors for differential drive
    printf("Creating precision motors...\n");
    PrecisionMotor motor_left(MOTOR_L_ENA, MOTOR_L_IN1, MOTOR_L_IN2,
                             ENCODER_L_A, ENCODER_L_B, 
                             ENCODER_TICKS_PER_REV, GEAR_RATIO, dt,
                             PID_KP, PID_TI, PID_TD);
    
    PrecisionMotor motor_right(MOTOR_R_ENA, MOTOR_R_IN1, MOTOR_R_IN2,
                              ENCODER_R_A, ENCODER_R_B, 
                              ENCODER_TICKS_PER_REV, GEAR_RATIO, dt,
                              PID_KP, PID_TI, PID_TD);
    
    sleep_ms(2000);
    printf("Motors initialized successfully!\n");
    
    // Speed cycle array
    float speed_targets[] = {0.0f, 50.0f, 100.0f, 150.0f, 0.0f};
    int num_speeds = sizeof(speed_targets) / sizeof(speed_targets[0]);
    
    // Control loop timing
    uint64_t last_time_control = to_ms_since_boot(get_absolute_time());
    uint64_t last_time_status = to_ms_since_boot(get_absolute_time());
    uint64_t speed_change_time = to_ms_since_boot(get_absolute_time());
    
    int current_speed_index = 0;
    
    while (true) {
        uint64_t current_time = to_ms_since_boot(get_absolute_time());
        
        // CONTROL LOOP - 100ms (10Hz)
        if (current_time - last_time_control >= dt*1000) {
            // Run PID control for both motors - this updates the motor speeds
            // based on current encoder feedback and maintains the setpoint
            motor_left.set_motor(motor_left.get_setpoint());
            motor_right.set_motor(motor_right.get_setpoint());
            last_time_control = current_time;
        }
        
        // STATUS DISPLAY - 1000ms (1Hz)
        if (current_time - last_time_status >= 1000) {
            // Left motor status
            float left_desired = motor_left.get_setpoint();
            float left_current = motor_left.get_filtered_rpm();
            float left_control = motor_left.get_control_output();
            float left_error = motor_left.get_error();
            
            // Right motor status
            float right_desired = motor_right.get_setpoint();
            float right_current = motor_right.get_filtered_rpm();
            float right_control = motor_right.get_control_output();
            float right_error = motor_right.get_error();
            
            printf("\n=== Motor Status ===\n");
            printf("LEFT  Motor: Desired=%.1f rpm, Current=%.1f rpm, Control=%.3f, Error=%.1f rpm\n",
                   left_desired, left_current, left_control, left_error);
            printf("RIGHT Motor: Desired=%.1f rpm, Current=%.1f rpm, Control=%.3f, Error=%.1f rpm\n",
                   right_desired, right_current, right_control, right_error);
            
            last_time_status = current_time;
        }
        
        // SPEED CHANGE - 5000ms (5 seconds per speed)
        if (current_time - speed_change_time >= 5000) {
            float target_speed_L = speed_targets[current_speed_index];
            
            printf("\n=== Setting motors to %.0f RPM ===\n", target_speed_L);
            motor_left.set_motor(target_speed_L);
            motor_right.set_motor(target_speed_L);

            printf("\n=== Motor Status ===\n");
            printf("LEFT  Motor: Desired=%.1f rpm, Current=%.1f rpm, Control=%.3f, Error=%.1f rpm\n",
                   motor_left.get_setpoint(), motor_left.get_filtered_rpm(), motor_left.get_control_output(), motor_left.get_error());
            printf("RIGHT Motor: Desired=%.1f rpm, Current=%.1f rpm, Control=%.3f, Error=%.1f rpm\n",
                   motor_right.get_setpoint(), motor_right.get_filtered_rpm(), motor_right.get_control_output(), motor_right.get_error());

            current_speed_index = (current_speed_index + 1) % num_speeds;
            speed_change_time = current_time;
        }
        
        sleep_ms(10);
    }

#elif defined(TEST_IMU_MPU)
    printf("=== TEST 2: MPU9250 IMU SENSOR ===\n");
    
    // Create IMU and filter using the same pattern as example code
    printf("Creating MPU9250 IMU...\n");
    MPU9250 imu(i2c1, IMU_SDA, IMU_SCL);  // Use i2c1
    MadgwickFilter filter;
    
    sleep_ms(2000);
    
    // Initialize IMU using init() method like example
    if (!imu.init()) {
        printf("ERROR: Failed to initialize IMU!\n");
        return -1;
    }
    
    printf("IMU initialized successfully!\n");
    
    uint64_t last_time_imu = to_ms_since_boot(get_absolute_time());
    
    while (true) {
        uint64_t current_time = to_ms_since_boot(get_absolute_time());
        
        // IMU UPDATE - 100ms (10Hz)
        if (current_time - last_time_imu >= dt*1000) {
            // Read sensor data using read_accel_gyro() method like example
            if (imu.read_accel_gyro()) {
                // Calculate delta time for filter
                static uint64_t last_filter_time = current_time;
                last_filter_time = current_time;
                
                // Update filter using direct access to imu data like example
                filter.update(imu.gx, imu.gy, imu.gz, imu.ax, imu.ay, imu.az, dt);
                
                // Get quaternion
                float x, y, z, w;
                filter.getQuaternion(x, y, z, w);
                
                // Print all data using direct access like example
                printf("\n=== IMU Data ===\n");
                printf("Accel: X=%.3f, Y=%.3f, Z=%.3f m/s²\n", imu.ax, imu.ay, imu.az);
                printf("Gyro:  X=%.3f, Y=%.3f, Z=%.3f rad/s\n", imu.gx, imu.gy, imu.gz);
                printf("Quat:  X=%.3f, Y=%.3f, Z=%.3f, W=%.3f\n", x, y, z, w);
            } else {
                printf("ERROR: Failed to read IMU data!\n");
            }
            
            last_time_imu = current_time;
        }
        
        sleep_ms(10);
    }

#elif defined(TEST_IMU_BMI)
    printf("=== TEST 3: BMI160 IMU SENSOR ===\n");
    
    // Create BMI160 IMU and filter
    printf("Creating BMI160 IMU...\n");
    BMI160 imu;
    MadgwickFilter filter;
    
    sleep_ms(2000);
    
    // Initialize BMI160 using I2C (same pins as MPU9250)
    if (!imu.begin(BMI160::I2C_MODE, i2c1, IMU_SDA, IMU_SCL)) {
        printf("ERROR: Failed to initialize BMI160!\n");
        return -1;
    }
    
    printf("BMI160 initialized successfully!\n");
    printf("Chip ID: 0x%02X\n", imu.getChipID());
    
    uint32_t last_time_imu = time_us_32() / 1000;
    uint32_t last_filter_time = last_time_imu;
    uint32_t start_time = last_time_imu;
    
    printf("Starting 10-second BMI160 test...\n");
    
    while (true) {
        uint32_t current_time = time_us_32() / 1000;
        
        // IMU UPDATE - 100ms (10Hz)
        if (current_time - last_time_imu >= dt*1000) {
            // Read sensor data using read_accel_gyro() method
            if (imu.read_accel_gyro()) {
                // Calculate delta time for filter
                last_filter_time = current_time;
                
                // Update filter using direct access to imu data
                filter.update(imu.gx, imu.gy, imu.gz, imu.ax, imu.ay, imu.az, dt);
                
                // Get quaternion
                float x, y, z, w;
                filter.getQuaternion(x, y, z, w);
                
                printf("\n=== BMI160 Data ===\n");
                printf("Accel: X=%.3f, Y=%.3f, Z=%.3f m/s²\n", imu.ax, imu.ay, imu.az);
                printf("Gyro:  X=%.3f, Y=%.3f, Z=%.3f rad/s\n", imu.gx, imu.gy, imu.gz);
                printf("Quat:  X=%.3f, Y=%.3f, Z=%.3f, W=%.3f\n", x, y, z, w);
            } else {
                printf("ERROR: Failed to read BMI160 data!\n");
            }
            
            last_time_imu = current_time;
        }
        
        sleep_ms(10);
    }

#elif defined(TEST_ULTRASONIC)
    printf("=== TEST 3: ULTRASONIC SENSOR ===\n");
    
    // Create ultrasonic sensor
    printf("Creating UltrasonicSensor...\n");
    UltrasonicSensor ultrasonic_sensor(ULTRASONIC_TRIG, ULTRASONIC_ECHO);
    
    sleep_ms(2000);
    printf("Ultrasonic sensor initialized successfully!\n");
    
    uint64_t last_time_test = to_ms_since_boot(get_absolute_time());
    
    while (true) {
        uint64_t current_time = to_ms_since_boot(get_absolute_time());
        
        // SENSOR TEST - 1000ms (1Hz)
        if (current_time - last_time_test >= 1000) {
            printf("\n=== UltrasonicSensor Reading ===\n");
            
            float distance;
            if (ultrasonic_sensor.measure_distance(distance)) {
                // Convert from meters to centimeters for readability
                printf("Distance: %.2f cm\n", distance * 100.0f);
            } else {
                printf("Failed to read distance\n");
            }
            
            last_time_test = current_time;
        }
        
        sleep_ms(10);
    }

#elif defined(TEST_OBSTACLE_AVOID)
    printf("=== TEST 4: OBSTACLE AVOIDANCE ===\n");
    
    // Create motors and ultrasonic sensor
    printf("Creating motors and ultrasonic sensor...\n");
    PrecisionMotor motor_left(MOTOR_L_ENA, MOTOR_L_IN1, MOTOR_L_IN2,
                             ENCODER_L_A, ENCODER_L_B, 
                             ENCODER_TICKS_PER_REV, GEAR_RATIO, dt,
                             PID_KP, PID_TI, PID_TD);
    
    PrecisionMotor motor_right(MOTOR_R_ENA, MOTOR_R_IN1, MOTOR_R_IN2,
                              ENCODER_R_A, ENCODER_R_B, 
                              ENCODER_TICKS_PER_REV, GEAR_RATIO, dt,
                              PID_KP, PID_TI, PID_TD, 0.995f); // Slightly different correction factor for right motor
    
    UltrasonicSensor ultrasonic(ULTRASONIC_TRIG, ULTRASONIC_ECHO);
    
    sleep_ms(2000);
    printf("Motors and ultrasonic sensor initialized successfully!\n");
    
    // Control loop timing
    uint64_t last_time_control = to_ms_since_boot(get_absolute_time());
    uint64_t last_time_ultrasonic = to_ms_since_boot(get_absolute_time());
    uint64_t last_time_status = to_ms_since_boot(get_absolute_time());
    
    bool moving = false;
    const float OBSTACLE_THRESHOLD = 10.0f; // cm
    const float TARGET_SPEED_L = 50.0f; // rpm
    const float TARGET_SPEED_R = 50.0f; // rpm

    while (true) {
        uint64_t current_time = to_ms_since_boot(get_absolute_time());
        
        // MOTOR CONTROL - 100ms (10Hz)
        if (current_time - last_time_control >= dt*1000) {
            // Run PID control for both motors - this updates the motor speeds
            // based on current encoder feedback and maintains the setpoint
            if (moving) {
                motor_left.set_motor(TARGET_SPEED_L);
                motor_right.set_motor(TARGET_SPEED_R);
            } else {
                motor_left.set_motor(0.0f);
                motor_right.set_motor(0.0f);
                motor_left.stop();
                motor_right.stop();
            }
            last_time_control = current_time;
        }
        
        // ULTRASONIC SENSOR - 300ms (3.33Hz)
        if (current_time - last_time_ultrasonic >= 300) {
            float distance_m;
            float distance = 0.0f; // Distance in cm
            
            if (ultrasonic.measure_distance(distance_m)) {
                distance = distance_m * 100.0f; // Convert meters to cm
            }
            
            if (distance > 0 && distance <= OBSTACLE_THRESHOLD) {
                // Obstacle detected - stop
                if (moving) {
                    printf("\n=== OBSTACLE DETECTED at %.1f cm - STOPPING ===\n", distance);
                    motor_left.set_motor(0.0f);
                    motor_right.set_motor(0.0f);
                    motor_left.stop();
                    motor_right.stop();
                    moving = false;
                }
            } else if (distance > OBSTACLE_THRESHOLD || distance <= 0) {
                // Path clear - move forward
                if (!moving) {
                    printf("\n=== PATH CLEAR - MOVING FORWARD at %.0f RPM ===\n", TARGET_SPEED_L);
                    motor_left.set_motor(TARGET_SPEED_L);
                    motor_right.set_motor(TARGET_SPEED_R);
                    moving = true;
                }
            }
            
            last_time_ultrasonic = current_time;
        }
        
        // STATUS DISPLAY - 1000ms (1Hz)
        if (current_time - last_time_status >= 1000) {
            float distance_m;
            float distance = 0.0f; // Distance in cm
            
            if (ultrasonic.measure_distance(distance_m)) {
                distance = distance_m * 100.0f; // Convert meters to cm
            }
            
            printf("Status: Distance=%.1f cm, Left=%.1f rpm, Right=%.1f rpm, Moving=%s\n",
                   distance, motor_left.get_filtered_rpm(), motor_right.get_filtered_rpm(),
                   moving ? "YES" : "NO");
            last_time_status = current_time;
        }
        
        sleep_ms(10);
    }

#elif defined(TEST_DIFF_DRIVE)
    printf("=== TEST 6: DIFFERENTIAL DRIVE ROBOT ===\n");
    
    // Create differential drive robot
    printf("Creating DiffDrive robot...\n");
    DiffDrive robot(MOTOR_L_ENA, MOTOR_L_IN1, MOTOR_L_IN2,
                   ENCODER_L_A, ENCODER_L_B,
                   MOTOR_R_ENA, MOTOR_R_IN1, MOTOR_R_IN2,
                   ENCODER_R_A, ENCODER_R_B,
                   ULTRASONIC_TRIG, ULTRASONIC_ECHO,
                   IMU_SDA, IMU_SCL, i2c1,
                   ENCODER_TICKS_PER_REV, GEAR_RATIO, dt,
                   PID_KP, PID_TI, PID_TD);
    
    sleep_ms(2000);
    
    // Initialize robot
    if (!robot.init()) {
        printf("ERROR: Failed to initialize DiffDrive robot!\n");
        return -1;
    }
    
    printf("DiffDrive robot initialized successfully!\n");
    
    // Speed ramp array: 0 -> 20 -> 40 -> 60 -> 80 -> 100 RPM
    float speed_targets[] = {0.0f, 20.0f, 40.0f, 60.0f, 80.0f, 100.0f, 0.0f};
    int num_speeds = sizeof(speed_targets) / sizeof(speed_targets[0]);
    
    // Control loop timing
    uint64_t last_time_control = to_ms_since_boot(get_absolute_time());
    uint64_t last_time_status = to_ms_since_boot(get_absolute_time());
    uint64_t speed_change_time = to_ms_since_boot(get_absolute_time());
    
    int current_speed_index = 0;
    bool last_warning_state = false;
    bool last_emergency_state = false;
    
    printf("Starting speed ramp test: 0->20->40->60->80->100->0 RPM (10 seconds each)\n");
    
    while (true) {
        uint64_t current_time = to_ms_since_boot(get_absolute_time());
        
        // ROBOT UPDATE - 100ms (10Hz)
        if (current_time - last_time_control >= dt*1000) {
            // Update robot state (motor control, sensor reading, safety checks)
            robot.update();
            last_time_control = current_time;
        }
        
        // STATUS DISPLAY - 1000ms (1Hz)
        if (current_time - last_time_status >= 1000) {
            // Motor status
            float left_desired = robot.get_left_desired_speed();
            float left_current = robot.get_left_current_speed();
            float left_error = robot.get_left_error();
            float left_power = robot.get_left_power();
            
            float right_desired = robot.get_right_desired_speed();
            float right_current = robot.get_right_current_speed();
            float right_error = robot.get_right_error();
            float right_power = robot.get_right_power();
            
            // Sensor readings
            float distance = robot.get_distance_cm();
            float ax, ay, az, gx, gy, gz, qx, qy, qz, qw;
            bool accel_valid = robot.get_acceleration(ax, ay, az);
            bool gyro_valid = robot.get_gyroscope(gx, gy, gz);
            bool quat_valid = robot.get_quaternion(qx, qy, qz, qw);
            
            // Safety status
            bool warning_active = robot.is_warning_active();
            bool emergency_active = robot.is_emergency_stop_active();
            
            printf("\n=== DiffDrive Robot Status ===\n");
            printf("LEFT  Motor: Desired=%.1f rpm, Current=%.1f rpm, Error=%.1f rpm, Power=%.3f\n",
                   left_desired, left_current, left_error, left_power);
            printf("RIGHT Motor: Desired=%.1f rpm, Current=%.1f rpm, Error=%.1f rpm, Power=%.3f\n",
                   right_desired, right_current, right_error, right_power);
            
            printf("Distance: ");
            if (distance >= 0) {
                printf("%.1f cm", distance);
            } else {
                printf("INVALID");
            }
            printf("\n");
            
            if (accel_valid) {
                printf("Accel: X=%.3f, Y=%.3f, Z=%.3f m/s²\n", ax, ay, az);
            }
            if (gyro_valid) {
                printf("Gyro:  X=%.3f, Y=%.3f, Z=%.3f rad/s\n", gx, gy, gz);
            }
            if (quat_valid) {
                printf("Quat:  X=%.3f, Y=%.3f, Z=%.3f, W=%.3f\n", qx, qy, qz, qw);
            }
            
            // Check for status changes
            if (warning_active && !last_warning_state) {
                printf("*** WARNING: Obstacle detected at %.1f cm! ***\n", distance);
            }
            if (emergency_active && !last_emergency_state) {
                printf("*** EMERGENCY STOP: Motors stopped due to obstacle! ***\n");
            }
            if (!emergency_active && last_emergency_state) {
                printf("*** Emergency stop cleared - motors can resume ***\n");
            }
            
            last_warning_state = warning_active;
            last_emergency_state = emergency_active;
            last_time_status = current_time;
        }
        
        // SPEED CHANGE - 10000ms (10 seconds per speed)
        if (current_time - speed_change_time >= 10000) {
            float target_speed = speed_targets[current_speed_index];
            
            printf("\n=== Setting both motors to %.0f RPM ===\n", target_speed);
            robot.set_speeds(target_speed, target_speed);
            
            current_speed_index = (current_speed_index + 1) % num_speeds;
            speed_change_time = current_time;
        }
        
        sleep_ms(10);
    }

#else
    printf("ERROR: No test mode selected!\n");
    printf("Please uncomment one of the following in the defines section:\n");
    printf("- #define TEST_PRECISION_MOTORS\n");
    printf("- #define TEST_IMU_MPU\n");
    printf("- #define TEST_IMU_BMI\n");
    printf("- #define TEST_ULTRASONIC\n");
    printf("- #define TEST_OBSTACLE_AVOID\n");
    printf("- #define TEST_DIFF_DRIVE\n");
    
    while (true) {
        gpio_put(LED_PIN, 1);
        sleep_ms(500);
        gpio_put(LED_PIN, 0);
        sleep_ms(500);
    }
#endif
    
    return 0;
}