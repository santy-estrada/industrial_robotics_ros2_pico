#include <hardware/gpio.h>
#include <hardware/pwm.h>
#include <hardware/irq.h>
#include <pico/stdlib.h>
#include <math.h>
#include <map>

#include "pico_uart_transports.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <sensor_msgs/msg/range.h>
#include <sensor_msgs/msg/imu.h>

#include <rosidl_runtime_c/string_functions.h>
#include <rmw_microros/time_sync.h>

// Differential Drive Robot libraries
#include "robot/DiffDrive.h"
#include <stdio.h>

// Pin configuration defines - Updated to match diff_app.cpp
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

// Global DiffDrive robot pointer - initialized in main() after hardware setup
DiffDrive* g_diff_robot = nullptr;

// uROS node configuration
#define NODE_NAME "diff_drive_robot_node"
#define DIFF_MEASUREMENTS_TOPIC "diff_measurements"
#define WHEEL_SETPOINT_TOPIC "wheel_setpoint"
#define IMU_TOPIC "imu"
#define ULTRASONIC_TOPIC "ultrasonic"

// Timing configuration
#define CONTROL_TIMER_PERIOD_MS 100    // 10Hz control loop (dt = 0.1s)

// ROS2 Publishers and Subscribers
rcl_subscription_t wheel_setpoint_subs;        // Subscriber for wheel speed setpoints
geometry_msgs__msg__Twist wheel_setpoint_msg;  // Message for wheel setpoints

rcl_publisher_t diff_measurements_pub;         // Publisher for wheel measurements
geometry_msgs__msg__Twist diff_measurements_msg; // Message for wheel measurements

rcl_publisher_t imu_pub;                       // Publisher for IMU data
sensor_msgs__msg__Imu imu_msg;                 // IMU message

rcl_publisher_t range_pub;                     // Publisher for ultrasonic range
sensor_msgs__msg__Range range_msg;             // Range message

// State tracking
bool system_initialized = false;

// Wheel speed setpoints (RPM)
float left_wheel_setpoint = 0.0f;
float right_wheel_setpoint = 0.0f;

// ROS2 subscriber callback for receiving wheel speed setpoints
void wheel_setpoint_subscription_callback(const void * msgin) {
    const geometry_msgs__msg__Twist * twist_msg_const = (const geometry_msgs__msg__Twist *)msgin;
    
    printf("DIFF: Received wheel setpoints - Left:%.2f rpm, Right:%.2f rpm\n", 
           twist_msg_const->linear.x, twist_msg_const->angular.z);
    
    // Only accept commands if system is initialized
    if (!system_initialized) {
        printf("DIFF: Commands ignored - system not initialized\n");
        return;
    }
    
    // Extract wheel setpoints from Twist message
    left_wheel_setpoint = twist_msg_const->linear.x;   // Left wheel speed (RPM)
    right_wheel_setpoint = twist_msg_const->angular.z; // Right wheel speed (RPM)
    
    // Set motor speeds
    g_diff_robot->set_speeds(left_wheel_setpoint, right_wheel_setpoint);
    
    printf("DIFF: Setpoints applied - Left:%.2f rpm, Right:%.2f rpm\n", 
           left_wheel_setpoint, right_wheel_setpoint);
}

// IMU data callback function
void imu_callback() {
    if (!system_initialized || !g_diff_robot) return;
    
    float ax, ay, az, gx, gy, gz, qx, qy, qz, qw;
    
    // Get IMU data from the robot
    bool accel_valid = g_diff_robot->get_acceleration(ax, ay, az);
    bool gyro_valid = g_diff_robot->get_gyroscope(gx, gy, gz);
    bool quat_valid = g_diff_robot->get_quaternion(qx, qy, qz, qw);
    
    if (!accel_valid || !gyro_valid || !quat_valid) {
        return; // Skip publishing if data is invalid
    }
    
    // Set timestamp
    int64_t epoch_ms = rmw_uros_epoch_millis();
    imu_msg.header.stamp.sec = epoch_ms / 1000;
    imu_msg.header.stamp.nanosec = (epoch_ms % 1000) * 1000000;
    
    // Set frame ID
    rosidl_runtime_c__String__assign(&imu_msg.header.frame_id, "base_link");
    
    // Fill IMU message
    imu_msg.linear_acceleration.x = ax;
    imu_msg.linear_acceleration.y = ay;
    imu_msg.linear_acceleration.z = az;
    
    imu_msg.angular_velocity.x = gx;
    imu_msg.angular_velocity.y = gy;
    imu_msg.angular_velocity.z = gz;
    
    imu_msg.orientation.x = qx;
    imu_msg.orientation.y = qy;
    imu_msg.orientation.z = qz;
    imu_msg.orientation.w = qw;
    
    // Set covariance values
    imu_msg.orientation_covariance[0] = 0.02;
    imu_msg.orientation_covariance[4] = 0.02;
    imu_msg.orientation_covariance[8] = 0.02;
    
    imu_msg.angular_velocity_covariance[0] = 0.01;
    imu_msg.angular_velocity_covariance[4] = 0.01;
    imu_msg.angular_velocity_covariance[8] = 0.01;
    
    imu_msg.linear_acceleration_covariance[0] = 0.01;
    imu_msg.linear_acceleration_covariance[4] = 0.01;
    imu_msg.linear_acceleration_covariance[8] = 0.01;
    
    // Publish IMU data
    rcl_publish(&imu_pub, &imu_msg, NULL);
}

// Ultrasonic range callback function
void range_callback() {
    if (!system_initialized || !g_diff_robot) return;
    
    // Get distance measurement from the robot
    float distance = g_diff_robot->get_distance_cm();
    
    if (distance < 0) {
        return; // Skip publishing if measurement is invalid
    }
    
    // Convert cm to meters
    distance = distance / 100.0f;
    
    // Set timestamp
    int64_t epoch_ms = rmw_uros_epoch_millis();
    range_msg.header.stamp.sec = epoch_ms / 1000;
    range_msg.header.stamp.nanosec = (epoch_ms % 1000) * 1000000;
    
    // Set frame ID
    rosidl_runtime_c__String__assign(&range_msg.header.frame_id, "base_link");
    
    // Fill range message
    range_msg.range = distance;
    range_msg.min_range = 0.02;   // 2 cm minimum
    range_msg.max_range = 4.0;    // 4 m maximum
    range_msg.field_of_view = 0.5; // radians
    range_msg.radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
    
    // Publish range data
    rcl_publish(&range_pub, &range_msg, NULL);
}

// Timer callback to perform control and publish measurements every 100ms
void control_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    if (timer == nullptr || !system_initialized || !g_diff_robot) {
        return;
    }
    
    // Update robot state (motor control, sensor reading, safety checks)
    g_diff_robot->update();
    
    // Get current wheel speeds
    float left_current_speed = g_diff_robot->get_left_current_speed();
    float right_current_speed = g_diff_robot->get_right_current_speed();

    // Get warning and emergency stop status
    bool warning_active = g_diff_robot->is_warning_active();
    bool emergency_active = g_diff_robot->is_emergency_stop_active();
    

    // Prepare and publish wheel measurements message
    // linear.x = left wheel speed (RPM), angular.z = right wheel speed (RPM)
    diff_measurements_msg.linear.x = left_current_speed;
    diff_measurements_msg.linear.y = 0.0f; // Unused
    diff_measurements_msg.linear.z = (warning_active ? 1.0f : 0.0f); // Warning status

    diff_measurements_msg.angular.x = (emergency_active ? 1.0f : 0.0f); // Emergency status
    diff_measurements_msg.angular.y = 0.0f; // Unused
    diff_measurements_msg.angular.z = right_current_speed;
    
    // Publish wheel measurements
    rcl_ret_t ret = rcl_publish(&diff_measurements_pub, &diff_measurements_msg, NULL);
    
    // Publish sensor data
    imu_callback();
    range_callback();
}



int main() {
    stdio_init_all();  // Initialize standard I/O (for debugging)

    // Initialize basic hardware
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0); // LED off initially
    
    // Create DiffDrive robot instance after hardware initialization
    DiffDrive diff_robot(
        // Left motor pins
        MOTOR_L_ENA, MOTOR_L_IN1, MOTOR_L_IN2,
        ENCODER_L_A, ENCODER_L_B,
        // Right motor pins
        MOTOR_R_ENA, MOTOR_R_IN1, MOTOR_R_IN2,
        ENCODER_R_A, ENCODER_R_B,
        // Ultrasonic sensor pins
        ULTRASONIC_TRIG, ULTRASONIC_ECHO,
        // IMU pins and I2C port
        IMU_SDA, IMU_SCL, i2c1,
        // Motor parameters
        ENCODER_TICKS_PER_REV, GEAR_RATIO, dt,
        PID_KP, PID_TI, PID_TD
    );
    
    // Set global pointer to the robot instance
    g_diff_robot = &diff_robot;
    
    rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );

    // Initialize ROS2 components
    rcl_timer_t control_timer;
    rcl_node_t node;
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rclc_executor_t executor;

    rcl_ret_t ret = rmw_uros_ping_agent(1000, 120);  // Wait for the Micro-ROS agent
    if (ret != RCL_RET_OK) {
        printf("Failed to connect to Micro-ROS agent.\n");
        return ret;
    }
    printf("Connected to Micro-ROS agent.\n");

    // LED on to indicate agent connection established
    gpio_put(LED_PIN, 1);
    sleep_ms(2000);
    gpio_put(LED_PIN, 0);

    // Initialize support and node
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, NODE_NAME, "", &support);

    // Initialize the publisher for wheel measurements
    rclc_publisher_init_default(
        &diff_measurements_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        DIFF_MEASUREMENTS_TOPIC
    );

    // Initialize the publisher for IMU data
    rclc_publisher_init_default(
        &imu_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        IMU_TOPIC
    );

    // Initialize the publisher for ultrasonic range data
    rclc_publisher_init_default(
        &range_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
        ULTRASONIC_TOPIC
    );

    // Initialize the timer for control (100ms interval)
    rclc_timer_init_default2(
        &control_timer,
        &support,
        RCL_MS_TO_NS(CONTROL_TIMER_PERIOD_MS),
        control_timer_callback,
        true
    );

    // Initialize the subscriber to receive wheel setpoint commands
    rclc_subscription_init_default(
        &wheel_setpoint_subs,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        WHEEL_SETPOINT_TOPIC
    );

    // Initialize executor
    rclc_executor_init(&executor, &support.context, 2, &allocator);
    
    rclc_executor_add_subscription(&executor, &wheel_setpoint_subs, &wheel_setpoint_msg, &wheel_setpoint_subscription_callback, ON_NEW_DATA);

    // Indicate robot created successfully
    for(int i = 0; i < 3; i++) {
        gpio_put(LED_PIN, 1);
        sleep_ms(500);
        gpio_put(LED_PIN, 0);
        sleep_ms(500);
    }

    // CRITICAL: Initialize robot BEFORE starting timers to avoid timing conflicts
    if (!g_diff_robot->init()) {
        printf("ERROR: Failed to initialize DiffDrive robot!\n");
        return -1;
    }
    
    system_initialized = true;
    printf("DiffDrive robot initialized successfully!\n");

    // Indicate system initialized successfully
    for (int i = 0; i < 8; i++) {
        gpio_put(LED_PIN, 1);
        sleep_ms(250);
        gpio_put(LED_PIN, 0);
        sleep_ms(250);
    }

    // Add timer AFTER initialization is complete
    rclc_executor_add_timer(&executor, &control_timer);
    
    printf("DiffDrive micro-ROS node ready!\n");
    printf("Subscribed to: %s\n", WHEEL_SETPOINT_TOPIC);
    printf("Publishing to: %s, %s, %s\n", DIFF_MEASUREMENTS_TOPIC, IMU_TOPIC, ULTRASONIC_TOPIC);

    while (true) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50));
    }

    return 0;
}