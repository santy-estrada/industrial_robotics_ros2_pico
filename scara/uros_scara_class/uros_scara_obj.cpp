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

// Scara libraries
#include "motor/PrecisionMotor.h"
#include "motor/ServoMotor.h"
#include "sensor/LimitSwitch.h"
#include "scara/Joint.h"
#include "scara/ServoJoint.h"
#include "scara/ScaraRobot.h"
#include <stdio.h>

// Joint configuration defines - Updated to match tested hardware pins
#define LED_PIN 25

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

// Global SCARA robot pointer - initialized in main() after hardware setup
ScaraRobot* g_scara_robot = nullptr;

// uROS node configuration
#define NODE_NAME "scara_robot_node"
#define SCARA_MEASUREMENTS_TOPIC "scara_measurements"
#define DEBUG_PUB_TOPIC "debug_pub"
#define SCARA_CONF_TOPIC "scara_conf"

// Timing configuration
#define CONTROL_TIMER_PERIOD_MS 50    // 20Hz control loop
#define DEBUG_TIMER_PERIOD_MS 500     // 2Hz debug publishing


// Initialize global variables for ROS2
rcl_subscription_t scara_conf_subs; // Subscriber for configuration commands
geometry_msgs__msg__Twist conf_msg; // Message for configuration commands

rcl_publisher_t scara_measurements_pub;    // Publisher for scara measurements
geometry_msgs__msg__Twist measurements_msg;     // Message type for the scara measurements

rcl_publisher_t debug_pub;    // Publisher for Debugging messages
geometry_msgs__msg__Twist debug_msg;     // Message type for the Debugging messages


// State tracking
bool system_calibrated = false;

// Function to map a value from one range to another
float map(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float conf[] = {0.0f, 0.0f, 0.0f}; // Joint configuration setpoints

// ROS2 subscriber callback for receiving Twist messages -> Configuration commands
void scara_conf_subscription_callback(const void * msgin) {

    const geometry_msgs__msg__Twist * twist_msg_const = (const geometry_msgs__msg__Twist *)msgin;
    
    printf("SCARA: Received command - x:%.2f, y:%.2f, z:%.2f\n", 
           twist_msg_const->linear.x, twist_msg_const->linear.y, twist_msg_const->linear.z);
    
    // Only accept commands if system is calibrated and safe to move
    if (!system_calibrated) {
        printf("SCARA: Commands ignored - system not calibrated\n");
        return;
    }
    
    if (!g_scara_robot->isSafeToMove()) {
        printf("SCARA: Commands ignored - safety pendant not active\n");
        return;
    }

    // Configuration values for each joint
    conf[0] = twist_msg_const->linear.x;
    conf[1] = twist_msg_const->linear.y;
    conf[2] = twist_msg_const->linear.z;
    
    printf("SCARA: Command accepted - conf[%.2f, %.2f, %.2f]\n", conf[0], conf[1], conf[2]);
}

// Timer callback to perform control and publish state every 50ms
void control_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {

    if (timer == nullptr) {
        return;
    }

    if (!system_calibrated) {
        return;
    }
    
    
    // Update all joint states (includes safety pendant check)
    g_scara_robot->moveToConfiguration(conf[0], conf[1], conf[2]);


    // Prepare and publish joint measurements message
    // Map joint positions to linear fields, joint speeds to angular fields
    measurements_msg.linear.x = g_scara_robot->getJoint1CurrentPosition();
    measurements_msg.linear.y = g_scara_robot->getJoint2CurrentPosition();  
    measurements_msg.linear.z = g_scara_robot->getJoint3CurrentPosition();
    
    measurements_msg.angular.x = g_scara_robot->getJoint1CurrentSpeed();
    measurements_msg.angular.y = g_scara_robot->getJoint2CurrentSpeed();
    measurements_msg.angular.z = 0.0f; // Joint 3 speed not available for servo


    rcl_ret_t ret2 = rcl_publish(&scara_measurements_pub, &measurements_msg, NULL);
}

// Timer callback to publish debug info every 500ms
void debug_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {

    if (timer == nullptr || !system_calibrated) {
        return;
    }

   // Prepare debug message 
    // Map joint targets to linear fields, joint errors to angular fields
    debug_msg.linear.x = g_scara_robot->getJoint1DesiredPosition();
    debug_msg.linear.y = g_scara_robot->getJoint1ErrorPosition();
    debug_msg.linear.z = g_scara_robot->getJoint2DesiredPosition(); // Servo target equals current for debug
    
    debug_msg.angular.x = g_scara_robot->getJoint2ErrorPosition();
    debug_msg.angular.y = g_scara_robot->areRevoluteJointsCalibrated() ? 1.0 : 0.0;  // Calibration status
    debug_msg.angular.z = g_scara_robot->isPendantEnabled() ? 1.0 : 0.0;  // Safety pendant status

    rcl_ret_t ret2 = rcl_publish(&debug_pub, &debug_msg, NULL);
}



int main() {
    stdio_init_all();  // Initialize standard I/O (for debugging)

    // Initialize basic hardware
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0); // LED off initially
    
    // Create SCARA robot instance after hardware initialization
    ScaraRobot scara_robot(
        // Joint 1 pins
        JOINT1_MOTOR_ENA, JOINT1_MOTOR_IN1, JOINT1_MOTOR_IN2,
        JOINT1_ENCODER_A, JOINT1_ENCODER_B, JOINT1_LIMIT_MIN, JOINT1_LIMIT_MAX,
        // Joint 2 pins
        JOINT2_MOTOR_ENA, JOINT2_MOTOR_IN1, JOINT2_MOTOR_IN2,
        JOINT2_ENCODER_A, JOINT2_ENCODER_B, JOINT2_LIMIT_MIN, JOINT2_LIMIT_MAX,
        // Joint 3 pins
        JOINT3_SERVO_PWM,
        // // Safety pendant pin
        SAFETY_PENDANT_PIN
    );
    
    // Set global pointer to the robot instance
    g_scara_robot = &scara_robot;
    
    rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );

    // Initialize ROS2 components
    rcl_timer_t debug_timer;
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


    // Initialize the publisher to publish debug messages
    rclc_publisher_init_default(
        &debug_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        DEBUG_PUB_TOPIC
    );

    // Initialize the timer for debug (500ms interval)
    rclc_timer_init_default(
        &debug_timer,
        &support,
        RCL_MS_TO_NS(DEBUG_TIMER_PERIOD_MS),
        debug_timer_callback
    );

    // Initialize the publisher to control and publish state
    rclc_publisher_init_default(
        &scara_measurements_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        SCARA_MEASUREMENTS_TOPIC
    );

    // Initialize the timer for control (50ms interval)
    rclc_timer_init_default(
        &control_timer,
        &support,
        RCL_MS_TO_NS(CONTROL_TIMER_PERIOD_MS),
        control_timer_callback
    );

    // Initialize the subscriber to receive Twist messages with configuration
    rclc_subscription_init_default(
        &scara_conf_subs,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        SCARA_CONF_TOPIC
    );


    // Initialize executor
    rclc_executor_init(&executor, &support.context, 3, &allocator);
    
    rclc_executor_add_subscription(&executor, &scara_conf_subs, &conf_msg, &scara_conf_subscription_callback, ON_NEW_DATA);

    //Indicate robot created successfully
    for(int i =0; i<3; i++) {
        gpio_put(LED_PIN, 1);
        sleep_ms(500);
        gpio_put(LED_PIN, 0);
        sleep_ms(500);
    }

    // CRITICAL: Calibrate BEFORE starting timers to avoid timing conflicts
    g_scara_robot->calibrateRevoluteJoints();  // Calibrates joint2 then joint1
    g_scara_robot->calibrateServoJoint(90.0f); // Set servo 90° as joint origin (0°)
    system_calibrated = true;

    //Indicate system calibrated successfully
    for (int i = 0; i < 8; i++)
    {
        gpio_put(LED_PIN, 1);
        sleep_ms(250);
        gpio_put(LED_PIN, 0);
        sleep_ms(250);
    }

    // Add timers AFTER calibration is complete
    rclc_executor_add_timer(&executor, &debug_timer);
    rclc_executor_add_timer(&executor, &control_timer);
    

    while (true) {

        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50));
    }

    return 0;
}