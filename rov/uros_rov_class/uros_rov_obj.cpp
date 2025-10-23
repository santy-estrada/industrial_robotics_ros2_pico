#include <hardware/gpio.h>
#include <pico/stdlib.h>
#include <stdio.h>

#include "pico_uart_transports.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <geometry_msgs/msg/twist.h>

#include "motor/Motor.h"
#include "sensor/LimitSwitch.h"
#include "sensor/MPS20N0040D.h"
#include "sensor/SOIL_MOIST.h"
#include "rov/ROV.h"

#define LED_PIN 25

// uROS node configuration
#define NODE_NAME "rov_node"
#define ROV_CMD_VEL_TOPIC "rov_motor_control"
#define ROV_STATUS_TOPIC "rov_status"

// Timing configuration
#define STATUS_TIMER_PERIOD_MS 50    // 20Hz status publishing

// Initialize global variables for ROS2
rcl_subscription_t cmd_vel_sub;           // Subscriber for command velocity
geometry_msgs__msg__Twist cmd_vel_msg;    // Message for command velocity

rcl_publisher_t status_pub;               // Publisher for ROV status
geometry_msgs__msg__Twist status_msg;     // Message for ROV status

// Global ROV instance pointer
ROV* g_rov = nullptr;

// Global flags and command storage
bool g_emergency_active = false;
bool g_stop_all_motors = false;
float g_thrust1_cmd = 0.0f;
float g_thrust2_cmd = 0.0f;
float g_ballast_cmd = 0.0f;

// ROS2 subscriber callback for receiving Twist messages -> ROV commands
void cmd_vel_subscription_callback(const void* msgin) {
    const geometry_msgs__msg__Twist* msg = (const geometry_msgs__msg__Twist*)msgin;
    
    if (g_rov == nullptr) return;
    
    // angular.z: Emergency emerge (1 = activate, 0 = normal operation)
    if (msg->angular.z > 0.5) {
        printf("Emergency emerge activated!\n");
        g_emergency_active = true;
        g_stop_all_motors = false;  // Emergency overrides stop
        return;  // Emergency takes priority over other commands
    } else if (g_emergency_active) {
        // Deactivate emergency if command is now 0
        g_emergency_active = false;
        printf("Emergency emerge deactivated\n");
    }
    
    // angular.y: Calibrate zero depth (1 = calibrate)
    if (msg->angular.y > 0.5) {
        g_rov->calibrateZeroDepth();
        printf("Depth calibration performed\n");
    }
    
    // angular.x: Stop all motors (1 = stop, 0 = allow motion)
    if (msg->angular.x > 0.5) {
        g_stop_all_motors = true;
        printf("Stop all motors flag set\n");
    } else {
        g_stop_all_motors = false;
    }
    
    // Store movement commands (will be applied in timer callback)
    g_thrust1_cmd = msg->linear.x;   // Thrust motor 1 (-1.0 to 1.0)
    g_thrust2_cmd = msg->linear.y;   // Thrust motor 2 (-1.0 to 1.0)
    g_ballast_cmd = msg->linear.z;   // Ballast motor (-1.0 to 1.0, exponential mapping)
    
    printf("ROV: Command received - thrust1:%.2f, thrust2:%.2f, ballast:%.2f, stop:%d, emergency:%d\n",
           g_thrust1_cmd, g_thrust2_cmd, g_ballast_cmd, g_stop_all_motors, g_emergency_active);
}

// Timer callback to publish ROV status AND apply motor commands
void status_timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
    (void)last_call_time;
    
    if (timer == NULL || g_rov == nullptr) return;
    
    // ===== MOTOR CONTROL LOGIC =====
    // Priority: Emergency > Stop > Normal commands
    
    if (g_emergency_active) {
        // Emergency emerge: run ballast backward until empty switch is pressed
        if (!g_rov->isEmptySwitchPressed()) {
            g_rov->emergencyEmerge();
        } else {
            // Emergency complete - switch pressed
            g_emergency_active = false;
            g_rov->stop();
            printf("Emergency emerge complete - empty switch reached\n");
        }
    } else if (g_stop_all_motors) {
        // Stop flag is active - stop all motors
        g_rov->stop();
    } else {
        // Normal operation - apply received commands
        g_rov->setThrustMotor1(g_thrust1_cmd);
        g_rov->setThrustMotor2(g_thrust2_cmd);
        g_rov->setBallast(g_ballast_cmd);
    }
    
    // ===== SENSOR READING AND PUBLISHING =====
    // Prepare and publish ROV status message
    status_msg.linear.x = g_rov->getDepth();
    status_msg.linear.y = 0.0;
    status_msg.linear.z = g_rov->isMoistureDetected() ? 1.0 : 0.0;
    
    status_msg.angular.x = g_rov->isEmptySwitchPressed() ? 1.0 : 0.0;
    status_msg.angular.y = 0.0;
    status_msg.angular.z = g_rov->isFullSwitchPressed() ? 1.0 : 0.0;
    
    rcl_ret_t ret = rcl_publish(&status_pub, &status_msg, NULL);
}

int main() {
    stdio_init_all();  // Initialize standard I/O (for debugging)
    
    // Initialize basic hardware
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);  // LED off initially
    
    // Initialize ROV with same pins as TEST_ROV
    // Thrust motors: Motor1=GPIO(0,1,2), Motor2=GPIO(3,4,5)
    // Ballast motor: GPIO(21,20,19)
    // Limit switches: Empty=GPIO8, Full=GPIO7
    // Moisture sensor: Digital=GPIO26, Analog=GPIO28
    // Pressure sensor: SCK=GPIO11, OUT=GPIO10
    ROV rov(0, 1, 2,      // Thrust motor 1
            3, 4, 5,      // Thrust motor 2
            21, 20, 19,   // Ballast motor
            8, 7,         // Empty and Full limit switches
            26, 28,       // Moisture sensor
            11, 10);      // Pressure sensor
    
    // Set depth conversion (user should calibrate in practice)
    rov.setDepthConversion(0.0f, 1.0f);
    
    // Set global ROV pointer
    g_rov = &rov;
    
    printf("ROV initialized successfully!\n");
    
    // Initialize micro-ROS transport
    rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );
    
    // Initialize ROS2 components
    rcl_timer_t status_timer;
    rcl_node_t node;
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rclc_executor_t executor;
    
    // Wait for agent connection
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
    
    // Initialize the publisher for ROV status
    rclc_publisher_init_default(
        &status_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        ROV_STATUS_TOPIC
    );
    
    // Initialize the timer for status publishing (100ms interval)
    rclc_timer_init_default(
        &status_timer,
        &support,
        RCL_MS_TO_NS(STATUS_TIMER_PERIOD_MS),
        status_timer_callback
    );
    
    // Initialize the subscriber to receive Twist messages for commands
    rclc_subscription_init_default(
        &cmd_vel_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        ROV_CMD_VEL_TOPIC
    );
    
    // Initialize executor
    rclc_executor_init(&executor, &support.context, 2, &allocator);  // 1 subscriber + 1 timer
    
    rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, &cmd_vel_subscription_callback, ON_NEW_DATA);
    
    // Indicate ROV created successfully
    for(int i = 0; i < 3; i++) {
        gpio_put(LED_PIN, 1);
        sleep_ms(500);
        gpio_put(LED_PIN, 0);
        sleep_ms(500);
    }
    
    // Add timer to executor
    rclc_executor_add_timer(&executor, &status_timer);
    
    printf("ROV micro-ROS system ready!\n");
    printf("Listening on: %s\n", ROV_CMD_VEL_TOPIC);
    printf("Publishing on: %s\n", ROV_STATUS_TOPIC);
    
    // Main loop - just spin the executor, all logic is in callbacks
    while (true) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50));
    }
    
    return 0;
}
