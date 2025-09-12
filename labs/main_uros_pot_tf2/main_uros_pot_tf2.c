#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"

// Micro-ROS headers
#include <rcl/rcl.h>                     // Core ROS Client Library
#include <rcl/error_handling.h>
#include <rclc/rclc.h>                   // Convenience init for nodes, pubs, subs, timers
#include <rclc/executor.h>               // Executor to handle callbacks
#include <geometry_msgs/msg/twist.h>     // Standard ROS 2 Twist message (linear + angular)
#include <rmw_microros/rmw_microros.h>   // Micro-ROS middleware (transport handling)

#include "pico_uart_transports.h"        // Custom UART transport for Pico

// Define pins for Pico
#define LED_PIN 25   // Onboard LED
#define POT1_PIN 26  // Potentiometer 1 connected to ADC input
#define POT2_PIN 27  // Potentiometer 2 connected to ADC input

// Global publisher handle (used across callbacks + main)
rcl_publisher_t publisher;

// Global Twist message to be filled and published
geometry_msgs__msg__Twist twist_msg;

// Map helper function: converts value from [in_min, in_max] to [out_min, out_max]
long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Timer callback: executed periodically by rclc_executor
void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    // Read first potentiometer (mapped to Twist.linear.x)
    adc_select_input(0);                  // ADC input channel for POT1
    uint16_t result_pot1 = adc_read();    // Raw 12-bit ADC value [0–4095]

    // Read second potentiometer (mapped to Twist.angular.z)
    adc_select_input(1);                  // ADC input channel for POT2
    uint16_t result_pot2 = adc_read();

    // Map raw ADC values to a smaller range [0–255] for control
    long pot1_value_map = map(result_pot1, 0, 4095, 0, 255);
    long pot2_value_map = map(result_pot2, 0, 4095, 0, 255);

    // Fill the Twist message
    twist_msg.linear.x  = pot1_value_map; // Forward/backward "speed"
    twist_msg.angular.z = pot2_value_map; // Rotation "speed"

    // Publish Twist message to ROS 2 topic
    rcl_ret_t ret = rcl_publish(&publisher, &twist_msg, NULL);
}

int main()
{
    // Configure Micro-ROS to use custom UART transport on the Pico
    rmw_uros_set_custom_transport(
        true,                             // Enable custom transport
        NULL,                             // Optional user data (unused)
        pico_serial_transport_open,       // UART open function
        pico_serial_transport_close,      // UART close function
        pico_serial_transport_write,      // UART write (TX)
        pico_serial_transport_read        // UART read (RX)
    );

    // Initialize LED (used to signal readiness)
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // Initialize ADC for potentiometer inputs
    adc_init();
    adc_gpio_init(POT1_PIN);  // Enable GPIO pin for ADC
    adc_gpio_init(POT2_PIN);  // Enable GPIO pin for ADC

    // Declare ROS 2 entities
    rcl_timer_t pub_timer;        // Timer to periodically publish Twist
    rcl_node_t node;              // Node handle ("pico_node")
    rcl_allocator_t allocator = rcl_get_default_allocator(); // Memory allocator
    rclc_support_t support;       // Wraps init context and allocator
    rclc_executor_t executor;     // Runs callbacks (timers, subs, etc.)

    // Ensure Micro-ROS agent is reachable before starting
	rcl_ret_t ret = rmw_uros_ping_agent(1000, 120);  // Wait for the Micro-ROS agent
    if (ret != RCL_RET_OK) {
        printf("Failed to connect to Micro-ROS agent.\n");
        return ret;
    }

	// Initialize support and node
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "pico_node", "", &support);       //pico_node can be replaced to anytthing

	// Initialize the publisher to publish twist
    rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "twist_msg"                                                 //twist_msg can be replaced to anything
    );

	// Initialize the timer for twist msg (100ms interval)
    rclc_timer_init_default(
        &pub_timer,
        &support,
        RCL_MS_TO_NS(100),
        timer_callback
    );

	// Initialize executor and the timer
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor, &pub_timer);
    
    // LED On
    gpio_put(LED_PIN, 1);

    //printf("start pico adc\n");
    while (true) {
 		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    
    return 0;
}