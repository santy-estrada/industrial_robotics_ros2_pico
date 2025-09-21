#include "motor/Motor.h"
#include "motor/PrecisionMotor.h"
#include "sensor/TEMT6000.h"
#include "sensor/HCSR05.h"
#include "sensor/LimitSwitch.h"
#include "sensor/MPS20N0040D.h"
#include <stdio.h>

// Test configuration defines - comment/uncomment to enable/disable tests
// #define TEST_MOTOR
// #define TEST_BALLAST
// #define TEST_PRECISION_MOTOR
#define TEST_ENCODER_TICKS
// #define TEST_LIMITED_CONTROLLER
// #define TEST_LIGHT_SENSOR
// #define TEST_ULTRASONIC_SENSOR
// #define TEST_LIMIT_SWITCH
// #define TEST_PRESSURE_SENSOR

#define LED_PIN 25


// Example usage of the motor and sensor classes
int main() {
    stdio_init_all();
     // Always initialize LED pin
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    
#ifdef TEST_LIGHT_SENSOR
    // Create sensor on GPIO28 (ADC2)
    TEMT6000 light_sensor(28);
#endif

#ifdef TEST_ULTRASONIC_SENSOR
    // Create sensor with trigger on GPIO27, echo on GPIO26
    HCSR05 ultrasonic(27, 26);
#endif

#ifdef TEST_LIMIT_SWITCH
    // Create limit switches on different GPIO pins
    LimitSwitch limit_switch1(8);  // GPIO8
    LimitSwitch limit_switch2(9);  // GPIO9
#endif

#ifdef TEST_PRESSURE_SENSOR
    // Create pressure sensor with HX710B amplifier
    MPS20N0040D pressure_sensor(11, 10);  // OUT=GPIO10, SCK=GPIO11
    pressure_sensor.setGain(128);  // Set gain to 128 (default)
#endif

#ifdef TEST_MOTOR
    // Create a simple motor
    Motor simple_motor1(0, 1, 2);  // ENA=0, IN1=1, IN2=2
    Motor simple_motor2(3, 4, 5);  // ENA=3, IN1=4, IN2=5
#endif

#ifdef TEST_BALLAST
    // Create ballast motor
    Motor simple_motor3(16, 18, 17);  // ENA=16, IN1=17, IN2=18
#endif
    
#ifdef TEST_PRECISION_MOTOR
    // Create precision motors with different configurations
    PrecisionMotor precision_motor1(16, 18, 17, 12, 13, 28, 150.0f);   // Microgear motor
    PrecisionMotor precision_motor2(21, 19, 20, 14, 15, 64, 50.0f);    // Pololu motor
#endif

#ifdef TEST_ENCODER_TICKS
    // Create precision motors for encoder tick testing (motors will stay stopped)
    PrecisionMotor encoder_motor1(16, 18, 17, 12, 13, 28, 150.0f);   // Microgear motor
    PrecisionMotor encoder_motor2(21, 19, 20, 14, 15, 64, 50.0f);    // Pololu motor
#endif

#ifdef TEST_LIMITED_CONTROLLER
    // Create precision motors for limited controller testing
    PrecisionMotor limited_motor1(16, 18, 17, 12, 13, 28, 150.0f);   // Microgear motor
    PrecisionMotor limited_motor2(21, 19, 20, 14, 15, 64, 50.0f);    // Pololu motor
#endif
    
    printf("Components initialized successfully!\n");
    
#ifdef TEST_MOTOR
    // Test simple motor1
    printf("Testing simple motors...\n");
    fflush(stdout);
    printf("Motor 1: 50%% power forward\n");
    fflush(stdout);
    simple_motor1.moveFwd(50.0f);  // Move forward at 50% power
    sleep_ms(5000);
    printf("Motor 1: 30%% power backward\n");
    fflush(stdout);
    simple_motor1.moveBckwd(30.0f); // Move backward at 30% power
    sleep_ms(5000);
    printf("Motor 1: Stopping\n");
    fflush(stdout);
    simple_motor1.stop();
    sleep_ms(1000);
    // Test simple motor2
    printf("Motor 2: 70%% power forward\n");
    fflush(stdout);
    simple_motor2.moveFwd(50.0f);  // Move forward at 70% power
    sleep_ms(5000);
    printf("Motor 2: 20%% power backward\n");
    fflush(stdout);
    simple_motor2.moveBckwd(20.0f); // Move backward at 20% power
    sleep_ms(5000);
    printf("Motor 2: Stopping\n");
    fflush(stdout);
    simple_motor2.stop();
    sleep_ms(1000);
    //Test both motors together
    printf("Both motors: 60%% power forward\n");
    fflush(stdout);
    simple_motor1.moveFwd(60.0f);
    simple_motor2.moveFwd(60.0f);
    sleep_ms(5000);
    printf("Both motors: Stopping\n");
    fflush(stdout);
    simple_motor1.stop();
    simple_motor2.stop();
    sleep_ms(1000); 
#endif

#ifdef TEST_BALLAST
    // Test ballast motor separately
    printf("Testing ballast motor...\n");
    fflush(stdout);
    printf("Ballast: 70%% power forward (fill)\n");
    fflush(stdout);
    simple_motor3.moveFwd(70.0f);  // Move forward at 70% power
    sleep_ms(2000);
    printf("Ballast: Stopping\n");
    fflush(stdout);
    simple_motor3.stop();
    sleep_ms(1000);
    printf("Ballast: 70%% power backward (empty)\n");
    fflush(stdout);
    simple_motor3.moveBckwd(70.0f); // Move backward at 70% power
    sleep_ms(2000);
    printf("Ballast: Stopping\n");
    fflush(stdout);
    simple_motor3.stop();
    sleep_ms(1000);
#endif

#ifdef TEST_ENCODER_TICKS
    // Encoder tick testing - motors stay stopped for manual rotation
    printf("=== ENCODER TICK TEST ===\n");
    fflush(stdout);
    printf("Motors are STOPPED. Manually rotate them to see tick counts.\n");
    fflush(stdout);
    printf("Motor1: Microgear (28 ticks/rev, 150:1 gear ratio)\n");
    fflush(stdout);
    printf("Motor2: Pololu (64 ticks/rev, 50:1 gear ratio)\n");
    fflush(stdout);
    printf("Rotate motors slowly to find mechanical limits...\n\n");
    fflush(stdout);
    
    // Ensure motors are stopped
    encoder_motor1.stop();
    encoder_motor2.stop();
#endif

#ifdef TEST_LIMITED_CONTROLLER
    // Limited controller testing - motors move between tick limits
    printf("=== LIMITED CONTROLLER TEST ===\n");
    fflush(stdout);
    printf("Motors will move between tick limits with automatic direction toggle.\n");
    fflush(stdout);
    printf("Motor1: Range -4000 to -5 ticks (starts going backward to -4000)\n");
    fflush(stdout);
    printf("Motor2: Range -4000 to -5 ticks (starts going backward to -4000)\n");
    fflush(stdout);
    printf("Speed controller active with automatic limit detection...\n\n");
    fflush(stdout);
#endif
    
    // Main testing loop with separated timing
    uint64_t last_time_control = to_ms_since_boot(get_absolute_time());      // Fast control loop (100ms)
    uint64_t last_time_slow_sensors = to_ms_since_boot(get_absolute_time()); // Slow sensors (500ms)
    uint64_t last_time_very_slow = to_ms_since_boot(get_absolute_time());    // Very slow sensors (1000ms)

    gpio_put(LED_PIN, 1); // Turn on the onboard LED to indicate the program is running

    while (true) {
        uint64_t current_time = to_ms_since_boot(get_absolute_time());

        // FAST CONTROL LOOP - 100ms (10Hz) - Critical for motor control
        if (current_time - last_time_control >= 100) {
            
#ifdef TEST_MOTOR
            // Alternate motor speeds every 2 seconds to test sensor performance with motors running
            static uint64_t motor_change_time = 0;
            static bool motor_high_speed = false;
            
            if (current_time - motor_change_time >= 2000) {  // Change every 2 seconds
                if (motor_high_speed) {
                    simple_motor1.moveFwd(3.0f);  // Low speed: 3%
                    simple_motor2.moveFwd(3.0f);
                    printf("Motors: 3%% speed\n");
                } else {
                    simple_motor1.moveFwd(7.0f);  // High speed: 7%
                    simple_motor2.moveFwd(7.0f);
                    printf("Motors: 7%% speed\n");
                }
                motor_high_speed = !motor_high_speed;
                motor_change_time = current_time;
            }
#endif
            
#ifdef TEST_LIGHT_SENSOR
            // Light sensor - fast analog reading, no timing constraints
            float brightness = light_sensor.read();
            float last_value = light_sensor.getValue();
            printf("Light: %.1f%% (Last: %.1f%%)\n", brightness, last_value);
#endif

#ifdef TEST_LIMIT_SWITCH
            // Limit switches - digital, instant reading
            bool switch1_state = limit_switch1.read();
            bool switch2_state = limit_switch2.read();
            
            bool switch1_stored = limit_switch1.getValue();
            bool switch2_stored = limit_switch2.getValue();
            
            printf("Switch1: %s | Switch2: %s\n",
                   limit_switch1.isPressed() ? "PRESSED" : "RELEASED",
                   limit_switch2.isPressed() ? "PRESSED" : "RELEASED");
#endif

#ifdef TEST_PRECISION_MOTOR
            // Motor control - needs consistent timing for PID
            precision_motor1.set_motor(-5.0f);
            precision_motor2.set_motor(5.0f);

            printf("Motor1: %.2f RPM (%.2f%%) | Motor2: %.2f RPM (%.2f%%)\n",
                   precision_motor1.get_filtered_rpm(), precision_motor1.get_control_output(),
                   precision_motor2.get_filtered_rpm(), precision_motor2.get_control_output());
            //Print ticks
            printf("Ticks1: %d | Ticks2: %d\n",
                   precision_motor1.get_encoder_ticks(),
                   precision_motor2.get_encoder_ticks());
#endif

#ifdef TEST_ENCODER_TICKS
            // Display encoder ticks for manual rotation testing
            // Calculate revolutions for both motors
            float revs1 = (float)encoder_motor1.get_encoder_ticks() / 28.0f;  // 28 ticks per rev
            float revs2 = (float)encoder_motor2.get_encoder_ticks() / 64.0f;  // 64 ticks per rev
            
            // Calculate output shaft revolutions (after gear reduction)
            float output_revs1 = revs1 / 150.0f;  // 150:1 gear ratio
            float output_revs2 = revs2 / 50.0f;   // 50:1 gear ratio
            
            // Calculate degrees
            float degrees1 = output_revs1 * 360.0f;
            float degrees2 = output_revs2 * 360.0f;
            
            printf("Motor1: %d ticks | %.3f revs | OUT: %.3f revs (%.1f°) | Motor2: %d ticks | %.3f revs | OUT: %.3f revs (%.1f°)\n",
                   encoder_motor1.get_encoder_ticks(), revs1, output_revs1, degrees1,
                   encoder_motor2.get_encoder_ticks(), revs2, output_revs2, degrees2);
#endif

#ifdef TEST_LIMITED_CONTROLLER
            // Limited controller - toggle direction when reaching tick limits
            static float desired_speed1 = -5.0f;  // Start going backward (negative speed)
            static float desired_speed2 = -5.0f;  // Start going backward (negative speed)
            static bool direction_changed1 = false;
            static bool direction_changed2 = false;
            
            // Get current tick counts
            int32_t ticks1 = limited_motor1.get_encoder_ticks();
            int32_t ticks2 = limited_motor2.get_encoder_ticks();
            
            // Motor 1 limit logic
            if (ticks1 <= -4000 && desired_speed1 < 0) {
                // Hit lower limit, change to positive speed
                desired_speed1 = 5.0f;
                direction_changed1 = true;
                printf("Motor1: Hit lower limit (-4000), reversing direction!\n");
                fflush(stdout);
            } else if (ticks1 >= -5 && desired_speed1 > 0) {
                // Hit upper limit, change to negative speed
                desired_speed1 = -5.0f;
                direction_changed1 = true;
                printf("Motor1: Hit upper limit (-5), reversing direction!\n");
                fflush(stdout);
            }
            
            // Motor 2 limit logic
            if (ticks2 <= -4000 && desired_speed2 < 0) {
                // Hit lower limit, change to positive speed
                desired_speed2 = 5.0f;
                direction_changed2 = true;
                printf("Motor2: Hit lower limit (-4000), reversing direction!\n");
                fflush(stdout);
            } else if (ticks2 >= -5 && desired_speed2 > 0) {
                // Hit upper limit, change to negative speed
                desired_speed2 = -5.0f;
                direction_changed2 = true;
                printf("Motor2: Hit upper limit (-5), reversing direction!\n");
                fflush(stdout);
            }
            
            // Apply speed control
            limited_motor1.set_motor(desired_speed1);
            limited_motor2.set_motor(desired_speed2);

            // Display status
            printf("Motor1: %d ticks | %.1f RPM (%.1f%%) | Target: %.1fRPM | Motor2: %d ticks | %.1f RPM (%.1f%%) | Target: %.1fRPM\n",
                   ticks1, limited_motor1.get_filtered_rpm(), limited_motor1.get_control_output(), desired_speed1,
                   ticks2, limited_motor2.get_filtered_rpm(), limited_motor2.get_control_output(), desired_speed2);
#endif
            
            last_time_control = current_time;
        }
        
        // SLOW SENSORS - 500ms (2Hz) - Time-dependent but not critical
        if (current_time - last_time_slow_sensors >= 500) {
            
#ifdef TEST_ULTRASONIC_SENSOR
            // Ultrasonic sensor - needs time for sound wave travel (~300ms recommended)
            long distance = ultrasonic.read();
            long last_dist = ultrasonic.getDistance();
            long safe_dist = ultrasonic.getLastKnownDistance();

            if (ultrasonic.isValidDistance(distance)) {
                printf("Distance: %ld cm (Last: %ld cm, Safe: %ld cm)\n", 
                       distance, last_dist, safe_dist);
            }
#endif
            
            last_time_slow_sensors = current_time;
        }
        
        // VERY SLOW SENSORS - 1000ms (1Hz) - Very time-intensive operations
        if (current_time - last_time_very_slow >= 1000) {
            
#ifdef TEST_PRESSURE_SENSOR
            // Pressure sensor - averaging takes ~500ms (10 samples × 50ms delay)
            if (pressure_sensor.isReady()) {
                // Quick single reading for immediate feedback
                long raw_reading = pressure_sensor.read();
                
                // Averaged reading only once per second to avoid blocking
                float averaged_reading = pressure_sensor.readAveraged(5);
                
                printf("Pressure - Raw: %ld | Averaged: %.0f\n", 
                       raw_reading, averaged_reading);
            } else {
                printf("Pressure sensor not ready\n");
            }
#endif
            
            last_time_very_slow = current_time;
        }
        
        sleep_ms(10);  // Small sleep to prevent excessive CPU usage
    }
    
    return 0;
}