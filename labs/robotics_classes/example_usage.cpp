#include "motor/Motor.h"
#include "motor/PrecisionMotor.h"
#include "sensor/TEMT6000.h"
#include "sensor/HCSR05.h"
#include "sensor/LimitSwitch.h"
#include "sensor/MPS20N0040D.h"
#include <stdio.h>

// Test configuration defines - comment/uncomment to enable/disable tests
#define TEST_MOTOR
// #define TEST_PRECISION_MOTOR
// #define TEST_LIGHT_SENSOR
// #define TEST_ULTRASONIC_SENSOR
// #define TEST_LIMIT_SWITCH
#define TEST_PRESSURE_SENSOR

// Example usage of the motor and sensor classes
int main() {
    stdio_init_all();
    
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
    MPS20N0040D pressure_sensor(10, 11);  // OUT=GPIO10, SCK=GPIO11
    pressure_sensor.setGain(128);  // Set gain to 128 (default)
#endif

#ifdef TEST_MOTOR
    // Create a simple motor
    Motor simple_motor1(0, 1, 2);  // ENA=0, IN1=1, IN2=2
    Motor simple_motor2(3, 4, 5);  // ENA=3, IN1=4, IN2=5
#endif
    
#ifdef TEST_PRECISION_MOTOR
    // Create precision motors with different configurations
    PrecisionMotor precision_motor1(16, 18, 17, 12, 13, 28, 150.0f);   // Microgear motor
    PrecisionMotor precision_motor2(21, 19, 20, 14, 15, 64, 50.0f);    // Pololu motor
#endif
    
    printf("Components initialized successfully!\n");
    
#ifdef TEST_MOTOR
    // Test simple motor1
    simple_motor1.moveFwd(50.0f);  // Move forward at 50% power
    sleep_ms(2000);
    simple_motor1.moveBckwd(30.0f); // Move backward at 30% power
    sleep_ms(2000);
    simple_motor1.stop();
    sleep_ms(1000);
    // Test simple motor2
    simple_motor2.moveFwd(50.0f);  // Move forward at 70% power
    sleep_ms(2000);
    simple_motor2.moveBckwd(20.0f); // Move backward at 20% power
    sleep_ms(2000);
    simple_motor2.stop();
    sleep_ms(1000);
    //Test both motors together
    simple_motor1.moveFwd(60.0f);
    simple_motor2.moveFwd(60.0f);
    sleep_ms(2000);
    simple_motor1.stop();
    simple_motor2.stop();
    sleep_ms(1000); 
#endif
    
    // Main testing loop with separated timing
    uint64_t last_time_control = to_ms_since_boot(get_absolute_time());      // Fast control loop (100ms)
    uint64_t last_time_slow_sensors = to_ms_since_boot(get_absolute_time()); // Slow sensors (500ms)
    uint64_t last_time_very_slow = to_ms_since_boot(get_absolute_time());    // Very slow sensors (1000ms)
    
    while (true) {
        uint64_t current_time = to_ms_since_boot(get_absolute_time());

        // FAST CONTROL LOOP - 100ms (10Hz) - Critical for motor control
        if (current_time - last_time_control >= 100) {
            
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
            precision_motor1.set_motor(50.0f);
            precision_motor2.set_motor(-30.0f);

            printf("Motor1: %.2f RPM (%.2f%%) | Motor2: %.2f RPM (%.2f%%)\n",
                   precision_motor1.get_filtered_rpm(), precision_motor1.get_control_output(),
                   precision_motor2.get_filtered_rpm(), precision_motor2.get_control_output());
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
                float averaged_reading = pressure_sensor.readAveraged(10);
                
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