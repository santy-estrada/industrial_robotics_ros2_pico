#include "motor/Motor.h"
#include "sensor/TEMT6000.h"
#include "sensor/LimitSwitch.h"
#include "sensor/MPS20N0040D.h"
#include "sensor/SOIL_MOIST.h"
#include "rov/ROV.h"
#include <stdio.h>

// Test configuration defines - comment/uncomment to enable/disable tests
// #define TEST_MOTOR
// #define TEST_BALLAST
// #define TEST_LIMITED_BALLAST
// #define TEST_LIMITED_BALLAST_1
// #define TEST_LIGHT_SENSOR
// #define TEST_LIMIT_SWITCH
// #define TEST_PRESSURE_SENSOR
// #define TEST_MOISTURE_SENSOR
#define TEST_ROV

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

#ifdef TEST_LIMIT_SWITCH
    // Create limit switches on different GPIO pins
    LimitSwitch limit_switch1(6);  // GPIO6
    LimitSwitch limit_switch2(7);  // GPIO7
#endif

#ifdef TEST_PRESSURE_SENSOR
    // Create pressure sensor with HX710B amplifier
    MPS20N0040D pressure_sensor(11, 10);  // OUT=GPIO10, SCK=GPIO11
    pressure_sensor.setGain(128);  // Set gain to 128 (default)
#endif

#ifdef TEST_MOISTURE_SENSOR
    // Create moisture sensor with digital and analog outputs
    SOIL_MOIST moisture_sensor(26, 28);  // Digital=GPIO26, Analog=GPIO28 (ADC2)
#endif

#ifdef TEST_ROV
    // Create ROV with all components
    // Thrust motors: Motor1=GPIO(0,1,2), Motor2=GPIO(3,4,5)
    // Ballast motor: GPIO(21,19,20)
    // Limit switches: Full=GPIO7, Empty=GPIO8
    // Moisture sensor: Digital=GPIO9, Analog=GPIO28
    // Pressure sensor: OUT=GPIO10, SCK=GPIO11
    ROV rov(0, 1, 2,      // Thrust motor 1
            3, 4, 5,      // Thrust motor 2
            21, 20, 19,   // Ballast motor
            8, 7,         // Full and Empty limit switches
            26, 28,        // Moisture sensor
            11, 10);      // Pressure sensor
    
    // Set placeholder depth conversion (user should calibrate)
    rov.setDepthConversion(0.0f, 1.0f);  // offset=0, factor=1.0
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

#if defined(TEST_LIMITED_BALLAST) || defined(TEST_LIMITED_BALLAST_1)
    // Create ballast motor and limit switches
    Motor ballast_motor(21, 19, 20);  // ENA=21, IN1=19, IN2=20
    LimitSwitch limit_switch1(7);     // GPIO7 - End switch 1 (Prevents fwd)
    LimitSwitch limit_switch2(8);     // GPIO8 - End switch 2 (Prevents bckwd)
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


#if defined(TEST_LIMITED_BALLAST) || defined(TEST_LIMITED_BALLAST_1)
    // Limited ballast testing - motor with limit switch protection
    printf("=== LIMITED BALLAST TEST ===\n");
    fflush(stdout);
    printf("Ballast motor with limit switch protection.\n");
    fflush(stdout);
    printf("Motor will cycle: Forward 2s -> Stop 1s -> Backward 2s -> Stop 1s\n");
    fflush(stdout);
    printf("Switch1 (GPIO7): Prevents FORWARD motion when pressed\n");
    fflush(stdout);
    printf("Switch2 (GPIO8): Prevents BACKWARD motion when pressed\n");
    fflush(stdout);
    printf("Starting ballast cycle...\n\n");
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

#ifdef TEST_MOISTURE_SENSOR
            // Moisture sensor - digital and analog readings
            bool moisture_detected = moisture_sensor.readDigital();
            float moisture_percent = moisture_sensor.readAnalog();
            
            bool last_digital = moisture_sensor.getDigitalValue();
            float last_analog = moisture_sensor.getAnalogValue();
            
            // Display with warning if moisture detected
            if (moisture_sensor.isMoistureDetected()) {
                printf("*** MOISTURE ALERT *** Digital: %s | Analog: %.1f%% (Last: %.1f%%)\n",
                       moisture_detected ? "WET" : "DRY",
                       moisture_percent, last_analog);
            } else {
                printf("Moisture: Digital: %s | Analog: %.1f%% (Last: %.1f%%)\n",
                       moisture_detected ? "WET" : "DRY",
                       moisture_percent, last_analog);
            }
#endif

#ifdef TEST_ROV
            // ROV comprehensive test with state machine
            static uint64_t rov_test_time = 0;
            static int rov_test_state = 0;
            
            // State machine timing (each state lasts 3 seconds)
            if (current_time - rov_test_time >= 10000) {
                rov_test_state = (rov_test_state + 1) % 10;  // 10 states
                rov_test_time = current_time;
                printf("\n=== ROV TEST STATE %d ===\n", rov_test_state);
            }
            
            // Read all sensors
            float depth = rov.getDepth();
            bool moisture = rov.isMoistureDetected();
            bool full_sw = rov.isFullSwitchPressed();
            bool empty_sw = rov.isEmptySwitchPressed();
            
            // Execute test states
            switch(rov_test_state) {
                case 0:  // Test thrust motor 1 forward (0.2)
                    printf("Testing Thrust Motor 1 (Right): 20%% forward\n");
                    rov.setThrustMotor1(0.2f);
                    rov.setThrustMotor2(0.0f);
                    rov.setBallast(0.0f);
                    break;
                    
                case 1:  // Test thrust motor 2 forward (0.2)
                    printf("Testing Thrust Motor 2 (Left): 20%% forward\n");
                    rov.setThrustMotor1(0.0f);
                    rov.setThrustMotor2(0.2f);
                    rov.setBallast(0.0f);
                    break;
                    
                case 2:  // Test both thrust motors forward (0.3)
                    printf("Testing Both Thrust Motors: 30%% forward\n");
                    rov.setThrust(0.3f);
                    rov.setBallast(0.0f);
                    break;
                    
                case 3:  // Test both thrust motors backward (-0.2)
                    printf("Testing Both Thrust Motors: 20%% backward\n");
                    rov.setThrust(-0.2f);
                    rov.setBallast(0.0f);
                    break;
                    
                case 4:  // Test ballast low power (0.3 exponential = ~9%)
                    printf("Testing Ballast: Low power 0.3 (exponential = ~9%%)\n");
                    rov.setThrust(0.0f);
                    rov.setBallast(0.3f);
                    break;
                    
                case 5:  // Test ballast medium power (0.7 exponential = ~49%)
                    printf("Testing Ballast: Medium power 0.7 (exponential = ~49%%)\n");
                    rov.setThrust(0.0f);
                    rov.setBallast(0.7f);
                    rov.calibrateZeroDepth();  // Calibrate zero depth here
                    break;
                    
                case 6:  // Test ballast full power (1.0 exponential = 100%)
                    printf("Testing Ballast: Full power 1.0 (exponential = 100%%)\n");
                    rov.setThrust(0.0f);
                    rov.setBallast(1.0f);
                    break;
                    
                case 7:  // Test ballast reverse (-0.5 exponential = -25%)
                    printf("Testing Ballast: Reverse -0.5 (exponential = -25%%)\n");
                    rov.setThrust(0.0f);
                    rov.setBallast(-0.5f);
                    break;

                case 8:  // Stop all motors
                    printf("Stopping all motors\n");
                    rov.stop();
                    break;
                
                case 9:  // Emergency emerge - full ballast in reverse
                    printf("Emergency Emerge: Full ballast in reverse\n");
                    rov.emergencyEmerge();
                    rov.calibrateZeroDepth();  // Calibrate zero depth here
                    break;
            }
            
            // Display sensor status
            printf("Depth: %.2f | Moisture: %s | Full SW: %s | Empty SW: %s\n",
                   depth,
                   moisture ? "DETECTED" : "OK",
                   full_sw ? "PRESSED" : "OK",
                   empty_sw ? "PRESSED" : "OK");
            
            // Warning messages
            if (moisture) {
                printf("*** WARNING: MOISTURE DETECTED - THRUST MOTORS BLOCKED ***\n");
            }
            if (full_sw) {
                printf("*** WARNING: FULL SWITCH - CANNOT FILL BALLAST ***\n");
            }
            if (empty_sw) {
                printf("*** WARNING: EMPTY SWITCH - CANNOT EMPTY BALLAST ***\n");
            }
#endif

#ifdef TEST_LIMITED_BALLAST
            // Limited ballast - cycle through movements with limit switch protection
            static uint64_t ballast_cycle_time = 0;
            static int ballast_state = 0;  // 0=forward, 1=stop1, 2=backward, 3=stop2
            
            // Read limit switches
            bool switch1_pressed = limit_switch1.read();  // Prevents BACKWARD
            bool switch2_pressed = limit_switch2.read();  // Prevents FORWARD
            
            // Check if it's time to change state (cycle timing)
            uint64_t state_duration = 0;
            switch(ballast_state) {
                case 0: state_duration = 2000; break;  // Forward for 2s
                case 1: state_duration = 1000; break;  // Stop for 1s
                case 2: state_duration = 2000; break;  // Backward for 2s
                case 3: state_duration = 1000; break;  // Stop for 1s
            }
            
            if (current_time - ballast_cycle_time >= state_duration) {
                ballast_state = (ballast_state + 1) % 4;  // Cycle through states
                ballast_cycle_time = current_time;
            }
            
            // Execute state with limit switch protection
            switch(ballast_state) {
                case 0:  // Forward (fill)
                    if (switch2_pressed) {
                        ballast_motor.stop();
                        printf("Ballast: BLOCKED - Switch2 pressed, cannot move forward!\n");
                        fflush(stdout);
                    } else if (switch1_pressed) {
                        ballast_motor.stop();
                        printf("Ballast: BLOCKED - Switch1 pressed, cannot move forward!\n");
                        fflush(stdout);
                    } else {
                        ballast_motor.moveFwd(70.0f);
                        printf("Ballast: 70%% forward (fill) | SW1:%s SW2:%s\n",
                               switch1_pressed ? "PRESSED" : "OK",
                               switch2_pressed ? "PRESSED" : "OK");
                    }
                    break;
                    
                case 1:  // Stop
                    ballast_motor.stop();
                    printf("Ballast: Stopped | SW1:%s SW2:%s\n",
                           switch1_pressed ? "PRESSED" : "OK",
                           switch2_pressed ? "PRESSED" : "OK");
                    break;
                    
                case 2:  // Backward (empty)
                    if (switch1_pressed) {
                        ballast_motor.stop();
                        printf("Ballast: BLOCKED - Switch1 pressed, cannot move backward!\n");
                        fflush(stdout);
                    } else if (switch2_pressed) {
                        ballast_motor.stop();
                        printf("Ballast: BLOCKED - Switch2 pressed, cannot move backward!\n");
                        fflush(stdout);
                    } else {
                        ballast_motor.moveBckwd(70.0f);
                        printf("Ballast: 70%% backward (empty) | SW1:%s SW2:%s\n",
                               switch1_pressed ? "PRESSED" : "OK",
                               switch2_pressed ? "PRESSED" : "OK");
                    }
                    break;
                    
                case 3:  // Stop
                    ballast_motor.stop();
                    printf("Ballast: Stopped | SW1:%s SW2:%s\n",
                           switch1_pressed ? "PRESSED" : "OK",
                           switch2_pressed ? "PRESSED" : "OK");
                    break;
            }
#endif

#ifdef TEST_LIMITED_BALLAST_1

            static bool ballast_state = false;  // false=forward, true=backward
            static uint64_t last_switch_time = 0;  // Track last switch press time
            static const uint64_t DEBOUNCE_DELAY = 2000;  // 2000ms non-blocking delay
            
            // Read limit switches
            bool switch1_pressed = limit_switch1.read();  // Prevents BACKWARD
            bool switch2_pressed = limit_switch2.read();  // Prevents FORWARD

            // Only change direction if enough time has passed since last change (debounce)
            if ((switch1_pressed || switch2_pressed) && 
                (current_time - last_switch_time >= DEBOUNCE_DELAY)) {
                ballast_state = !ballast_state; // Change direction if any switch is pressed
                last_switch_time = current_time;  // Update last switch time
                printf("Direction changed! Now going %s\n", 
                       ballast_state ? "BACKWARD" : "FORWARD");
                fflush(stdout);
            }

            if (!ballast_state) {
                ballast_motor.moveFwd(70.0f);
                printf("Ballast: 70%% forward (empty) | SW1:%s SW2:%s\n",
                       switch1_pressed ? "PRESSED" : "OK",
                       switch2_pressed ? "PRESSED" : "OK");
            } else {
                ballast_motor.moveBckwd(70.0f);
                printf("Ballast: 70%% backward (fill) | SW1:%s SW2:%s\n",
                       switch1_pressed ? "PRESSED" : "OK",
                       switch2_pressed ? "PRESSED" : "OK");
            }
#endif

            
            last_time_control = current_time;
        }
        
        // SLOW SENSORS - 500ms (2Hz) - Time-dependent but not critical
        if (current_time - last_time_slow_sensors >= 500) {
            
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