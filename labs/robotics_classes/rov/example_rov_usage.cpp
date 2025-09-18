#include "rov/ROV.h"
#include <stdio.h>

// ROV Example Usage - Demonstrates both manual and automatic modes
int main() {
    stdio_init_all();
    
    // Create ROV with pin configuration
    ROV rov(0,1,2,      // Thruster1: ENA=0, IN1=1, IN2=2
            3,4,5,      // Thruster2: ENA=3, IN1=4, IN2=5  
            6,7,8,9,10, // Ballast: ENA=6, IN1=7, IN2=8, ENC_A=9, ENC_B=10
            28,         // Light sensor: GPIO28 (ADC2)
            11,12);     // Pressure sensor: OUT=11, SCK=12
    
    printf("ROV Example Started\n");
    sleep_ms(2000);
    
    // === MANUAL MODE DEMONSTRATION ===
    printf("\n=== MANUAL MODE TEST ===\n");
    rov.setAutomaticMode(false);
    
    // Test thrusters
    printf("Testing thrusters...\n");
    int forward[2] = {50, 50};      // Both forward
    int turn_left[2] = {30, 70};    // Turn left
    int reverse[2] = {-40, -40};    // Both reverse
    
    rov.setThrusterLevels(forward);
    sleep_ms(3000);
    
    rov.setThrusterLevels(turn_left);
    sleep_ms(2000);
    
    rov.setThrusterLevels(reverse);
    sleep_ms(2000);
    
    // Stop thrusters
    int stop_thrusters[2] = {0, 0};
    rov.setThrusterLevels(stop_thrusters);
    
    // Test manual ballast control
    printf("Testing manual ballast control...\n");
    rov.manualBallastControl(true);   // Fill ballast
    sleep_ms(3000);
    rov.manualBallastControl(false);  // Empty ballast  
    sleep_ms(3000);
    
    // === AUTOMATIC MODE DEMONSTRATION ===
    printf("\n=== AUTOMATIC MODE TEST ===\n");
    rov.setAutomaticMode(true);
    
    // Set initial depth target
    rov.setDepth(1.5f);
    printf("Target depth set to 1.5 meters\n");
    
    // Main control loop with proper timing
    uint64_t last_control_time = to_ms_since_boot(get_absolute_time());
    uint64_t last_sensor_update = to_ms_since_boot(get_absolute_time());
    uint64_t last_phase_change = to_ms_since_boot(get_absolute_time());
    
    const uint64_t CONTROL_PERIOD_MS = 100;    // 100ms control loop
    const uint64_t SENSOR_PERIOD_MS = 1000;    // 1s sensor updates
    const uint64_t PHASE_PERIOD_MS = 15000;    // 15s phase changes
    
    int demonstration_phase = 0;
    
    while (true) {
        uint64_t current_time = to_ms_since_boot(get_absolute_time());
        
        // FAST CONTROL LOOP (100ms) - Critical for depth control
        if (current_time - last_control_time >= CONTROL_PERIOD_MS) {
            
            // Run depth control (this calls adjustBallast internally)
            rov.controlDepth();
            
            last_control_time = current_time;
        }
        
        // SENSOR UPDATES (1000ms) - Update display and sensors
        if (current_time - last_sensor_update >= SENSOR_PERIOD_MS) {
            
            // Update sensors
            float depth = rov.getDepth();
            float luminosity = rov.getLuminosity();
            
            // Display status
            printf("Status - Depth: %.2fm (Target: %.2fm) | Light: %.1f%% | Error: %.3fm\n",
                   depth, rov.getDesiredDepth(), luminosity, rov.getDepthError());
            
            last_sensor_update = current_time;
        }
        
        // DEMONSTRATION PHASES (15000ms) - Change behavior every 15 seconds
        if (current_time - last_phase_change >= PHASE_PERIOD_MS) {
            
            switch (demonstration_phase) {
                case 0:
                    printf("\n=== Phase 1: Target 2.0m depth ===\n");
                    rov.setDepth(2.0f);
                    break;
                    
                case 1:
                    printf("\n=== Phase 2: Target 0.5m depth (shallow) ===\n");
                    rov.setDepth(0.5f);
                    break;
                    
                case 2:
                    printf("\n=== Phase 3: Target 3.0m depth (deep) ===\n");
                    rov.setDepth(3.0f);
                    break;
                    
                case 3:
                    printf("\n=== Phase 4: Manual mode override ===\n");
                    rov.setAutomaticMode(false);
                    rov.manualBallastControl(true); // Fill ballast manually
                    break;
                    
                case 4:
                    printf("\n=== Phase 5: Back to automatic - Target 1.0m ===\n");
                    rov.setAutomaticMode(true);
                    rov.setDepth(1.0f);
                    break;
                    
                default:
                    printf("\n=== Demo Complete - Emergency Stop ===\n");
                    rov.stop();
                    printf("ROV demonstration completed successfully!\n");
                    return 0;
            }
            
            demonstration_phase++;
            last_phase_change = current_time;
        }
        
        sleep_ms(10); // Small delay to prevent excessive CPU usage
    }
    
    return 0;
}