#include "ROV.h"
#include <stdio.h>

ROV::ROV(uint thruster1_ena, uint thruster1_in1, uint thruster1_in2,
         uint thruster2_ena, uint thruster2_in1, uint thruster2_in2,
         uint ballast_ena, uint ballast_in1, uint ballast_in2, 
         uint ballast_enc_a, uint ballast_enc_b,
         uint light_sensor_pin, 
         uint pressure_out_pin, uint pressure_sck_pin)
    : thruster1(thruster1_ena, thruster1_in1, thruster1_in2),
      thruster2(thruster2_ena, thruster2_in1, thruster2_in2),
      ballast_motor(ballast_ena, ballast_in1, ballast_in2, ballast_enc_a, ballast_enc_b),
      light_sensor(light_sensor_pin),
      pressure_sensor(pressure_out_pin, pressure_sck_pin),
      current_depth(0.0f), current_luminosity(0.0f), 
      desired_depth(0.0f), depth_error(0.0f), automatic_mode(false) {
    
    printf("ROV initialized successfully!\n");
    printf("Thrusters: ENA1=%d, ENA2=%d | Ballast: ENA=%d | Sensors: Light=%d, Pressure=OUT%d/SCK%d\n", 
           thruster1_ena, thruster2_ena, ballast_ena, light_sensor_pin, pressure_out_pin, pressure_sck_pin);
}

float ROV::getDepth() {
    // Read pressure sensor and convert to depth
    float pressure_kPa = pressure_sensor.readAveraged(5); // Quick averaged reading (5 samples)
    
    // Convert pressure to depth (approximate: 1 kPa ≈ 10.197 cm of water)
    // Subtract atmospheric pressure (101.325 kPa) to get gauge pressure
    float gauge_pressure = pressure_kPa - 101.325f;
    
    // Convert to depth in meters (positive depth below surface)
    current_depth = (gauge_pressure > 0) ? gauge_pressure / PRESSURE_TO_DEPTH_FACTOR : 0.0f;
    
    return current_depth;
}

float ROV::getLuminosity() {
    current_luminosity = light_sensor.read();
    return current_luminosity;
}

void ROV::setDepth(float depth_setpoint) {
    desired_depth = depth_setpoint;
    printf("Depth setpoint set to: %.2f meters\n", desired_depth);
}

void ROV::controlDepth() {
    // Update current depth reading
    getDepth();
    
    // Calculate depth error
    depth_error = desired_depth - current_depth;
    
    if (automatic_mode) {
        // ========================================
        // AUTOMATIC DEPTH CONTROL (CASCADE LOOP)
        // ========================================
        
        // TODO: IMPLEMENT YOUR DEPTH CONTROL LAW HERE
        // Example structure:
        /*
        // Outer loop: Depth controller (Position Controller)
        // Input: depth_error (meters)
        // Output: ballast_speed_reference (RPM)
        
        // Your control law goes here:
        // float ballast_speed_reference = your_depth_controller_function(depth_error);
        
        // Inner loop: Speed controller (already implemented in PrecisionMotor)
        // The PrecisionMotor class handles the speed control automatically
        */
        
        // PLACEHOLDER: Simple proportional control for demonstration
        float kp_depth = 10.0f; // Proportional gain for depth control
        float ballast_speed_reference = kp_depth * depth_error;
        
        // Limit speed reference to reasonable range
        if (ballast_speed_reference > 100.0f) ballast_speed_reference = 100.0f;
        if (ballast_speed_reference < -100.0f) ballast_speed_reference = -100.0f;
        
        // Determine ballast direction based on error
        bool fill_ballast = (depth_error > 0); // Positive error = need to go deeper = fill ballast
        
        // Apply control to ballast motor
        adjustBallast(fill_ballast, fabsf(ballast_speed_reference));
        
        printf("AUTO MODE - Depth: %.2fm | Error: %.2fm | Ballast Speed: %.1f RPM | Direction: %s\n",
               current_depth, depth_error, fabsf(ballast_speed_reference), 
               fill_ballast ? "FILL" : "EMPTY");
               
    } else {
        // Manual mode - depth control is disabled
        printf("MANUAL MODE - Current Depth: %.2fm | Setpoint: %.2fm\n", 
               current_depth, desired_depth);
    }
}

void ROV::adjustBallast(bool fill_ballast, float speed_reference) {
    // This method handles both automatic and manual ballast control
    // Called by controlDepth() for automatic mode
    // Called by manualBallastControl() for manual mode
    
    if (automatic_mode) {
        // Automatic mode: Use provided speed reference from cascade controller
        if (fill_ballast) {
            // Fill ballast (go deeper) - forward direction
            ballast_motor.set_motor(speed_reference);
        } else {
            // Empty ballast (go shallower) - reverse direction
            ballast_motor.set_motor(-speed_reference);
        }
    } else {
        // Manual mode: Use constant speed
        if (fill_ballast) {
            ballast_motor.set_motor(MANUAL_BALLAST_SPEED);
        } else {
            ballast_motor.set_motor(-MANUAL_BALLAST_SPEED);
        }
    }
}

void ROV::setThrusterLevels(int thruster_levels[2]) {
    // Set thruster levels: thruster_levels[0] = left, thruster_levels[1] = right
    // Range: -100 to 100 (negative = reverse, positive = forward)
    
    // Clamp values to valid range
    int left_level = thruster_levels[0];
    int right_level = thruster_levels[1];
    
    if (left_level > 100) left_level = 100;
    if (left_level < -100) left_level = -100;
    if (right_level > 100) right_level = 100;
    if (right_level < -100) right_level = -100;
    
    // Control left thruster
    if (left_level > 0) {
        thruster1.moveFwd((float)left_level);
    } else if (left_level < 0) {
        thruster1.moveBckwd((float)(-left_level));
    } else {
        thruster1.stop();
    }
    
    // Control right thruster  
    if (right_level > 0) {
        thruster2.moveFwd((float)right_level);
    } else if (right_level < 0) {
        thruster2.moveBckwd((float)(-right_level));
    } else {
        thruster2.stop();
    }
    
    printf("Thrusters set - Left: %d%%, Right: %d%%\n", left_level, right_level);
}

void ROV::stop() {
    // Emergency stop - stop all motors immediately
    thruster1.stop();
    thruster2.stop();
    ballast_motor.stop();
    
    printf("ROV EMERGENCY STOP - All motors stopped!\n");
}

void ROV::setAutomaticMode(bool is_automatic) {
    automatic_mode = is_automatic;
    printf("ROV mode set to: %s\n", automatic_mode ? "AUTOMATIC" : "MANUAL");
}

bool ROV::isAutomaticMode() const {
    return automatic_mode;
}

void ROV::manualBallastControl(bool fill_ballast) {
    // Manual ballast control - only works in manual mode
    if (!automatic_mode) {
        adjustBallast(fill_ballast, MANUAL_BALLAST_SPEED);
        printf("Manual ballast control: %s at %.1f RPM\n", 
               fill_ballast ? "FILLING" : "EMPTYING", MANUAL_BALLAST_SPEED);
    } else {
        printf("Cannot use manual ballast control in automatic mode!\n");
    }
}

// Getter methods for monitoring
float ROV::getCurrentDepth() const {
    return current_depth;
}

float ROV::getDesiredDepth() const {
    return desired_depth;
}

float ROV::getDepthError() const {
    return depth_error;
}

float ROV::getCurrentLuminosity() const {
    return current_luminosity;
}