#include "Joint.h"
#include <pico/stdlib.h>

// Constructor (threshold controller)
Joint::Joint(char joint_type, float min_lim, float max_lim, 
             PrecisionMotor* precision_motor, float gear_ratio_value,
             LimitSwitch* limit_min, LimitSwitch* limit_max, float resistance_value)
    : type(joint_type), min_limit(min_lim), max_limit(max_lim),
      current_position(0.0f), desired_position(0.0f), current_speed(0.0f),
      motor(precision_motor), gear_ratio(gear_ratio_value),
      limit_switch_min(limit_min), limit_switch_max(limit_max),
      origin_set(false), resistance(resistance_value),
      use_pi_controller(false), kp_pos(0.0f), ti_pos(0.0f), 
      q0_pos(0.0f), q1_pos(0.0f), speed_scale_factor(1.0f) {
    
    // Initialize error array for PI controller (not used with threshold controller)
    error_pos[0] = 0.0f;
    error_pos[1] = 0.0f;
    
    printf("Joint created: Type=%c, Limits=[%.2f, %.2f], GearRatio=%.2f, Resistance=%.2f, Controller=Threshold\n", 
           type, min_limit, max_limit, gear_ratio, resistance);
}

// Constructor (PI controller)
Joint::Joint(char joint_type, float min_lim, float max_lim, 
             PrecisionMotor* precision_motor, float gear_ratio_value,
             LimitSwitch* limit_min, LimitSwitch* limit_max, float resistance_value,
             float kp, float ti)
    : type(joint_type), min_limit(min_lim), max_limit(max_lim),
      current_position(0.0f), desired_position(0.0f), current_speed(0.0f),
      motor(precision_motor), gear_ratio(gear_ratio_value),
      limit_switch_min(limit_min), limit_switch_max(limit_max),
      origin_set(false), resistance(resistance_value),
      use_pi_controller(true), kp_pos(kp), ti_pos(ti), speed_scale_factor(1.0f) {
    
    // Initialize error array for PI controller
    error_pos[0] = 0.0f;
    error_pos[1] = 0.0f;
    
    // Calculate PI parameters
    calculate_pi_parameters();
    
    printf("Joint created: Type=%c, Limits=[%.2f, %.2f], GearRatio=%.2f, Resistance=%.2f, Controller=PI(Kp=%.2f, Ti=%.2f)\n", 
           type, min_limit, max_limit, gear_ratio, resistance, kp_pos, ti_pos);
}

// Destructor
Joint::~Joint() {
    // Note: We don't delete motor or limit switches as they may be used elsewhere
    printf("Joint destroyed\n");
}

// Setters
void Joint::setMinLimit(float min_lim) {
    min_limit = min_lim;
    printf("Joint min limit set to %.2f\n", min_limit);
}

void Joint::setMaxLimit(float max_lim) {
    max_limit = max_lim;
    printf("Joint max limit set to %.2f\n", max_limit);
}

bool Joint::setDesiredPosition(float desired_pos) {
    // Check if desired position is within limits
    if (desired_pos < min_limit || desired_pos > max_limit) {
        printf("WARNING: Desired position %.2f is outside limits [%.2f, %.2f]\n", 
               desired_pos, min_limit, max_limit);
        return false;
    }
    
    desired_position = desired_pos;
    printf("Joint desired position set to %.2f\n", desired_position);
    return true;
}

void Joint::setOrigin() {
    if (motor != nullptr) {
        // Set current encoder position as origin (0°)
        int32_t current_ticks = motor->get_encoder_ticks();
        motor->reset_encoder_ticks();
        origin_set = true;
        
        printf("Origin set at current position (encoder ticks: %d)\n", current_ticks);
        update_position();  // Recalculate position with new origin
    } else {
        printf("ERROR: Cannot set origin - motor is null\n");
    }
}


bool Joint::calibrateOrigin() {
    if (motor == nullptr) {
        printf("ERROR: Cannot calibrate origin - motor is null\n");
        return false;
    }
    
    if (limit_switch_min == nullptr || limit_switch_max == nullptr) {
        printf("ERROR: Cannot calibrate origin - limit switches are null\n");
        return false;
    }
    
    printf("=== Starting Joint Origin Calibration ===\n");
    printf("Joint will move between limits 3 times to find center position\n");
    
    const float calibration_speed = 20.0f; // RPM for calibration movement
    const int num_measurements = 3;
    float min_positions[num_measurements]; // Store min limit positions
    float max_positions[num_measurements]; // Store max limit positions
    
    // Temporary disable origin for calibration
    bool original_origin_set = origin_set;
    origin_set = false;
    
    for (int i = 0; i < num_measurements; i++) {
        printf("\n--- Calibration cycle %d/%d ---\n", i + 1, num_measurements);
        
        // 1. Move to minimum limit
        printf("Moving to minimum limit...\n");
        
        while (true) {
            limit_switch_min->read();
            if (limit_switch_min->isPressed()) {
                printf("Minimum limit reached\n");
                break;
            }
            motor->set_motor(-calibration_speed);
            sleep_ms((int)(motor->get_dt() * 1000));  // Use motor's dt for sleep interval
        }
        
        motor->set_motor(0.0f);
        motor->stop();
        sleep_ms(500);
        
        min_positions[i] = motor->get_position_degrees();
        printf("Min limit at: %.2f motor degrees\n", min_positions[i]);
        
        // 2. Move to maximum limit
        printf("Moving to maximum limit...\n");
        
        while (true) {
            limit_switch_max->read();
            if (limit_switch_max->isPressed()) {
                printf("Maximum limit reached\n");
                break;
            }
            motor->set_motor(calibration_speed);
            sleep_ms((int)(motor->get_dt() * 1000));  // Use motor's dt for sleep interval
        }
        
        motor->set_motor(0.0f);
        motor->stop();
        sleep_ms(500);

        max_positions[i] = motor->get_position_degrees();
        printf("Max limit at: %.2f motor degrees\n", max_positions[i]);
        
        float range_motor_degrees = fabs(max_positions[i] - min_positions[i]);
        float range_joint_degrees = range_motor_degrees / gear_ratio;
        
        printf("Measurement %d: Range = %.2f joint degrees (%.2f motor degrees)\n", 
               i + 1, range_joint_degrees, range_motor_degrees);
        
        // Validation check
        if (range_joint_degrees <= 1.83 * fabs(max_limit)) {
            printf("ERROR: Invalid range measurement, repeating...\n");
            i--; // Repeat this measurement
        }
    }
    
    // Calculate average positions
    float avg_min_pos = 0.0f, avg_max_pos = 0.0f;
    for (int i = 0; i < num_measurements; i++) {
        avg_min_pos += min_positions[i];
        avg_max_pos += max_positions[i];
    }
    avg_min_pos /= num_measurements;
    avg_max_pos /= num_measurements;
    
    float average_range_motor = fabs(avg_max_pos - avg_min_pos);
    float average_range_joint = average_range_motor / gear_ratio;
    
    printf("\n--- Calibration Results ---\n");
    printf("Average min position: %.2f motor degrees\n", avg_min_pos);
    printf("Average max position: %.2f motor degrees\n", avg_max_pos);
    printf("Average range: %.2f joint degrees (%.2f motor degrees)\n", 
           average_range_joint, average_range_motor);
    
    // Calculate center position
    float center_motor_degrees = (avg_min_pos + avg_max_pos) / 2.0f;
    
    printf("\nCalculated center position: %.2f motor degrees\n", center_motor_degrees);
    
    // Move to center using the reusable method
    move_to_position_internal(center_motor_degrees, calibration_speed);
    
    printf("Reached center position\n");
    
    // Set this position as the new origin
    setOrigin();  // Set origin at current position
    // Update position to reflect new origin
    update_position();
    
    printf("\n=== Origin Calibration Complete ===\n");
    printf("New origin set at center position\n");
    printf("Joint range: ±%.2f degrees\n", average_range_joint / 2.0f);
    printf("Current position: %.2f degrees\n", getCurrentPosition());
    
    // // Update the joint limits based on measured range
    // float half_range = average_range_joint / 2.0f;
    // min_limit = -half_range;
    // max_limit = half_range;
    // printf("Updated joint limits to [%.2f, %.2f] degrees\n", min_limit, max_limit);
    
    return true;
}

// Getters
char Joint::getType() const {
    return type;
}

float Joint::getMinLimit() const {
    return min_limit;
}

float Joint::getMaxLimit() const {
    return max_limit;
}

float Joint::getCurrentPosition() {
    update_position();
    return current_position;
}

float Joint::getDesiredPosition() const {
    return desired_position;
}

float Joint::getCurrentSpeed() {
    update_speed();
    return current_speed;
}

float Joint::getGearRatio() const {
    return gear_ratio;
}

float Joint::getErrorPosition() const {
    return error_pos[0];  // Return current error from error vector
}

bool Joint::isOriginSet() const {
    return origin_set;
}


// Private helper methods
void Joint::update_position() {
    if (motor == nullptr) return;
    
    // Get motor's position in degrees
    float motor_position = motor->get_position_degrees();

    // Convert to joint position using gear ratio
    float joint_revolutions = motor_position / gear_ratio;

    // Already in degrees (for revolute) or appropriate units (for prismatic)
    if (type == 'R') {
        current_position = joint_revolutions;
    } else if (type == 'P') {
        // For prismatic joints, this would need a different conversion
        // For now, assume 1 revolution = 1mm travel
        current_position = joint_revolutions;
    }
}

void Joint::update_speed() {
    if (motor == nullptr) return;
    
    // Get motors speed in rpm and convert to joint speed
    float motor_rpm = motor->get_filtered_rpm();
    float joint_revs_per_sec = motor_rpm / (60.0f * gear_ratio);

    if (type == 'R') {
        current_speed = joint_revs_per_sec * 360.0f;  // degrees per second
    } else if (type == 'P') {
        current_speed = joint_revs_per_sec;  // units per second
    }

}

bool Joint::check_limits() {
    // Check limit switches first
    bool min_switch_pressed = false;
    bool max_switch_pressed = false;
    
    if (limit_switch_min != nullptr) {
        min_switch_pressed = limit_switch_min->read();  // Update switch state
    }
    
    if (limit_switch_max != nullptr) {
        max_switch_pressed = limit_switch_max->read();  // Update switch state
    }
    
    // Check position limits
    bool position_below_min = current_position < min_limit;
    bool position_above_max = current_position > max_limit;
    
    // Print warnings if limits are violated
    if (min_switch_pressed) {
        printf("WARNING: Minimum limit switch pressed!\n");
    }
    if (max_switch_pressed) {
        printf("WARNING: Maximum limit switch pressed!\n");
    }
    if (position_below_min) {
        printf("WARNING: Position %.2f below minimum limit %.2f!\n", current_position, min_limit);
    }
    if (position_above_max) {
        printf("WARNING: Position %.2f above maximum limit %.2f!\n", current_position, max_limit);
    }
    
    // Return true if any limit is violated
    return min_switch_pressed || max_switch_pressed || position_below_min || position_above_max;
}

float Joint::position_control() {
    return position_control_internal(current_position, desired_position);
}

void Joint::calculate_pi_parameters() {
    if (motor == nullptr) {
        printf("ERROR: Cannot calculate PI parameters - motor is null\n");
        q0_pos = 0.0f;
        q1_pos = 0.0f;
        return;
    }
    
    // Get sampling time from the motor (default is 0.1s)
    float Ts = motor->get_dt();
    
    // Calculate PI coefficients using the provided formulas
    q0_pos = kp_pos * (1.0f + Ts / (2.0f * ti_pos));
    q1_pos = kp_pos * (Ts / (2.0f * ti_pos) - 1.0f);
    
    printf("PI Parameters calculated: Ts=%.3f, q0_pos=%.4f, q1_pos=%.4f\n", Ts, q0_pos, q1_pos);
}

float Joint::position_control_internal(float current_pos, float desired_pos) {
    // Update error vector - always shift history and calculate new error
    error_pos[1] = error_pos[0];  // e(k-1) = e(k) 
    error_pos[0] = desired_pos - current_pos;  // e(k) = current error
    const float tolerance = 0.15f;  // degrees or mm

    
    float position_control_output = 0.0f;
    
    if (use_pi_controller) {
        // Check tolerance FIRST for PI controller
        if (fabs(error_pos[0]) < tolerance) {
            position_control_output = 0.0f;  // Stop motor when within tolerance
            // Reset error history when within tolerance to prevent integral windup
            error_pos[1] = 0.0f;
        } else {
            // PI Controller implementation - uses both error_pos[0] and error_pos[1]
            position_control_output = q0_pos * error_pos[0] + q1_pos * error_pos[1];
            
            // Apply saturation limits to prevent excessive control output
            const float max_control_output = 50.0f;  // Maximum control output
            if (position_control_output > max_control_output) {
                position_control_output = max_control_output;
            } else if (position_control_output < -max_control_output) {
                position_control_output = -max_control_output;
            }

            // Apply minimum control output to overcome static friction (only when not in tolerance)
            if (fabs(position_control_output) < 10.0f && fabs(error_pos[0]) >= tolerance) {
                position_control_output = (position_control_output < 0) ? -10.0f : 10.0f;
            }
        }
        
    } else {
        // Threshold Controller - uses only error_pos[0] (current error)
        float error = error_pos[0];  // Use current error from vector
        
        if (fabs(error) < tolerance) {
            position_control_output = 0.0f;
            // Reset error history when stopped to ensure clean start on next movement
            error_pos[1] = 0.0f;
        } else if (fabs(error) < 2) {
            position_control_output = (error < 0) ? -resistance : resistance;
        } else if (fabs(error) < 10) {
            position_control_output = (error < 0) ? -7 : 7;
        } else if (fabs(error) < 15) {
            position_control_output = (error < 0) ? -10 : 10;
        } else {
            position_control_output = (error < 0) ? -12 : 12;
        }
    }

    // Apply speed scaling for synchronized motion
    return position_control_output * speed_scale_factor;
}

void Joint::move_to_position_internal(float target_motor_degrees, float max_speed) {
    printf("Moving to %.2f motor degrees...\n", target_motor_degrees);
    int cont = 0;
    
    while (true) {
        float current_motor_degrees = motor->get_position_degrees();
        float error_degrees = target_motor_degrees - current_motor_degrees;
        
        // Use position control logic to determine speed
        float control_output = position_control_internal(current_motor_degrees, target_motor_degrees);
        
        // Limit the maximum speed for calibration
        if (fabs(control_output) > max_speed) {
            control_output = (control_output < 0) ? -max_speed : max_speed;
        }
        
        motor->set_motor(control_output);
        
        if (fabs(error_degrees) < 1.0f) { // Close enough to target
            break;
        }
        
        // Safety check - ensure we don't hit limit switches unexpectedly after 1 second (40 iterations of 25ms)
        if (cont > 40){
            limit_switch_min->read();
            limit_switch_max->read();
            if (limit_switch_min->isPressed() || limit_switch_max->isPressed()) {
                printf("WARNING: Hit limit switch while moving to position!\n");
                break;
            }
            cont++;
        }
        
        
        printf("Moving to target... remaining: %.2f degrees\n", error_degrees);
        sleep_ms(((int)(motor->get_dt() * 1000)));  // Use motor's dt for sleep interval
    }
    
    // Stop at target position
    motor->set_motor(0.0f);
    motor->stop();
    sleep_ms(200);
}

// Public control methods
void Joint::set_joint(float desired_position) {
    // Update position and speed from encoder
    update_position();
    update_speed();
    
    // Check safety limits
    bool limits_violated = check_limits();
    
    if (!limits_violated && origin_set && setDesiredPosition(desired_position)) {
        // Only run position control if limits are OK and origin is set
        float u = position_control();
        if (u != 0.0f) {
            motor->set_motor(u);
        } else {
            motor->stop();
            motor->reset_pid_state();  // Clear PID state when stopped at target
        }
    } else if (limits_violated) {
        // Stop motor if limits are violated
        stop();
    }
}

void Joint::set_joint() {
    // Update position and speed from encoder
    update_position();
    update_speed();
    
    // Check safety limits
    bool limits_violated = check_limits();

    if (!limits_violated && origin_set) {
        // Only run position control if limits are OK and origin is set
        float u = position_control();
        if (u != 0.0f) {
            motor->set_motor(u);
        } else {
            motor->stop();
            motor->reset_pid_state();  // Clear PID state when stopped at target
        }
    } else if (limits_violated) {
        // Stop motor if limits are violated
        stop();
    }
}

void Joint::stop() {
    if (motor != nullptr) {
        motor->set_motor(0.0f);  // Set speed to 0
        motor->stop();
        printf("Joint stopped\n");
    }
}

void Joint::setSpeedScaleFactor(float scale) {
    // Clamp scale factor between 0.0 and 1.0
    if (scale < 0.0f) {
        speed_scale_factor = 0.0f;
    } else if (scale > 1.0f) {
        speed_scale_factor = 1.0f;
    } else {
        speed_scale_factor = scale;
    }
}

// Safety methods
bool Joint::isAtMinLimit() const {
    bool switch_pressed = false;
    if (limit_switch_min != nullptr) {
        switch_pressed = limit_switch_min->read();
    }
    return switch_pressed || (current_position <= min_limit);
}

bool Joint::isAtMaxLimit() const {
    bool switch_pressed = false;
    if (limit_switch_max != nullptr) {
        switch_pressed = limit_switch_max->read();
    }
    return switch_pressed || (current_position >= max_limit);
}

bool Joint::isWithinLimits() const {
    return !isAtMinLimit() && !isAtMaxLimit();
}
