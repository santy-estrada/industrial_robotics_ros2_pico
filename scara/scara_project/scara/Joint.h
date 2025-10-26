#ifndef JOINT_H
#define JOINT_H

#include "../motor/PrecisionMotor.h"
#include "../sensor/LimitSwitch.h"
#include <stdio.h>

class Joint {
private:
    // Joint type ('R' for revolute, 'P' for prismatic)
    char type;
    float resistance;         // Joint resistance 
    
    // Physical limits (degrees for revolute, mm for prismatic)
    float min_limit;
    float max_limit;
    
    // Position and motion variables
    float current_position;      // Current position (calculated from encoder in deg)
    float desired_position;      // Target position for control loop
    float current_speed;         // Current speed (calculated from encoder)
    
    // Motor and mechanical configuration
    PrecisionMotor* motor;       // Pointer to precision motor
    float gear_ratio;            // Motor shaft revolutions per joint revolution
    
    // Limit switches
    LimitSwitch* limit_switch_min;  // Limit switch at minimum position
    LimitSwitch* limit_switch_max;  // Limit switch at maximum position
    
    // Origin calibration
    bool origin_set;             // Flag indicating if origin has been calibrated
    
    // PI controller variables for position control
    bool use_pi_controller;      // Flag to select PI or threshold controller
    float kp_pos;               // Proportional gain for position control
    float ti_pos;               // Integral time constant for position control
    float q0_pos, q1_pos;       // PI coefficients for position control
    float error_pos[2];         // Position error history: e(k), e(k-1)
    
    // Synchronized motion support
    float speed_scale_factor;    // Scaling factor for synchronized motion (1.0 = normal speed)
    
    // Private methods
    float position_control();     // Position control implementation
    float position_control_internal(float current_pos, float desired_pos); // Internal position control without origin check
    void update_position();      // Update current position from encoder
    void update_speed();         // Update current speed from encoder
    bool check_limits();         // Check limit switches and position limits
    bool setDesiredPosition(float desired_pos);
    void move_to_position_internal(float target_motor_degrees, float speed); // Internal movement without origin check
    void calculate_pi_parameters(); // Calculate PI controller coefficients


public:
    // Constructor (threshold controller)
    Joint(char joint_type, float min_lim, float max_lim, 
          PrecisionMotor* precision_motor, float gear_ratio_value,
          LimitSwitch* limit_min, LimitSwitch* limit_max, float resistance_value);
    
    // Constructor (PI controller)
    Joint(char joint_type, float min_lim, float max_lim, 
          PrecisionMotor* precision_motor, float gear_ratio_value,
          LimitSwitch* limit_min, LimitSwitch* limit_max, float resistance_value,
          float kp, float ti);
    
    // Destructor
    ~Joint();
    
    // Setters
    void setMinLimit(float min_lim);
    void setMaxLimit(float max_lim);
    void setOrigin();                           // Set current position as origin (0°)
    bool calibrateOrigin();                     // Auto-calibrate origin by moving between limits
    
    // Getters
    char getType() const;
    float getMinLimit() const;
    float getMaxLimit() const;
    float getCurrentPosition();           // Only getter - no setter
    float getDesiredPosition() const;
    float getCurrentSpeed();              // Only getter - no setter
    float getGearRatio() const;
    float getErrorPosition() const;
    bool isOriginSet() const;
    
    // Control methods
    void set_joint();                           // Update position, speed, and run control
    void set_joint(float desired_position);     // Update position, speed, and run control
    void setSpeedScaleFactor(float scale);      // Set speed scaling for synchronized motion (0.0-1.0)
    void stop();                                // Stop the joint motor
    
    // Safety methods
    bool isAtMinLimit() const;
    bool isAtMaxLimit() const;
    bool isWithinLimits() const;
    
};

#endif // JOINT_H