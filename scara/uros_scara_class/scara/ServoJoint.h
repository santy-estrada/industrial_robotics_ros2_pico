#ifndef SERVO_JOINT_H
#define SERVO_JOINT_H

#include "../motor/ServoMotor.h"
#include <stdio.h>

class ServoJoint {
private:
    // Joint type ('R' for revolute, 'P' for prismatic)
    char type;
    
    // Physical limits (degrees for revolute, mm for prismatic)
    float min_limit;
    float max_limit;
    
    // Position and gear ratio
    float current_position;      // Current position in joint coordinates
    float gear_ratio;            // Motor shaft revolutions per joint revolution
    
    // Servo motor
    ServoMotor* servo_motor;     // Pointer to servo motor
    
    // Origin calibration
    bool origin_set;             // Flag indicating if origin has been calibrated
    float origin_offset;         // Offset angle for origin calibration
    
    // Helper methods
    void update_position();      // Update current position from servo
    float joint_to_servo_angle(float joint_angle);  // Convert joint angle to servo angle
    float servo_to_joint_angle(float servo_angle);  // Convert servo angle to joint angle

public:
    // Constructor
    ServoJoint(char joint_type, float min_lim, float max_lim, 
               ServoMotor* servo_motor_ptr, float gear_ratio_value);
    
    // Destructor
    ~ServoJoint();
    
    // Setters
    void setMinLimit(float min_lim);
    void setMaxLimit(float max_lim);
    void setOrigin();                           // Set current position as origin (0°)
    void setOriginAt(float servo_origin_angle); // Set servo angle that corresponds to joint 0°
    bool setPosition(float position);           // Move to specific position
    
    // Getters
    char getType() const;
    float getMinLimit() const;
    float getMaxLimit() const;
    float getCurrentPosition() const;           // Get current joint position
    float getGearRatio() const;
    bool isOriginSet() const;
    
    // Control methods
    bool moveToPosition(float target_position);  // Move to target position
    void stop();                                 // Stop the servo
    
    // Safety methods
    bool isWithinLimits() const;
    bool isWithinLimits(float position) const;   // Check if specific position is within limits
};

#endif // SERVO_JOINT_H