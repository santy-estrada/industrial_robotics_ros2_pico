#ifndef SCARA_ROBOT_H
#define SCARA_ROBOT_H

#include "Joint.h"
#include "ServoJoint.h"
#include "../motor/PrecisionMotor.h"
#include "../motor/ServoMotor.h"
#include "../sensor/LimitSwitch.h"
#include <stdio.h>

class ScaraRobot {
private:
    // Joint components
    Joint* joint1;          // Revolute joint 1 (±90°, 4:1 gear ratio)
    Joint* joint2;          // Revolute joint 2 (±120°, 4:1 gear ratio)  
    ServoJoint* joint3;     // Prismatic joint 3 (servo-based)
    
    // Motor components (owned by this class)
    PrecisionMotor* joint1_motor;
    PrecisionMotor* joint2_motor;
    ServoMotor* joint3_servo_motor;
    
    // Limit switch components (owned by this class)
    LimitSwitch* joint1_limit_min;
    LimitSwitch* joint1_limit_max;
    LimitSwitch* joint2_limit_min;
    LimitSwitch* joint2_limit_max;
    
    // Robot state
    bool is_initialized;
    bool revolute_joints_calibrated;
    
    // Safety pendant (dead man's switch)
    int pendant_pin;
    bool pendant_enabled;

public:
    // Constructor - takes all the pin definitions
    ScaraRobot(
        // Joint 1 pins
        int j1_motor_ena, int j1_motor_in1, int j1_motor_in2,
        int j1_encoder_a, int j1_encoder_b, int j1_limit_min_pin, int j1_limit_max_pin,
        // Joint 2 pins  
        int j2_motor_ena, int j2_motor_in1, int j2_motor_in2,
        int j2_encoder_a, int j2_encoder_b, int j2_limit_min_pin, int j2_limit_max_pin,
        // Joint 3 pins
        int j3_servo_pwm,
        // Safety pendant pin (optional, -1 to disable)
        int pendant_pin = -1
    );
    
    // Destructor
    ~ScaraRobot();
    
    // Calibration methods
    void calibrateRevoluteJoints();        // Calibrate joint2 then joint1
    void calibrateServoJoint(float servo_origin_angle); // Set servo origin angle
    
    // Joint control methods
    void setJoint1(float angle = NAN);      // Control joint 1 (optional target angle)
    void setJoint2(float angle = NAN);      // Control joint 2 (optional target angle)
    bool moveJoint3(float position);        // Move joint 3 to position
    void updateJoints();                    // Update all joint states (call in control loop)
    bool isSafeToMove();                    // Check if safety pendant allows movement
    
    // Safety pendant methods
    void enableSafetyPendant(int pin);      // Enable safety pendant on specified pin
    void disableSafetyPendant();            // Disable safety pendant
    bool isPendantPressed() const;          // Check if pendant button is pressed
    bool isPendantEnabled() const;          // Check if pendant is enabled
    
    // Robot control methods
    void moveToConfiguration(float j1_angle, float j2_angle, float j3_position);
    void stopAllJoints();
    void emergencyStop();
    
    // Status getters - Joint 1
    float getJoint1CurrentPosition() const;
    float getJoint1CurrentSpeed() const;
    float getJoint1DesiredPosition() const;
    float getJoint1ErrorPosition() const;
    float getJoint1MinLimit() const;
    float getJoint1MaxLimit() const;
    bool isJoint1WithinLimits() const;
    
    // Status getters - Joint 2  
    float getJoint2CurrentPosition() const;
    float getJoint2CurrentSpeed() const;
    float getJoint2DesiredPosition() const;
    float getJoint2ErrorPosition() const;
    float getJoint2MinLimit() const;
    float getJoint2MaxLimit() const;
    bool isJoint2WithinLimits() const;
    
    // Status getters - Joint 3
    float getJoint3CurrentPosition() const;
    float getJoint3MinLimit() const;
    float getJoint3MaxLimit() const;
    char getJoint3Type() const;
    bool isJoint3WithinLimits() const;
    
    // Robot status getters
    bool isInitialized() const;
    bool areRevoluteJointsCalibrated() const;
    bool areAllJointsWithinLimits() const;
    
    // Configuration getters
    void getCurrentConfiguration(float& j1_pos, float& j2_pos, float& j3_pos) const;
    void printStatus() const;
};

#endif // SCARA_ROBOT_H