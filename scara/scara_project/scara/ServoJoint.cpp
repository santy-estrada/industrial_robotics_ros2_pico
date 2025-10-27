#include "ServoJoint.h"
#include <pico/stdlib.h>
#include <cmath>

// Constructor
ServoJoint::ServoJoint(char joint_type, float min_lim, float max_lim, 
                       ServoMotor* servo_motor_ptr, float gear_ratio_value)
    : type(joint_type), min_limit(min_lim), max_limit(max_lim),
      current_position(0.0f), gear_ratio(gear_ratio_value),
      servo_motor(servo_motor_ptr), origin_set(false), origin_offset(0.0f) {
    
    printf("ServoJoint created: Type=%c, Limits=[%.2f, %.2f], GearRatio=%.2f\n", 
           type, min_limit, max_limit, gear_ratio);
    
    // Initialize current position
    update_position();
}

// Destructor
ServoJoint::~ServoJoint() {
    // Note: We don't delete servo_motor as it may be used elsewhere
    printf("ServoJoint destroyed\n");
}

// Helper method to update current position from servo
void ServoJoint::update_position() {
    if (servo_motor == nullptr) return;
    
    float servo_angle = servo_motor->getCurrentAngle();
    current_position = servo_to_joint_angle(servo_angle);
}

// Convert joint position to servo angle (accounting for gear ratio and origin offset)
float ServoJoint::joint_to_servo_angle(float joint_position) {
    // For prismatic joints: gear_ratio is degrees per mm
    // For revolute joints: gear_ratio is standard gear ratio
    // Formula: servo_angle = (joint_position * gear_ratio) + origin_offset
    float servo_angle = (joint_position * gear_ratio) + origin_offset;
    return servo_angle;
}

// Convert servo angle to joint position (accounting for gear ratio and origin offset)
float ServoJoint::servo_to_joint_angle(float servo_angle) {
    // Reverse of joint_to_servo_angle
    // Formula: joint_position = (servo_angle - origin_offset) / gear_ratio
    float joint_position = (servo_angle - origin_offset) / gear_ratio;
    return joint_position;
}

// Setters
void ServoJoint::setMinLimit(float min_lim) {
    min_limit = min_lim;
    printf("ServoJoint min limit set to %.2f\n", min_limit);
}

void ServoJoint::setMaxLimit(float max_lim) {
    max_limit = max_lim;
    printf("ServoJoint max limit set to %.2f\n", max_limit);
}

void ServoJoint::setOrigin() {
    if (servo_motor == nullptr) {
        printf("ERROR: Cannot set origin - servo motor is null\n");
        return;
    }
    
    // Set current servo position as the origin (0°) for joint coordinates
    origin_offset = servo_motor->getCurrentAngle();
    origin_set = true;
    
    printf("Origin set at current servo position (%.2f°)\n", origin_offset);
    update_position();  // Recalculate position with new origin
}

void ServoJoint::setOriginAt(float servo_origin_angle) {
    if (servo_motor == nullptr) {
        printf("ERROR: Cannot set origin - servo motor is null\n");
        return;
    }
    
    // The servo_origin_angle will be the servo position that corresponds to joint position 0°
    // So: joint_angle = (servo_angle - servo_origin_angle) / gear_ratio
    // Therefore: origin_offset = servo_origin_angle
    origin_offset = servo_origin_angle;
    origin_set = true;
    
    printf("Origin set: Servo angle %.2f° now corresponds to joint angle 0.0°\n", servo_origin_angle);
    printf("Origin offset set to: %.2f°\n", origin_offset);
    
    // Move servo to the origin position
    servo_motor->setAngle(servo_origin_angle);
    
    update_position();
}

bool ServoJoint::setPosition(float position) {
    if (!isWithinLimits(position)) {
        printf("WARNING: Position %.2f is outside limits [%.2f, %.2f]\n", 
               position, min_limit, max_limit);
        return false;
    }
    
    return moveToPosition(position);
}

// Getters
char ServoJoint::getType() const {
    return type;
}

float ServoJoint::getMinLimit() const {
    return min_limit;
}

float ServoJoint::getMaxLimit() const {
    return max_limit;
}

float ServoJoint::getCurrentPosition() const {
    return current_position;
}

float ServoJoint::getGearRatio() const {
    return gear_ratio;
}

bool ServoJoint::isOriginSet() const {
    return origin_set;
}

// Control methods
bool ServoJoint::moveToPosition(float target_position) {
    if (servo_motor == nullptr) {
        printf("ERROR: Cannot move - servo motor is null\n");
        return false;
    }
    
    if (!origin_set) {
        printf("ERROR: Cannot move to position - origin not set\n");
        return false;
    }
    
    if (!isWithinLimits(target_position)) {
        printf("WARNING: Target position %.2f is outside limits [%.2f, %.2f]\n", 
               target_position, min_limit, max_limit);
        return false;
    }
    
    // Convert joint position to servo angle
    float target_servo_angle = joint_to_servo_angle(target_position);
    
    // Check if servo angle is within servo motor limits
    if (target_servo_angle < servo_motor->getMinAngle() || 
        target_servo_angle > servo_motor->getMaxAngle()) {
        printf("ERROR: Target joint position %.2f° requires servo angle %.2f° which is outside servo limits [%.2f°, %.2f°]\n",
               target_position, target_servo_angle, servo_motor->getMinAngle(), servo_motor->getMaxAngle());
        return false;
    }
    
    // Move servo to target angle
    bool success = servo_motor->setAngle(target_servo_angle);
    
    // Update current position
    update_position();
    
    printf("ServoJoint moved to %.2f° (servo: %.2f°)\n", current_position, target_servo_angle);
    
    return success;
}

void ServoJoint::stop() {
    if (servo_motor != nullptr) {
        servo_motor->stop();
        printf("ServoJoint stopped\n");
    }
}

// Safety methods
bool ServoJoint::isWithinLimits() const {
    return isWithinLimits(current_position);
}

bool ServoJoint::isWithinLimits(float position) const {
    return (position >= min_limit && position <= max_limit);
}