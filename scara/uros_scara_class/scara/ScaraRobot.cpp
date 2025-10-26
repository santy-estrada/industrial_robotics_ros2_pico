#include "ScaraRobot.h"
#include <pico/stdlib.h>
#include <cmath>

// Constructor
ScaraRobot::ScaraRobot(
    // Joint 1 pins
    int j1_motor_ena, int j1_motor_in1, int j1_motor_in2,
    int j1_encoder_a, int j1_encoder_b, int j1_limit_min_pin, int j1_limit_max_pin,
    // Joint 2 pins  
    int j2_motor_ena, int j2_motor_in1, int j2_motor_in2,
    int j2_encoder_a, int j2_encoder_b, int j2_limit_min_pin, int j2_limit_max_pin,
    // Joint 3 pins
    int j3_servo_pwm,
    // Safety pendant pin (optional, -1 to disable)
    int pendant_pin
) : is_initialized(false), revolute_joints_calibrated(false), 
    pendant_pin(pendant_pin), pendant_enabled(false) {
    
    printf("=== Initializing SCARA Robot ===\n");
    
    // Create precision motor for joint 1
    printf("Creating Joint 1 precision motor...\n");
    joint1_motor = new PrecisionMotor(j1_motor_ena, j1_motor_in1, j1_motor_in2,
                                     j1_encoder_a, j1_encoder_b, 
                                     64, 50.0f, 0.05f, 0.2332f, 0.35f, 0.0000125f);
    
    // Create limit switches for joint 1
    printf("Creating Joint 1 limit switches...\n");
    joint1_limit_min = new LimitSwitch(j1_limit_min_pin);
    joint1_limit_max = new LimitSwitch(j1_limit_max_pin);
    
    // Create SCARA joint 1
    printf("Creating SCARA Joint 1...\n");
    joint1 = new Joint('R', -80.0f, 80.0f, joint1_motor, 4.0f, 
                       joint1_limit_min, joint1_limit_max, 1.5f);
    
    // Create precision motor for joint 2
    printf("Creating Joint 2 precision motor...\n");
    joint2_motor = new PrecisionMotor(j2_motor_ena, j2_motor_in1, j2_motor_in2,
                                     j2_encoder_a, j2_encoder_b, 
                                     64, 50.0f, 0.05f, 0.2332f, 0.35f, 0.0000125f);
    
    // Create limit switches for joint 2
    printf("Creating Joint 2 limit switches...\n");
    joint2_limit_min = new LimitSwitch(j2_limit_min_pin);
    joint2_limit_max = new LimitSwitch(j2_limit_max_pin);
    
    // Create SCARA joint 2
    printf("Creating SCARA Joint 2...\n");
    joint2 = new Joint('R', -120.0f, 120.0f, joint2_motor, 4.0f, 
                       joint2_limit_min, joint2_limit_max, 1.8f);
    
    // Create servo motor and servo joint for joint 3
    printf("Creating Joint 3 servo motor and servo joint...\n");
    joint3_servo_motor = new ServoMotor(j3_servo_pwm);
    // Prismatic joint: -13mm to +10mm range (23mm total)
    // Measured: 10° servo movement = 4mm joint movement (opposite direction)
    // Gear ratio = -2.5 degrees per mm (negative for opposite direction)
    // When 0mm tool position, servo is at 70° angle
    joint3 = new ServoJoint('P', -13.0f, 10.0f, joint3_servo_motor, -2.5f);

    // Initialize safety pendant if pin is provided
    if (pendant_pin >= 0) {
        printf("Initializing safety pendant on pin %d...\n", pendant_pin);
        gpio_init(pendant_pin);
        gpio_set_dir(pendant_pin, GPIO_IN);
        gpio_pull_up(pendant_pin);  // Pull-up like in home_routine_fn.c
        pendant_enabled = true;
        printf("Safety pendant enabled - robot will only move when button is pressed\n");
    } else {
        printf("Safety pendant disabled\n");
        pendant_enabled = false;
    }
    
    is_initialized = true;
    printf("SCARA Robot initialized successfully!\n");
}

// Destructor
ScaraRobot::~ScaraRobot() {
    printf("Destroying SCARA Robot...\n");
    
    // Delete joints (which will call their destructors)
    delete joint1;
    delete joint2;
    delete joint3;
    
    // Delete motors
    delete joint1_motor;
    delete joint2_motor;
    delete joint3_servo_motor;
    
    // Delete limit switches
    delete joint1_limit_min;
    delete joint1_limit_max;
    delete joint2_limit_min;
    delete joint2_limit_max;
    
    printf("SCARA Robot destroyed\n");
}

// Calibration methods
void ScaraRobot::calibrateRevoluteJoints() {
    if (!is_initialized) {
        printf("ERROR: Cannot calibrate - robot not initialized\n");
        return;
    }
    
    printf("=== Calibrating Revolute Joints ===\n");
    
    // Calibrate joint 2 first, then joint 1
    printf("Calibrating Joint 2...\n");
    joint2->calibrateOrigin();
    
    printf("Calibrating Joint 1...\n");
    joint1->calibrateOrigin();
    
    revolute_joints_calibrated = true;
    printf("Revolute joints calibration complete!\n");
}

void ScaraRobot::calibrateServoJoint(float servo_origin_angle) {
    if (!is_initialized) {
        printf("ERROR: Cannot calibrate servo joint - robot not initialized\n");
        return;
    }
    
    printf("Calibrating Joint 3 (servo) at origin angle %.2f°...\n", servo_origin_angle);
    joint3->setOriginAt(servo_origin_angle);
}

// Joint control methods
void ScaraRobot::setJoint1(float angle) {
    if (!is_initialized) {
        printf("ERROR: Cannot control joint 1 - robot not initialized\n");
        return;
    }
    
    // Check safety pendant before allowing movement
    if (!isSafeToMove()) {
        joint1->stop();
        return;
    }
    
    if (std::isnan(angle)) {
        // No target angle provided, just update the joint
        joint1->set_joint();
    } else {
        // Target angle provided
        joint1->set_joint(angle);
    }
}

void ScaraRobot::setJoint2(float angle) {
    if (!is_initialized) {
        printf("ERROR: Cannot control joint 2 - robot not initialized\n");
        return;
    }
    
    // Check safety pendant before allowing movement
    if (!isSafeToMove()) {
        joint2->stop();
        return;
    }
    
    if (std::isnan(angle)) {
        // No target angle provided, just update the joint
        joint2->set_joint();
    } else {
        // Target angle provided
        joint2->set_joint(angle);
    }
}

bool ScaraRobot::moveJoint3(float position) {
    if (!is_initialized) {
        printf("ERROR: Cannot move joint 3 - robot not initialized\n");
        return false;
    }
    
    // Check safety pendant before allowing movement
    if (!isSafeToMove()) {
        joint3->stop();
        return false;
    }
    
    return joint3->moveToPosition(position);
}

void ScaraRobot::updateJoints() {
    if (!is_initialized) return;
    
    // Check safety pendant - only update if safe to move
    if (!isSafeToMove()) {
        // Stop all motors but still update position readings (encoders continue to work)
        joint1->stop();
        joint2->stop();
        // Don't call set_joint() to avoid updating historic error while stopped
        return;
    }
    
    // Update revolute joints (calls their control loops)
    joint1->set_joint();
    joint2->set_joint();
    
    // Joint 3 (servo) doesn't need continuous updates like the others
}

// Robot control methods
void ScaraRobot::moveToConfiguration(float j1_angle, float j2_angle, float j3_position) {
    if (!is_initialized) {
        printf("ERROR: Cannot move to configuration - robot not initialized\n");
        return;
    }
    
    // Check safety pendant before allowing movement
    if (!isSafeToMove()) {
        printf("SAFETY: Cannot move to configuration - pendant not pressed\n");
        stopAllJoints();
        return;
    }
    
    // Track last Joint 3 command to avoid redundant servo updates (prevents overshoot)
    static float last_j3_position = 0.0f;
    static bool j3_initialized = false;
    
    // Calculate position errors for Joint 1 and Joint 2
    float j1_error = fabs(j1_angle - joint1->getCurrentPosition());
    float j2_error = fabs(j2_angle - joint2->getCurrentPosition());
    
    // Find the maximum error (this joint will move at full speed)
    float max_error = (j1_error > j2_error) ? j1_error : j2_error;
    
    // Calculate speed scale factors for synchronized motion
    // The joint with the largest error moves at full speed (scale = 1.0)
    // The other joint moves proportionally slower so they finish together
    float j1_scale = 1.0f;
    float j2_scale = 1.0f;
    
    if (max_error > 0.5f) {  // Only apply scaling if movement is significant
        j1_scale = (j1_error / max_error);
        j2_scale = (j2_error / max_error);
        
        // Ensure minimum speed to avoid stalling
        const float min_scale = 0.3f;
        if (j1_scale < min_scale && j1_error > 0.15f) j1_scale = min_scale;
        if (j2_scale < min_scale && j2_error > 0.15f) j2_scale = min_scale;
    }
    
    // Apply speed scaling to joints
    joint1->setSpeedScaleFactor(j1_scale);
    joint2->setSpeedScaleFactor(j2_scale);
    
    // Set target positions for revolute joints (updated every cycle for control loop)
    joint1->set_joint(j1_angle);
    joint2->set_joint(j2_angle);
    
    // Only update Joint 3 (servo) when command actually changes (avoid overshoot from repetitive commands)
    if (!j3_initialized || fabs(j3_position - last_j3_position) > 0.01f) {
        joint3->moveToPosition(j3_position);
        last_j3_position = j3_position;
        j3_initialized = true;
    }
}

void ScaraRobot::stopAllJoints() {
    if (!is_initialized) return;
    
    printf("Stopping all SCARA joints...\n");
    joint1->stop();
    joint2->stop();
    joint3->stop();
}

void ScaraRobot::emergencyStop() {
    printf("EMERGENCY STOP: Stopping all SCARA joints immediately!\n");
    if (is_initialized) {
        joint1->stop();
        joint2->stop(); 
        joint3->stop();
    }
}

// Status getters - Joint 1
float ScaraRobot::getJoint1CurrentPosition() const {
    return is_initialized ? joint1->getCurrentPosition() : 0.0f;
}

float ScaraRobot::getJoint1CurrentSpeed() const {
    return is_initialized ? joint1->getCurrentSpeed() : 0.0f;
}

float ScaraRobot::getJoint1DesiredPosition() const {
    return is_initialized ? joint1->getDesiredPosition() : 0.0f;
}

float ScaraRobot::getJoint1ErrorPosition() const {
    return is_initialized ? joint1->getErrorPosition() : 0.0f;
}

float ScaraRobot::getJoint1MinLimit() const {
    return is_initialized ? joint1->getMinLimit() : 0.0f;
}

float ScaraRobot::getJoint1MaxLimit() const {
    return is_initialized ? joint1->getMaxLimit() : 0.0f;
}

bool ScaraRobot::isJoint1WithinLimits() const {
    return is_initialized ? joint1->isWithinLimits() : false;
}

// Status getters - Joint 2
float ScaraRobot::getJoint2CurrentPosition() const {
    return is_initialized ? joint2->getCurrentPosition() : 0.0f;
}

float ScaraRobot::getJoint2CurrentSpeed() const {
    return is_initialized ? joint2->getCurrentSpeed() : 0.0f;
}

float ScaraRobot::getJoint2DesiredPosition() const {
    return is_initialized ? joint2->getDesiredPosition() : 0.0f;
}

float ScaraRobot::getJoint2ErrorPosition() const {
    return is_initialized ? joint2->getErrorPosition() : 0.0f;
}

float ScaraRobot::getJoint2MinLimit() const {
    return is_initialized ? joint2->getMinLimit() : 0.0f;
}

float ScaraRobot::getJoint2MaxLimit() const {
    return is_initialized ? joint2->getMaxLimit() : 0.0f;
}

bool ScaraRobot::isJoint2WithinLimits() const {
    return is_initialized ? joint2->isWithinLimits() : false;
}

// Status getters - Joint 3
float ScaraRobot::getJoint3CurrentPosition() const {
    return is_initialized ? joint3->getCurrentPosition() : 0.0f;
}

float ScaraRobot::getJoint3MinLimit() const {
    return is_initialized ? joint3->getMinLimit() : 0.0f;
}

float ScaraRobot::getJoint3MaxLimit() const {
    return is_initialized ? joint3->getMaxLimit() : 0.0f;
}

char ScaraRobot::getJoint3Type() const {
    return is_initialized ? joint3->getType() : 'U'; // 'U' for undefined
}

bool ScaraRobot::isJoint3WithinLimits() const {
    return is_initialized ? joint3->isWithinLimits() : false;
}

// Robot status getters
bool ScaraRobot::isInitialized() const {
    return is_initialized;
}

bool ScaraRobot::areRevoluteJointsCalibrated() const {
    return revolute_joints_calibrated;
}

bool ScaraRobot::areAllJointsWithinLimits() const {
    if (!is_initialized) return false;
    
    return joint1->isWithinLimits() && 
           joint2->isWithinLimits() && 
           joint3->isWithinLimits();
}

// Configuration getters
void ScaraRobot::getCurrentConfiguration(float& j1_pos, float& j2_pos, float& j3_pos) const {
    if (is_initialized) {
        j1_pos = joint1->getCurrentPosition();
        j2_pos = joint2->getCurrentPosition();
        j3_pos = joint3->getCurrentPosition();
    } else {
        j1_pos = j2_pos = j3_pos = 0.0f;
    }
}

void ScaraRobot::printStatus() const {
    if (!is_initialized) {
        printf("SCARA Robot: NOT INITIALIZED\n");
        return;
    }
    
    printf("\n=== SCARA Robot Status ===\n");
    printf("Initialized: %s\n", is_initialized ? "YES" : "NO");
    printf("Revolute Joints Calibrated: %s\n", revolute_joints_calibrated ? "YES" : "NO");
    
    printf("\nJoint 1 (Revolute): Pos=%.2f°, Speed=%.2f°/s, Target=%.2f°, Error=%.2f°\n",
           joint1->getCurrentPosition(), joint1->getCurrentSpeed(), 
           joint1->getDesiredPosition(), joint1->getErrorPosition());
    
    printf("Joint 2 (Revolute): Pos=%.2f°, Speed=%.2f°/s, Target=%.2f°, Error=%.2f°\n",
           joint2->getCurrentPosition(), joint2->getCurrentSpeed(), 
           joint2->getDesiredPosition(), joint2->getErrorPosition());
    
    printf("Joint 3 (Servo): Pos=%.2f°, Range=[%.1f°, %.1f°]\n",
           joint3->getCurrentPosition(), joint3->getMinLimit(), joint3->getMaxLimit());
    
    // Safety status
    if (!areAllJointsWithinLimits()) {
        printf("WARNING: One or more joints are outside their limits!\n");
        if (!joint1->isWithinLimits()) printf("  - Joint 1 outside limits\n");
        if (!joint2->isWithinLimits()) printf("  - Joint 2 outside limits\n");
        if (!joint3->isWithinLimits()) printf("  - Joint 3 outside limits\n");
    }
    
    // Safety pendant status
    if (pendant_enabled) {
        printf("Safety Pendant: %s\n", isPendantPressed() ? "PRESSED (safe to move)" : "NOT PRESSED (motors stopped)");
    } else {
        printf("Safety Pendant: DISABLED\n");
    }
}

// Safety pendant methods
bool ScaraRobot::isSafeToMove() {
    if (!is_initialized) return false;
    
    // If pendant is disabled, always safe to move
    if (!pendant_enabled) return true;
    
    // If pendant is enabled, only safe when pressed
    // Button is pulled up, so pressed = LOW (false)
    return !gpio_get(pendant_pin);
}

void ScaraRobot::enableSafetyPendant(int pin) {
    if (pin < 0) {
        printf("ERROR: Invalid pendant pin %d\n", pin);
        return;
    }
    
    pendant_pin = pin;
    gpio_init(pendant_pin);
    gpio_set_dir(pendant_pin, GPIO_IN);
    gpio_pull_up(pendant_pin);  // Pull-up like in home_routine_fn.c
    pendant_enabled = true;
    
    printf("Safety pendant enabled on pin %d\n", pendant_pin);
}

void ScaraRobot::disableSafetyPendant() {
    pendant_enabled = false;
    printf("Safety pendant disabled\n");
}

bool ScaraRobot::isPendantPressed() const {
    if (!pendant_enabled) return false;
    
    // Button is pulled up, so pressed = LOW (false)
    return !gpio_get(pendant_pin);
}

bool ScaraRobot::isPendantEnabled() const {
    return pendant_enabled;
}