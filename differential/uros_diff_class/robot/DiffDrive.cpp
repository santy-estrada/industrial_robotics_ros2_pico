#include "DiffDrive.h"
#include <stdio.h>

DiffDrive::DiffDrive(uint motor_l_ena, uint motor_l_in1, uint motor_l_in2,
                     uint encoder_l_a, uint encoder_l_b,
                     uint motor_r_ena, uint motor_r_in1, uint motor_r_in2,
                     uint encoder_r_a, uint encoder_r_b,
                     uint ultrasonic_trig, uint ultrasonic_echo,
                     uint imu_sda, uint imu_scl, i2c_inst_t* i2c_port,
                     float encoder_ticks_per_rev, float gear_ratio,
                     float control_dt, float pid_kp, float pid_ti, float pid_td)
    : motor_l_ena(motor_l_ena), motor_l_in1(motor_l_in1), motor_l_in2(motor_l_in2),
      encoder_l_a(encoder_l_a), encoder_l_b(encoder_l_b),
      motor_r_ena(motor_r_ena), motor_r_in1(motor_r_in1), motor_r_in2(motor_r_in2),
      encoder_r_a(encoder_r_a), encoder_r_b(encoder_r_b),
      ultrasonic_trig(ultrasonic_trig), ultrasonic_echo(ultrasonic_echo),
      imu_sda(imu_sda), imu_scl(imu_scl), i2c_port(i2c_port),
      encoder_ticks_per_rev(encoder_ticks_per_rev), gear_ratio(gear_ratio),
      dt(control_dt), pid_kp(pid_kp), pid_ti(pid_ti), pid_td(pid_td),
      obstacle_threshold_cm(20.0f), emergency_stop_threshold_cm(15.0f), cont_stop(0),
      is_warning(false), is_emergency_stopped(false),
      motors_initialized(false), sensors_initialized(false),
      current_distance_cm(-1.0f), distance_valid(false),
      motor_left(nullptr), motor_right(nullptr),
      ultrasonic(nullptr), imu(nullptr), filter(nullptr) {
}

DiffDrive::~DiffDrive() {
    if (motor_left) {
        delete motor_left;
    }
    if (motor_right) {
        delete motor_right;
    }
    if (ultrasonic) {
        delete ultrasonic;
    }
    if (imu) {
        delete imu;
    }
    if (filter) {
        delete filter;
    }
}

bool DiffDrive::init() {
    printf("Initializing DiffDrive robot...\n");
    
    // Initialize motors
    printf("Creating precision motors...\n");
    motor_left = new PrecisionMotor(motor_l_ena, motor_l_in1, motor_l_in2,
                                   encoder_l_a, encoder_l_b,
                                   encoder_ticks_per_rev, gear_ratio, dt,
                                   pid_kp, pid_ti, pid_td);
    
    motor_right = new PrecisionMotor(motor_r_ena, motor_r_in1, motor_r_in2,
                                    encoder_r_a, encoder_r_b,
                                    encoder_ticks_per_rev, gear_ratio, dt,
                                    pid_kp, pid_ti, pid_td);
    
    if (!motor_left || !motor_right) {
        printf("ERROR: Failed to create motors!\n");
        return false;
    }
    
    motors_initialized = true;
    printf("Motors initialized successfully!\n");
    
    // Initialize ultrasonic sensor
    printf("Creating ultrasonic sensor...\n");
    ultrasonic = new UltrasonicSensor(ultrasonic_trig, ultrasonic_echo);
    
    if (!ultrasonic) {
        printf("ERROR: Failed to create ultrasonic sensor!\n");
        return false;
    }
    
    printf("Ultrasonic sensor initialized successfully!\n");
    
    // Initialize BMI160 IMU
    printf("Creating BMI160 IMU...\n");
    imu = new BMI160();
    filter = new MadgwickFilter();
    
    if (!imu || !filter) {
        printf("ERROR: Failed to create IMU or filter!\n");
        return false;
    }
    
    sleep_ms(1000);
    
    // Initialize BMI160 using I2C
    if (!imu->begin(BMI160::I2C_MODE, i2c_port, imu_sda, imu_scl)) {
        printf("ERROR: Failed to initialize BMI160!\n");
        return false;
    }
    
    sensors_initialized = true;
    printf("BMI160 initialized successfully! Chip ID: 0x%02X\n", imu->getChipID());
    
    printf("DiffDrive robot initialization complete!\n");
    return true;
}

void DiffDrive::update() {
    if (!motors_initialized || !sensors_initialized) {
        return;
    }
    
    // Update ultrasonic sensor reading
    float distance_m;
    if (ultrasonic->measure_distance(distance_m)) {
        current_distance_cm = distance_m * 100.0f; // Convert to cm
        distance_valid = true;
        
        // Check for warning condition
        is_warning = (current_distance_cm <= obstacle_threshold_cm);
        
        // Check for emergency stop condition
        if (current_distance_cm <= emergency_stop_threshold_cm) {
            if (!is_emergency_stopped) {
                printf("EMERGENCY STOP: Obstacle at %.1f cm (threshold: %.1f cm)\n", 
                       current_distance_cm, emergency_stop_threshold_cm);
                       
                motor_left->set_motor(0.0f);
                motor_right->set_motor(0.0f);
                is_emergency_stopped = true;
            }
        } else {
            // Clear emergency stop if distance is safe
            is_emergency_stopped = false;
        }
    } else {
        distance_valid = false;
        current_distance_cm = -1.0f;
    }
    
    // Update IMU readings and filter
    if (imu->read_accel_gyro()) {
        filter->update(imu->gx, imu->gy, imu->gz, imu->ax, imu->ay, imu->az, dt);
    }
    
    // Update motor control (only if not emergency stopped)
    if (!is_emergency_stopped) {
        motor_left->set_motor(motor_left->get_setpoint());
        motor_right->set_motor(motor_right->get_setpoint());
        cont_stop = 0;
    }else if (cont_stop < 10) {
        motor_left->set_motor(0.0f);
        motor_right->set_motor(0.0f);
        cont_stop++;
    }else {
        motor_left->stop();
        motor_right->stop();
    }
}

// === MOTOR CONTROL ===
void DiffDrive::set_left_speed(float speed_rpm) {
    if (motors_initialized && !is_emergency_stopped) {
        motor_left->set_motor(speed_rpm);
    }
}

void DiffDrive::set_right_speed(float speed_rpm) {
    if (motors_initialized && !is_emergency_stopped) {
        motor_right->set_motor(speed_rpm);
    }
}

void DiffDrive::set_speeds(float left_rpm, float right_rpm) {
    if (motors_initialized && !is_emergency_stopped) {
        if (left_rpm == 0.0f) {
            motor_left->stop();
        }
        motor_left->set_motor(left_rpm);

        if (right_rpm == 0.0f) {
            motor_right->stop();
        }
        motor_right->set_motor(right_rpm);
    } else {
        motor_left->set_motor(0.0f);
        motor_right->set_motor(0.0f);
    }
}

void DiffDrive::stop() {
    if (motors_initialized) {
        motor_left->stop();
        motor_right->stop();
        printf("DiffDrive: All motors stopped\n");
    }
}

// === GETTERS FOR MOTOR STATUS ===
float DiffDrive::get_left_desired_speed() const {
    return motors_initialized ? motor_left->get_setpoint() : 0.0f;
}

float DiffDrive::get_right_desired_speed() const {
    return motors_initialized ? motor_right->get_setpoint() : 0.0f;
}

float DiffDrive::get_left_current_speed() const {
    return motors_initialized ? motor_left->get_filtered_rpm() : 0.0f;
}

float DiffDrive::get_right_current_speed() const {
    return motors_initialized ? motor_right->get_filtered_rpm() : 0.0f;
}

float DiffDrive::get_left_error() const {
    return motors_initialized ? motor_left->get_error() : 0.0f;
}

float DiffDrive::get_right_error() const {
    return motors_initialized ? motor_right->get_error() : 0.0f;
}

float DiffDrive::get_left_power() const {
    return motors_initialized ? motor_left->get_control_output() : 0.0f;
}

float DiffDrive::get_right_power() const {
    return motors_initialized ? motor_right->get_control_output() : 0.0f;
}

// === SENSOR GETTERS ===
float DiffDrive::get_distance_cm() const {
    return distance_valid ? current_distance_cm : -1.0f;
}

bool DiffDrive::get_acceleration(float& ax, float& ay, float& az) const {
    if (sensors_initialized && imu) {
        ax = imu->ax;
        ay = imu->ay;
        az = imu->az;
        return true;
    }
    return false;
}

bool DiffDrive::get_gyroscope(float& gx, float& gy, float& gz) const {
    if (sensors_initialized && imu) {
        gx = imu->gx;
        gy = imu->gy;
        gz = imu->gz;
        return true;
    }
    return false;
}

bool DiffDrive::get_quaternion(float& qx, float& qy, float& qz, float& qw) const {
    if (sensors_initialized && filter) {
        filter->getQuaternion(qx, qy, qz, qw);
        return true;
    }
    return false;
}

// === SAFETY STATUS ===
bool DiffDrive::is_warning_active() const {
    return is_warning;
}

bool DiffDrive::is_emergency_stop_active() const {
    return is_emergency_stopped;
}

// === CONFIGURATION ===
void DiffDrive::set_obstacle_threshold(float threshold_cm) {
    obstacle_threshold_cm = threshold_cm;
    printf("Obstacle warning threshold set to %.1f cm\n", threshold_cm);
}

float DiffDrive::get_obstacle_threshold() const {
    return obstacle_threshold_cm;
}

bool DiffDrive::is_initialized() const {
    return motors_initialized && sensors_initialized;
}