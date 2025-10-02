#ifndef DIFF_DRIVE_H
#define DIFF_DRIVE_H

#include "../motor/PrecisionMotor.h"
#include "../sensor/ultrasonic_sensor.hpp"
#include "../sensor/bmi160.hpp"
#include "../sensor/madgwick_filter.hpp"
#include <hardware/i2c.h>

/**
 * @brief Differential Drive Robot Class
 * 
 * This class manages a differential drive robot with:
 * - Two precision motors with PID control
 * - BMI160 IMU sensor
 * - Ultrasonic sensor for obstacle detection
 * - Automatic obstacle avoidance
 */
class DiffDrive {
private:
    // Motors
    PrecisionMotor* motor_left;
    PrecisionMotor* motor_right;
    
    // Sensors
    UltrasonicSensor* ultrasonic;
    BMI160* imu;
    MadgwickFilter* filter;
    
    // Safety thresholds
    float obstacle_threshold_cm;
    float emergency_stop_threshold_cm;
    
    // Status flags
    bool is_warning;
    bool is_emergency_stopped;
    bool motors_initialized;
    bool sensors_initialized;
    
    // Current measurements
    float current_distance_cm;
    bool distance_valid;
    
    // Control parameters
    float dt;
    
    // Pin definitions
    uint motor_l_ena, motor_l_in1, motor_l_in2;
    uint encoder_l_a, encoder_l_b;
    uint motor_r_ena, motor_r_in1, motor_r_in2;
    uint encoder_r_a, encoder_r_b;
    uint ultrasonic_trig, ultrasonic_echo;
    uint imu_sda, imu_scl;
    i2c_inst_t* i2c_port;
    
    // Motor parameters
    float encoder_ticks_per_rev;
    float gear_ratio;
    float pid_kp, pid_ti, pid_td;

public:
    /**
     * @brief Constructor for DiffDrive robot
     * 
     * @param motor_l_ena PWM enable pin for left motor
     * @param motor_l_in1 Direction pin 1 for left motor
     * @param motor_l_in2 Direction pin 2 for left motor
     * @param encoder_l_a Encoder A pin for left motor
     * @param encoder_l_b Encoder B pin for left motor
     * @param motor_r_ena PWM enable pin for right motor
     * @param motor_r_in1 Direction pin 1 for right motor
     * @param motor_r_in2 Direction pin 2 for right motor
     * @param encoder_r_a Encoder A pin for right motor
     * @param encoder_r_b Encoder B pin for right motor
     * @param ultrasonic_trig Trigger pin for ultrasonic sensor
     * @param ultrasonic_echo Echo pin for ultrasonic sensor
     * @param imu_sda I2C SDA pin for IMU
     * @param imu_scl I2C SCL pin for IMU
     * @param i2c_port I2C port instance
     * @param encoder_ticks_per_rev Encoder ticks per motor revolution
     * @param gear_ratio Motor gear ratio
     * @param control_dt Control loop time interval
     * @param pid_kp PID proportional gain
     * @param pid_ti PID integral time constant
     * @param pid_td PID derivative time constant
     */
    DiffDrive(uint motor_l_ena, uint motor_l_in1, uint motor_l_in2,
              uint encoder_l_a, uint encoder_l_b,
              uint motor_r_ena, uint motor_r_in1, uint motor_r_in2,
              uint encoder_r_a, uint encoder_r_b,
              uint ultrasonic_trig, uint ultrasonic_echo,
              uint imu_sda, uint imu_scl, i2c_inst_t* i2c_port,
              float encoder_ticks_per_rev, float gear_ratio,
              float control_dt, float pid_kp, float pid_ti, float pid_td);
    
    /**
     * @brief Destructor
     */
    ~DiffDrive();
    
    /**
     * @brief Initialize all robot components
     * @return true if initialization successful, false otherwise
     */
    bool init();
    
    /**
     * @brief Update robot state (should be called regularly in control loop)
     * This updates motor control, reads sensors, and handles safety
     */
    void update();
    
    // === MOTOR CONTROL ===
    /**
     * @brief Set desired speed for left motor
     * @param speed_rpm Desired speed in RPM
     */
    void set_left_speed(float speed_rpm);
    
    /**
     * @brief Set desired speed for right motor
     * @param speed_rpm Desired speed in RPM
     */
    void set_right_speed(float speed_rpm);
    
    /**
     * @brief Set desired speeds for both motors
     * @param left_rpm Left motor speed in RPM
     * @param right_rpm Right motor speed in RPM
     */
    void set_speeds(float left_rpm, float right_rpm);
    
    /**
     * @brief Emergency stop - immediately stop all motors
     */
    void stop();
    
    // === GETTERS FOR MOTOR STATUS ===
    /**
     * @brief Get desired speed of left motor
     * @return Desired speed in RPM
     */
    float get_left_desired_speed() const;
    
    /**
     * @brief Get desired speed of right motor
     * @return Desired speed in RPM
     */
    float get_right_desired_speed() const;
    
    /**
     * @brief Get current speed of left motor
     * @return Current speed in RPM
     */
    float get_left_current_speed() const;
    
    /**
     * @brief Get current speed of right motor
     * @return Current speed in RPM
     */
    float get_right_current_speed() const;
    
    /**
     * @brief Get speed error of left motor
     * @return Speed error in RPM
     */
    float get_left_error() const;
    
    /**
     * @brief Get speed error of right motor
     * @return Speed error in RPM
     */
    float get_right_error() const;
    
    /**
     * @brief Get control output (power) of left motor
     * @return Control output (0.0 to 1.0)
     */
    float get_left_power() const;
    
    /**
     * @brief Get control output (power) of right motor
     * @return Control output (0.0 to 1.0)
     */
    float get_right_power() const;
    
    // === SENSOR GETTERS ===
    /**
     * @brief Get ultrasonic distance measurement
     * @return Distance in cm, or -1.0 if measurement invalid
     */
    float get_distance_cm() const;
    
    /**
     * @brief Get BMI160 accelerometer readings
     * @param ax Reference to store X acceleration (m/s²)
     * @param ay Reference to store Y acceleration (m/s²)
     * @param az Reference to store Z acceleration (m/s²)
     * @return true if readings valid, false otherwise
     */
    bool get_acceleration(float& ax, float& ay, float& az) const;
    
    /**
     * @brief Get BMI160 gyroscope readings
     * @param gx Reference to store X angular velocity (rad/s)
     * @param gy Reference to store Y angular velocity (rad/s)
     * @param gz Reference to store Z angular velocity (rad/s)
     * @return true if readings valid, false otherwise
     */
    bool get_gyroscope(float& gx, float& gy, float& gz) const;
    
    /**
     * @brief Get orientation quaternion from Madgwick filter
     * @param qx Reference to store quaternion X component
     * @param qy Reference to store quaternion Y component
     * @param qz Reference to store quaternion Z component
     * @param qw Reference to store quaternion W component
     * @return true if readings valid, false otherwise
     */
    bool get_quaternion(float& qx, float& qy, float& qz, float& qw) const;
    
    // === SAFETY STATUS ===
    /**
     * @brief Check if obstacle warning is active
     * @return true if object closer than warning threshold
     */
    bool is_warning_active() const;
    
    /**
     * @brief Check if emergency stop is active
     * @return true if motors stopped due to obstacle
     */
    bool is_emergency_stop_active() const;
    
    // === CONFIGURATION ===
    /**
     * @brief Set obstacle warning threshold
     * @param threshold_cm Distance in cm for warning
     */
    void set_obstacle_threshold(float threshold_cm);
    
    /**
     * @brief Get current obstacle warning threshold
     * @return Threshold in cm
     */
    float get_obstacle_threshold() const;
    
    /**
     * @brief Check if robot is properly initialized
     * @return true if all components initialized successfully
     */
    bool is_initialized() const;
};

#endif // DIFF_DRIVE_H