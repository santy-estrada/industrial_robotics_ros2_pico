# Differential Drive Robot - micro-ROS Integration

This project provides a complete micro-ROS integration for a differential drive robot using the Raspberry Pi Pico.

## Features

- **Differential Drive Control**: Two precision motors with PID control
- **BMI160 IMU Sensor**: 6-DOF inertial measurement with Madgwick filter
- **Ultrasonic Range Sensor**: HC-SR05 compatible distance measurement
- **Safety System**: Automatic obstacle detection and emergency stop
- **Real-time Communication**: micro-ROS integration with ROS2

## Hardware Configuration

### Motor Pins
- **Left Motor**: PWM(16), DIR1(18), DIR2(17), ENCA(13), ENCB(12)
- **Right Motor**: PWM(21), DIR1(19), DIR2(20), ENCA(14), ENCB(15)

### Sensor Pins
- **BMI160 IMU**: I2C SDA(10), SCL(11)
- **Ultrasonic**: TRIG(27), ECHO(26)
- **Status LED**: GPIO(25)

### Motor Parameters
- **Encoder Resolution**: 28 ticks per revolution
- **Gear Ratio**: 150:1
- **Max Speed**: 200 RPM
- **Control Frequency**: 10Hz (100ms)

## ROS2 Interface

### Published Topics

#### `/diff_measurements` (geometry_msgs/Twist)
- **linear.x**: Left wheel speed (RPM)
- **angular.z**: Right wheel speed (RPM)
- **Frequency**: 10Hz

#### `/imu` (sensor_msgs/Imu)
- **orientation**: Filtered quaternion from Madgwick filter
- **angular_velocity**: Gyroscope readings (rad/s)
- **linear_acceleration**: Accelerometer readings (m/s²)
- **Frequency**: 10Hz

#### `/ultrasonic` (sensor_msgs/Range)
- **range**: Distance measurement (meters)
- **min_range**: 0.02m, **max_range**: 4.0m
- **radiation_type**: ULTRASOUND
- **Frequency**: 10Hz

### Subscribed Topics

#### `/wheel_setpoint` (geometry_msgs/Twist)
- **linear.x**: Left wheel setpoint (RPM)
- **angular.z**: Right wheel setpoint (RPM)

## Safety Features

- **Warning Threshold**: 5cm (configurable)
- **Emergency Stop**: 4cm (automatic motor shutdown)
- **Status Monitoring**: Real-time safety alerts via console

## Build Instructions

1. **Prerequisites**:
   ```bash
   # Install Pico SDK and micro-ROS dependencies
   # Ensure PICO_SDK_PATH is set
   ```

2. **Build**:
   ```bash
   cd uros_diff_class
   mkdir build && cd build
   cmake ..
   make -j4
   ```

3. **Flash**:
   ```bash
   # Copy uros_diff_obj.uf2 to Pico in BOOTSEL mode
   ```

## Usage Example

### Start micro-ROS agent:
```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
```

### Send wheel commands:
```bash
# Move forward at 50 RPM
ros2 topic pub /wheel_setpoint geometry_msgs/Twist "{linear: {x: 50.0}, angular: {z: 50.0}}"

# Turn left (left wheel slower)
ros2 topic pub /wheel_setpoint geometry_msgs/Twist "{linear: {x: 30.0}, angular: {z: 50.0}}"

# Stop
ros2 topic pub /wheel_setpoint geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

### Monitor robot state:
```bash
# View wheel speeds
ros2 topic echo /diff_measurements

# View IMU data
ros2 topic echo /imu

# View distance sensor
ros2 topic echo /ultrasonic
```

## Console Output

The robot provides detailed console output:
```
=== Differential Drive Robot ===
Connected to Micro-ROS agent.
DiffDrive robot initialized successfully!
DIFF: Received wheel setpoints - Left:50.00 rpm, Right:50.00 rpm
DIFF: WARNING - Obstacle detected!
DIFF: EMERGENCY STOP - Motors stopped!
```

## Architecture

The system uses the **DiffDrive class** which encapsulates:
- **Motor Control**: PrecisionMotor with PID controllers
- **Sensor Management**: BMI160 IMU and UltrasonicSensor
- **Safety Systems**: Automatic obstacle avoidance
- **State Publishing**: Real-time robot status

All sensor data is published at 10Hz with proper ROS2 message formatting and timestamps.