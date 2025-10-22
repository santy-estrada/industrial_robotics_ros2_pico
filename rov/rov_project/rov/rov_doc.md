# ROV (Remotely Operated Vehicle) Class

## Overview
The ROV class integrates multiple components to create a complete underwater vehicle control system with cascade depth control, thruster management, and sensor monitoring.

## Components Integrated
- **2 Simple Motors (Thrusters)**: Left/Port and Right/Starboard propulsion
- **1 Precision Motor (Ballast)**: Ballast tank control for depth management  
- **Luminosity Sensor (TEMT6000)**: Light level monitoring
- **Pressure Sensor (MPS20N0040D)**: Depth measurement via pressure

## Control Architecture

### Cascade Depth Control System
```
Desired Depth → [OUTER CONTROLLER] → Speed Reference → [INNER CONTROLLER] → Ballast Motor
     ↑                                                        ↓
Current Depth ← [PRESSURE SENSOR] ← Actual Depth ← [BALLAST SYSTEM]
```

**Outer Loop (Position Controller):**
- Input: Depth error (meters)
- Output: Speed reference (RPM) for ballast motor

**Inner Loop (Speed Controller):**
- Handled automatically by PrecisionMotor class
- Input: Speed reference from outer controller
- Output: PWM signals to ballast motor

## Pin Configuration

### Constructor Parameters:
```cpp
ROV rov(thruster1_ena, thruster1_in1, thruster1_in2,    // Thruster 1 pins
        thruster2_ena, thruster2_in1, thruster2_in2,    // Thruster 2 pins  
        ballast_ena, ballast_in1, ballast_in2,          // Ballast motor pins
        ballast_enc_a, ballast_enc_b,                    // Ballast encoder pins
        light_sensor_pin,                                // Light sensor (ADC pin)
        pressure_out_pin, pressure_sck_pin);             // Pressure sensor pins
```

### Example Pin Assignment:
```cpp
// Thrusters (Simple Motors)
uint thruster1_ena = 0, thruster1_in1 = 1, thruster1_in2 = 2;   // Left thruster
uint thruster2_ena = 3, thruster2_in1 = 4, thruster2_in2 = 5;   // Right thruster

// Ballast Motor (Precision Motor)  
uint ballast_ena = 6, ballast_in1 = 7, ballast_in2 = 8;
uint ballast_enc_a = 9, ballast_enc_b = 10;

// Sensors
uint light_pin = 28;        // ADC2
uint pressure_out = 11;     // HX710B OUT pin
uint pressure_sck = 12;     // HX710B SCK pin
```

## Usage Examples

### Basic ROV Initialization
```cpp
#include "rov/ROV.h"

ROV rov(0,1,2,  3,4,5,  6,7,8,9,10,  28,  11,12);
```

### Manual Control Mode
```cpp
// Set to manual mode
rov.setAutomaticMode(false);

// Control thrusters manually
int thruster_levels[2] = {50, 50};  // Both thrusters forward at 50%
rov.setThrusterLevels(thruster_levels);

// Manual ballast control
rov.manualBallastControl(true);   // Fill ballast (go deeper)
rov.manualBallastControl(false);  // Empty ballast (go shallower)
```

### Automatic Depth Control Mode
```cpp
// Set to automatic mode
rov.setAutomaticMode(true);

// Set desired depth
rov.setDepth(2.5f);  // 2.5 meters depth

// Call controlDepth() every dt milliseconds in your main loop
// This handles the cascade control automatically
rov.controlDepth();
```

### Sensor Monitoring
```cpp
float current_depth = rov.getDepth();          // Get depth in meters
float brightness = rov.getLuminosity();        // Get light level (0-100%)
float depth_error = rov.getDepthError();       // Get control error
```

### Emergency Stop
```cpp
rov.stop();  // Immediately stops all motors
```

## Control Law Implementation

The `controlDepth()` method contains a placeholder for your control law. Replace the placeholder with your actual control algorithm:

```cpp
// In ROV.cpp, inside controlDepth() method:
// TODO: IMPLEMENT YOUR DEPTH CONTROL LAW HERE

// Example PID Controller:
float kp = 10.0f, ki = 0.1f, kd = 1.0f;
static float integral_error = 0.0f;
static float previous_error = 0.0f;

integral_error += depth_error * dt;
float derivative_error = (depth_error - previous_error) / dt;

float ballast_speed_reference = kp * depth_error + 
                               ki * integral_error + 
                               kd * derivative_error;

previous_error = depth_error;
```

## Timing Recommendations

### Main Loop Structure:
```cpp
uint64_t last_control_time = 0;
const uint64_t CONTROL_PERIOD_MS = 100;  // 100ms = 10Hz control rate

while (true) {
    uint64_t current_time = to_ms_since_boot(get_absolute_time());
    
    if (current_time - last_control_time >= CONTROL_PERIOD_MS) {
        // Update sensors and run control
        rov.getLuminosity();  // Update light reading
        rov.controlDepth();   // Run depth control loop
        
        last_control_time = current_time;
    }
    
    // Handle thruster commands, user input, etc.
    sleep_ms(10);
}
```

## Safety Features

1. **Emergency Stop**: `stop()` method immediately stops all motors
2. **Input Clamping**: Thruster levels automatically clamped to ±100%
3. **Mode Protection**: Manual ballast control disabled in automatic mode
4. **Sensor Timeouts**: Pressure sensor has built-in timeout protection

## Monitoring and Debugging

The ROV class provides extensive printf output for monitoring:
- Initialization status
- Mode changes (Manual/Automatic)
- Control system status (depth, error, commands)
- Thruster and ballast commands
- Emergency stop notifications

## Notes on Cascade Control Approach

Your proposed approach is excellent for ROV depth control:

1. **Consistent Sampling**: Calling `controlDepth()` every dt ensures consistent timing
2. **Cascade Structure**: Outer depth controller → Inner speed controller provides better disturbance rejection
3. **Mode Switching**: Manual/Automatic modes allow for testing and emergency override
4. **Sensor Integration**: Pressure-to-depth conversion handles the physical relationship

This architecture matches professional ROV control systems and provides a solid foundation for underwater vehicle control.