#ifndef ROV_H
#define ROV_H

#include "../motor/Motor.h"
#include "../sensor/LimitSwitch.h"
#include "../sensor/SOIL_MOIST.h"
#include "../sensor/MPS20N0040D.h"
#include <pico/stdlib.h>
#include <cmath>

class ROV {
private:
    // Motor components
    Motor thrust_motor1;       // Thrust motor 1
    Motor thrust_motor2;       // Thrust motor 2
    Motor ballast_motor;       // Ballast motor with limit switch control
    
    // Sensor components
    LimitSwitch full_switch;   // Full ballast limit switch
    LimitSwitch empty_switch;  // Empty ballast limit switch
    SOIL_MOIST moisture_sensor; // Water intrusion detection
    MPS20N0040D pressure_sensor; // Pressure sensor for depth measurement
    
    // Control variables
    float current_depth;       // Current depth reading (converted from pressure)
    
    // Placeholder conversion factors (user can modify these)
    float pressure_offset;     // Pressure offset for zero depth (to be calibrated)
    float pressure_to_depth_factor; // Conversion factor from pressure units to depth

    // Calibrate 0 depth - store raw pressure at zero depth
    float zero_depth_pressure;  // Raw pressure reading at zero depth (initialized to 0)
    
    // Safety flag
    bool moisture_detected;    // true if water detected inside ROV

public:
    // Constructor - initializes all components
    ROV(uint thrust1_ena, uint thrust1_in1, uint thrust1_in2,
        uint thrust2_ena, uint thrust2_in1, uint thrust2_in2,
        uint ballast_ena, uint ballast_in1, uint ballast_in2,
        uint full_switch_pin, uint empty_switch_pin,
        uint moisture_digital_pin, uint moisture_analog_pin,
        uint pressure_out_pin, uint pressure_sck_pin);
    
    // Depth sensor methods
    float getDepth();          // Returns current depth using pressure sensor with placeholder conversion
    void setDepthConversion(float offset, float factor); // Set placeholder conversion values
    
    // Safety sensor methods
    bool isMoistureDetected(); // Returns true if water detected inside ROV
    bool isFullSwitchPressed();  // Returns true if ballast is full
    bool isEmptySwitchPressed(); // Returns true if ballast is empty
    
    // Thrust control methods (range: -1.0 to 1.0, blocked if moisture detected)
    void setThrustMotor1(float thrust_value); // Set thrust motor 1 (-1.0 to 1.0)
    void setThrustMotor2(float thrust_value); // Set thrust motor 2 (-1.0 to 1.0)
    void setThrust(float thrust_value);       // Set both thrust motors to same value
    
    // Ballast control method (range: -1.0 to 1.0, exponential mapping, limit switch protected)
    void setBallast(float ballast_value);     // Set ballast motor with exponential mapping

    // Calibrate depth
    void calibrateZeroDepth(); // Calibrate current depth as zero depth
    
    // Emergency stop
    void stop();               // Emergency stop - stops all motors
    
    //Emergency emerging
    void emergencyEmerge();    // Emergency emerge - full ballast in reverse
};

#endif // ROV_H