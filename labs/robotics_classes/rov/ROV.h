#ifndef ROV_H
#define ROV_H

#include "../motor/Motor.h"
#include "../motor/PrecisionMotor.h"
#include "../sensor/TEMT6000.h"
#include "../sensor/MPS20N0040D.h"
#include <pico/stdlib.h>

class ROV {
private:
    // Motor components
    Motor thruster1;           // Left/Port thruster
    Motor thruster2;           // Right/Starboard thruster
    PrecisionMotor ballast_motor;  // Precision motor for ballast control
    
    // Sensor components
    TEMT6000 light_sensor;     // Luminosity sensor
    MPS20N0040D pressure_sensor; // Pressure sensor for depth measurement
    
    // Control variables
    float current_depth;       // Current depth reading (kPa converted to depth)
    float current_luminosity;  // Current luminosity reading (0-100%)
    float desired_depth;       // Setpoint for depth control
    float depth_error;         // Error for depth control (desired - current)
    bool automatic_mode;       // true = automatic depth control, false = manual
    
    // Control parameters
    static constexpr float MANUAL_BALLAST_SPEED = 50.0f; // RPM for manual ballast operation
    static constexpr float PRESSURE_TO_DEPTH_FACTOR = 10.197f; // kPa to meters (approx 1kPa = 10.197cm water)
    
    // Depth control method (to be called by controlDepth)
    void adjustBallast(bool fill_ballast, float speed_reference);

public:
    // Constructor - initializes all components
    ROV(uint thruster1_ena, uint thruster1_in1, uint thruster1_in2,
        uint thruster2_ena, uint thruster2_in1, uint thruster2_in2,
        uint ballast_ena, uint ballast_in1, uint ballast_in2, 
        uint ballast_enc_a, uint ballast_enc_b,
        uint light_sensor_pin, 
        uint pressure_out_pin, uint pressure_sck_pin);
    
    // Sensor reading methods
    float getDepth();          // Returns current depth in meters
    float getLuminosity();     // Returns current luminosity (0-100%)
    
    // Depth control methods
    void setDepth(float depth_setpoint);    // Sets desired depth in meters
    void controlDepth();       // Main depth control method (call every dt)
    
    // Thruster control methods
    void setThrusterLevels(int thruster_levels[2]); // Array: [left, right] (-100 to 100)
    void stop();               // Emergency stop - stops all motors
    
    // Mode control
    void setAutomaticMode(bool is_automatic);
    bool isAutomaticMode() const;
    
    // Getters for monitoring
    float getCurrentDepth() const;
    float getDesiredDepth() const;
    float getDepthError() const;
    float getCurrentLuminosity() const;
    
    // Manual ballast control (for manual mode)
    void manualBallastControl(bool fill_ballast); // true = fill, false = empty
};

#endif // ROV_H