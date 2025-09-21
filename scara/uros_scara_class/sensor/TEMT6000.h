#ifndef TEMT6000_H
#define TEMT6000_H

#include <hardware/gpio.h>
#include <hardware/adc.h>
#include <pico/stdlib.h>

class TEMT6000 {
private:
    const uint SENSOR_PIN;      // Analog pin for luminosity sensor
    float luminosity_value;     // Stored sensor reading (0-100%)
    
    // Map function to scale values from one range to another
    float map(float x, float in_min, float in_max, float out_min, float out_max);

public:
    // Constructor
    TEMT6000(uint sensor_pin);
    
    // Read sensor value and store it (returns percentage 0-100%)
    float read();
    
    // Get the last stored sensor value
    float getValue() const;
};

#endif // TEMT6000_H