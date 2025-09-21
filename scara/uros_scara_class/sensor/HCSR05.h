#ifndef HCSR05_H
#define HCSR05_H

#include <hardware/gpio.h>
#include <pico/stdlib.h>

class HCSR05 {
private:
    const uint TRIG_PIN;        // Trigger pin for HC-SR05
    const uint ECHO_PIN;        // Echo pin for HC-SR05
    long measured_distance;     // Current measured distance in cm
    long last_known_distance;   // Last valid distance for fallback
    
    // Constants from original implementation
    static constexpr uint32_t TIMEOUT_US = 3000;    // 3ms timeout for fast response
    static constexpr long MIN_DISTANCE_CM = 2;      // HC-SR05 minimum range
    static constexpr long MAX_DISTANCE_CM = 400;    // HC-SR05 maximum range
    static constexpr long DEFAULT_DISTANCE = 100;   // Default safe distance
    static constexpr uint32_t TRIGGER_PULSE_US = 10; // 10μs trigger pulse
    static constexpr long DISTANCE_FACTOR = 58;     // Conversion factor for cm

public:
    // Constructor
    HCSR05(uint trig_pin, uint echo_pin);
    
    // Read distance measurement and store it (returns distance in cm)
    long read();
    
    // Get the last stored distance measurement
    long getDistance() const;
    
    // Get the last known good distance (fallback value)
    long getLastKnownDistance() const;
    
    // Check if distance reading is valid (within HC-SR05 range)
    bool isValidDistance(long distance) const;
};

#endif // HCSR05_H