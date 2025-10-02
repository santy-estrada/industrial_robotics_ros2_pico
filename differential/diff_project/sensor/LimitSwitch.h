#ifndef LIMITSWITCH_H
#define LIMITSWITCH_H

#include <hardware/gpio.h>
#include <pico/stdlib.h>

class LimitSwitch {
private:
    const uint SWITCH_PIN;      // GPIO pin for limit switch
    bool switch_value;          // Stored switch state (true = pressed, false = released)

public:
    // Constructor
    LimitSwitch(uint switch_pin);
    
    // Read switch state and store it (returns true if pressed, false if released)
    bool read();
    
    // Get the last stored switch state
    bool getValue() const;
    
    // Check if switch is currently pressed (convenience method)
    bool isPressed() const;
    
    // Check if switch is currently released (convenience method)
    bool isReleased() const;
};

#endif // LIMITSWITCH_H