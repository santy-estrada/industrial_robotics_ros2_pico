#include "LimitSwitch.h"

LimitSwitch::LimitSwitch(uint switch_pin) 
    : SWITCH_PIN(switch_pin), switch_value(false) {
    
    // Initialize GPIO pin as input
    gpio_init(SWITCH_PIN);
    gpio_set_dir(SWITCH_PIN, GPIO_IN);
    
    // Configure as pull-up (characteristic of limit switches)
    gpio_pull_up(SWITCH_PIN);
    
    // Initialize with current state
    switch_value = !gpio_get(SWITCH_PIN);  // Inverted because pull-up means LOW when pressed
}

bool LimitSwitch::read() {
    // Read GPIO state (inverted because of pull-up configuration)
    // When switch is pressed: GPIO reads LOW (0) -> switch_value = true
    // When switch is released: GPIO reads HIGH (1) -> switch_value = false
    switch_value = !gpio_get(SWITCH_PIN);
    
    return switch_value;
}

bool LimitSwitch::getValue() const {
    return switch_value;
}

bool LimitSwitch::isPressed() const {
    return switch_value;
}

bool LimitSwitch::isReleased() const {
    return !switch_value;
}