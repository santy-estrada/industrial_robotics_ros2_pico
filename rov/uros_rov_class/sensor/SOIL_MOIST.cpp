#include "SOIL_MOIST.h"

uint SOIL_MOIST::pin_to_adc_channel(uint pin) {
    // RP2040 ADC channels mapping:
    // GPIO26 -> ADC0
    // GPIO27 -> ADC1
    // GPIO28 -> ADC2
    if (pin == 26) return 0;
    if (pin == 27) return 1;
    if (pin == 28) return 2;
    return 255; // Invalid channel
}

SOIL_MOIST::SOIL_MOIST(uint digital_pin, uint analog_pin) 
    : digital_pin(digital_pin), 
      analog_pin(analog_pin),
      last_digital_value(false),
      last_analog_value(0.0f) {
    
    // Initialize digital pin as input
    if(digital_pin != 255) { // Ensure valid pin
        gpio_init(digital_pin);
        gpio_set_dir(digital_pin, GPIO_IN);
        gpio_pull_up(digital_pin);  // Pull down to avoid floating state
        digital_conf = true;
    } else {
        digital_conf = false;
    }
    
    
    // Initialize analog pin
    adc_channel = pin_to_adc_channel(analog_pin);
    if (adc_channel != 255) {
        adc_init();
        adc_gpio_init(analog_pin);
        analog_conf = true;
    } else {
        analog_conf = false;
    }
}

bool SOIL_MOIST::readDigital() {
    if(!digital_conf) {
        return false; // Invalid configuration
    }
    last_digital_value = !gpio_get(digital_pin);
    return last_digital_value;
}

float SOIL_MOIST::readAnalog() {
    if (adc_channel == 255 || !analog_conf) {
        return 0.0f;  // Invalid ADC channel
    }
    
    // Select the ADC channel
    adc_select_input(adc_channel);
    
    // Read raw ADC value (0-4095 for 12-bit ADC)
    uint16_t raw_value = adc_read();
    
    // Convert to percentage (0-100%)
    // Higher ADC value = less moisture --> It must be inverted
    last_analog_value = 100.0f - (raw_value / 4095.0f) * 100.0f;
    // If you want direct mapping (higher ADC = more moisture), use the line below instead:
    // last_analog_value = (raw_value / 4095.0f) * 100.0f;
    
    return last_analog_value;
}

bool SOIL_MOIST::getDigitalValue() {
    return last_digital_value;
}

float SOIL_MOIST::getAnalogValue() {
    return last_analog_value;
}

bool SOIL_MOIST::isMoistureDetected() {
    return readDigital();
}
