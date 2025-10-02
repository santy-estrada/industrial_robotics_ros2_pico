#include "TEMT6000.h"

TEMT6000::TEMT6000(uint sensor_pin) 
    : SENSOR_PIN(sensor_pin), luminosity_value(0.0f) {
    
    // Initialize ADC
    adc_init();
    
    // Make sure GPIO is high-impedance, no pullups etc
    adc_gpio_init(SENSOR_PIN);
    
    // Initialize the stored value
    luminosity_value = 0.0f;
}

float TEMT6000::read() {
    // Select ADC input (ADC0 = GPIO26, ADC1 = GPIO27, ADC2 = GPIO28)
    uint adc_channel = SENSOR_PIN - 26;  // Convert GPIO pin to ADC channel
    adc_select_input(adc_channel);
    
    // Read raw ADC value (0-4095 for 12-bit ADC)
    uint16_t raw_value = adc_read();
    
    // Convert to voltage (0-3.3V reference)
    float voltage = (float)raw_value * 3.3f / 4095.0f;
    
    // Map voltage to percentage (0-100%)
    // TEMT6000 typically outputs 0V (dark) to ~3.3V (bright)
    luminosity_value = map(voltage, 0.0f, 3.3f, 0.0f, 100.0f);
    
    // Clamp to 0-100% range
    if (luminosity_value < 0.0f) luminosity_value = 0.0f;
    if (luminosity_value > 100.0f) luminosity_value = 100.0f;
    
    return luminosity_value;
}

float TEMT6000::getValue() const {
    return luminosity_value;
}

float TEMT6000::map(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}