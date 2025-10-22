#ifndef SOIL_MOIST_H
#define SOIL_MOIST_H

#include "pico/stdlib.h"
#include "hardware/adc.h"

/**
 * @brief Class for resistive moisture sensor with digital and analog outputs
 * 
 * This sensor is used as a safeguard to detect water entering the ROV.
 * It provides two operation modes:
 * - Digital: Outputs HIGH when moisture threshold is exceeded, LOW otherwise
 * - Analog: Provides a continuous reading that can be mapped to 0-100% moisture
 */
class SOIL_MOIST {
private:
    uint digital_pin;      // GPIO pin for digital threshold output
    uint analog_pin;       // GPIO pin for analog moisture reading (ADC)
    uint adc_channel;      // ADC channel number (0-2 for RP2040)
    bool last_digital_value;  // Last digital reading stored
    float last_analog_value;  // Last analog percentage stored
    
    /**
     * @brief Convert ADC pin number to ADC channel
     * @param pin GPIO pin number (26, 27, or 28 for ADC0-2)
     * @return ADC channel number or 255 on error
     */
    uint pin_to_adc_channel(uint pin);

public:
    /**
     * @brief Construct a new SOIL_MOIST sensor
     * @param digital_pin GPIO pin for digital output (HIGH when wet)
     * @param analog_pin GPIO pin for analog output (ADC pin: 26, 27, or 28)
     */
    SOIL_MOIST(uint digital_pin, uint analog_pin);
    
    /**
     * @brief Read the digital output (threshold detection)
     * @return true if moisture detected (above threshold), false otherwise
     */
    bool readDigital();
    
    /**
     * @brief Read the analog output and map to moisture percentage
     * @return Moisture level as percentage (0.0 = dry, 100.0 = wet)
     */
    float readAnalog();
    
    /**
     * @brief Get the last stored digital reading
     * @return Last digital state
     */
    bool getDigitalValue();
    
    /**
     * @brief Get the last stored analog reading
     * @return Last moisture percentage
     */
    float getAnalogValue();
    
    /**
     * @brief Check if moisture is detected (convenience method)
     * @return true if moisture detected, false otherwise
     */
    bool isMoistureDetected();
};

#endif // SOIL_MOIST_H
