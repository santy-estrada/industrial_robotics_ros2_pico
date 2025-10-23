#ifndef MPS20N0040D_H
#define MPS20N0040D_H

#include <hardware/gpio.h>
#include <pico/stdlib.h>

class MPS20N0040D {
private:
    const uint OUT_PIN;         // Data output pin (equivalent to MPS_OUT_pin)
    const uint SCK_PIN;         // Clock pin (equivalent to MPS_SCK_pin)
    uint8_t gain;               // Gain setting for HX710B
    bool pins_configured;       // Track if pins are initialized
    long last_reading;          // Store last sensor reading
    
    // Constants for HX710B communication
    static constexpr uint8_t GAIN_128 = 1;  // 128 gain = 1 extra clock pulse
    static constexpr uint8_t GAIN_64 = 3;   // 64 gain = 3 extra clock pulses  
    static constexpr uint8_t GAIN_32 = 2;   // 32 gain = 2 extra clock pulses
    static constexpr int DEFAULT_AVG_SIZE = 10;  // Default averaging samples
    static constexpr uint32_t TIMEOUT_US = 10000; // 10ms timeout - quick fail for motor interference, prevents callback blocking

    //Constant for pressure conversion
    float ATMOSPHERIC_RAW = 4847500.0f;  // Raw value at 0 kPa gauge pressure

    // Private methods for HX710B communication protocol
    bool readyToSend();
    long readRaw();
    void configurePins();

    // Private method to calibrate
    void calibrate();

public:
    // Constructor
    MPS20N0040D(uint out_pin, uint sck_pin);
    
    // Set gain (128, 64, or 32)
    void setGain(uint8_t gain_value = 128);
    
    // Read single raw value from sensor
    long read();
    
    // Read averaged value (multiple samples for stability)
    float readAveraged(int avg_size = DEFAULT_AVG_SIZE);
    
    // Get last stored reading
    long getLastReading() const;
    
    // Check if sensor is ready to provide data
    bool isReady();
};

#endif // MPS20N0040D_H