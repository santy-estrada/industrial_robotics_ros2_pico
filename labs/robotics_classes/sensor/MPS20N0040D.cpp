#include "MPS20N0040D.h"

MPS20N0040D::MPS20N0040D(uint out_pin, uint sck_pin) 
    : OUT_PIN(out_pin), SCK_PIN(sck_pin), gain(GAIN_128), 
      pins_configured(false), last_reading(0) {
    // Constructor - pins will be configured on first use
}

void MPS20N0040D::configurePins() {
    if (!pins_configured) {
        // Initialize clock pin as output
        gpio_init(SCK_PIN);
        gpio_set_dir(SCK_PIN, GPIO_OUT);
        gpio_put(SCK_PIN, 0);  // Start with clock LOW
        
        // Initialize data pin as input
        gpio_init(OUT_PIN);
        gpio_set_dir(OUT_PIN, GPIO_IN);
        
        pins_configured = true;
    }
}

bool MPS20N0040D::readyToSend() {
    configurePins();
    // HX710B is ready when OUT pin is LOW
    return !gpio_get(OUT_PIN);
}

bool MPS20N0040D::isReady() {
    return readyToSend();
}

void MPS20N0040D::setGain(uint8_t gain_value) {
    switch (gain_value) {
        case 128:
            gain = GAIN_128;
            break;
        case 64:
            gain = GAIN_64;
            break;
        case 32:
            gain = GAIN_32;
            break;
        default:
            gain = GAIN_128;  // Default to 128 if invalid
            break;
    }
    
    // Apply gain setting by doing a read operation
    gpio_put(SCK_PIN, 0);
    readRaw();
}

long MPS20N0040D::readRaw() {
    configurePins();
    
    // Wait for sensor to be ready with timeout
    uint32_t timeout_start = time_us_32();
    while (!readyToSend()) {
        if ((time_us_32() - timeout_start) > TIMEOUT_US) {
            return last_reading;  // Return last known value on timeout
        }
        sleep_us(1);
    }

    uint8_t data[3] = {0};

    // Read 3 bytes of data using bit-banging (equivalent to Arduino's shiftIn)
    for (int byte_idx = 2; byte_idx >= 0; byte_idx--) {
        for (int bit_idx = 7; bit_idx >= 0; bit_idx--) {
            // Clock HIGH
            gpio_put(SCK_PIN, 1);
            sleep_us(1);  // Small delay for signal stability
            
            // Read bit when clock is HIGH
            if (gpio_get(OUT_PIN)) {
                data[byte_idx] |= (1 << bit_idx);
            }
            
            // Clock LOW
            gpio_put(SCK_PIN, 0);
            sleep_us(1);  // Small delay for signal stability
        }
    }

    // Set gain - send additional clock pulses
    for (int i = 0; i < gain; i++) {
        gpio_put(SCK_PIN, 1);
        sleep_us(1);
        gpio_put(SCK_PIN, 0);
        sleep_us(1);
    }

    // Convert 24-bit two's complement to signed long (same as Arduino version)
    data[2] ^= 0x80;
    long result = ((uint32_t)data[2] << 16) | ((uint32_t)data[1] << 8) | (uint32_t)data[0];
    
    last_reading = result;
    return result;
}

long MPS20N0040D::read() {
    return readRaw();
}

float MPS20N0040D::readAveraged(int avg_size) {
    if (avg_size <= 0) {
        avg_size = DEFAULT_AVG_SIZE;
    }
    
    float avg_val = 0.0f;
    
    for (int i = 0; i < avg_size; i++) {
        avg_val += (float)readRaw();
        if (i < avg_size - 1) {  // Don't delay after last reading
            sleep_ms(50);  // Same delay as Arduino example
        }
    }
    
    avg_val /= avg_size;
    // Datasheet claims: offset is +25mV and that S ≈ 50mV/40kPa
    avg_val = ((avg_val / 8388608.0f) * 3.3f - 0.025f) * (40.0f / 0.05f); // Convert to kPa
    
    return avg_val;
}

long MPS20N0040D::getLastReading() const {
    return last_reading;
}