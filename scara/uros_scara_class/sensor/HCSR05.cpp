#include "HCSR05.h"

HCSR05::HCSR05(uint trig_pin, uint echo_pin) 
    : TRIG_PIN(trig_pin), ECHO_PIN(echo_pin), 
      measured_distance(DEFAULT_DISTANCE), last_known_distance(DEFAULT_DISTANCE) {
    
    // Initialize trigger pin (output)
    gpio_init(TRIG_PIN);
    gpio_set_dir(TRIG_PIN, GPIO_OUT);
    gpio_put(TRIG_PIN, 0);  // Initialize trigger low
    
    // Initialize echo pin (input)
    gpio_init(ECHO_PIN);
    gpio_set_dir(ECHO_PIN, GPIO_IN);
    
    // Initialize with safe default distances
    measured_distance = DEFAULT_DISTANCE;
    last_known_distance = DEFAULT_DISTANCE;
}

long HCSR05::read() {
    // Send trigger pulse (HC-SR05 needs at least 10μs trigger pulse)
    gpio_put(TRIG_PIN, 1);          // Set trigger high
    sleep_us(TRIGGER_PULSE_US);     // Wait 10 microseconds (optimal for HC-SR05)
    gpio_put(TRIG_PIN, 0);          // Set trigger low
    
    // Wait for echo to go high (start of pulse) - REDUCED timeout for timing control
    uint32_t timeout = time_us_32() + TIMEOUT_US; // 3ms timeout (much faster)
    while (!gpio_get(ECHO_PIN)) {
        if (time_us_32() > timeout) {
            measured_distance = last_known_distance; // Return last known distance on timeout
            return measured_distance;
        }
    }
    
    // Measure pulse duration
    uint32_t start_time = time_us_32();
    
    // Wait for echo to go low (end of pulse) - REDUCED timeout
    timeout = start_time + TIMEOUT_US; // 3ms timeout for echo pulse
    while (gpio_get(ECHO_PIN)) {
        if (time_us_32() > timeout) {
            measured_distance = last_known_distance; // Return last known distance on timeout
            return measured_distance;
        }
    }
    
    uint32_t end_time = time_us_32();
    
    // Calculate distance in cm
    long pulse_time = end_time - start_time;
    long distance = pulse_time / DISTANCE_FACTOR;
    
    // Validate distance (HC-SR05 range: 2cm to 400cm)
    if (isValidDistance(distance)) {
        measured_distance = distance;
        last_known_distance = distance;    // Update last known good distance
        return measured_distance;
    } else {
        measured_distance = last_known_distance; // Return last known distance if invalid reading
        return measured_distance;
    }
}

long HCSR05::getDistance() const {
    return measured_distance;
}

long HCSR05::getLastKnownDistance() const {
    return last_known_distance;
}

bool HCSR05::isValidDistance(long distance) const {
    return (distance >= MIN_DISTANCE_CM && distance <= MAX_DISTANCE_CM);
}