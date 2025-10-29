#include "PrecisionMotor.h"

// Initialize static members
std::map<uint, PrecisionMotor*> PrecisionMotor::motor_map;
bool PrecisionMotor::interrupt_initialized = false;

// Static interrupt handler - this is the ONLY interrupt function for ALL precision motors
void PrecisionMotor::encoder_irq_handler(uint gpio, uint32_t events) {
    // Route the interrupt to the correct motor instance
    auto it = motor_map.find(gpio);
    if (it != motor_map.end()) {
        it->second->handle_encoder_interrupt(gpio, events);
    }
}

// Constructor
PrecisionMotor::PrecisionMotor(uint ena_pin, uint in1_pin, uint in2_pin,
                               uint enc_a_pin, uint enc_b_pin,
                               int ticks_per_rev, float gear_ratio,float dt,
                               float kp, float ti, float td, float dead_zone)
    : Motor(ena_pin, in1_pin, in2_pin),
      ENCODER_A_PIN(enc_a_pin), ENCODER_B_PIN(enc_b_pin),
      TICKS_PER_REV(ticks_per_rev), GEAR_RATIO(gear_ratio), dt(dt),
      Kp(kp), Ti(ti), Td(td), dead_zone(dead_zone),
      encoder_ticks(0), last_ticks(0), filtered_rpm(0.0f), 
      setpoint_percentage(0.0f), control_output(0.0f) {
    
    // Initialize error array
    for (int i = 0; i < 3; i++) {
        error[i] = 0.0f;
    }
    
    // Calculate PID coefficients
    calculate_pid_coefficients();
    
    // Register this motor's encoder pins in the static map
    motor_map[ENCODER_A_PIN] = this;
    motor_map[ENCODER_B_PIN] = this;

    // Initialize encoder pins
    gpio_init(ENCODER_A_PIN);
    gpio_init(ENCODER_B_PIN);
    gpio_set_dir(ENCODER_A_PIN, GPIO_IN);
    gpio_set_dir(ENCODER_B_PIN, GPIO_IN);
    gpio_pull_up(ENCODER_A_PIN);
    gpio_pull_up(ENCODER_B_PIN);

    // Set up interrupts - but only ONCE for the entire system
    if (!interrupt_initialized) {
        // This sets up the callback for ALL GPIO interrupts
        gpio_set_irq_enabled_with_callback(ENCODER_A_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_irq_handler);
        interrupt_initialized = true;
    }
    
    // Enable interrupts for this motor's pins (without overwriting callback)
    gpio_set_irq_enabled(ENCODER_A_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(ENCODER_B_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
}

// Instance method to handle encoder interrupts
void PrecisionMotor::handle_encoder_interrupt(uint gpio, uint32_t events) {
    bool encoder_a = gpio_get(ENCODER_A_PIN);
    bool encoder_b = gpio_get(ENCODER_B_PIN);

    if (gpio == ENCODER_A_PIN) {
        // Process A pin interrupt
        if ((encoder_a && !encoder_b) || (!encoder_a && encoder_b)) {
            encoder_ticks++;  // Forward direction
        } else {
            encoder_ticks--;  // Reverse direction
        }
    } else if (gpio == ENCODER_B_PIN) {
        // Process B pin interrupt
        if ((encoder_a && !encoder_b) || (!encoder_a && encoder_b)) {
            encoder_ticks--;  // Reverse direction
        } else {
            encoder_ticks++;  // Forward direction
        }
    }
}

void PrecisionMotor::calculate_pid_coefficients() {
    float Ts = dt; // Sampling time
    q0 = Kp * (1.0f + Ts / (2.0f * Ti) + Td / Ts);
    q1 = Kp * (Ts / (2.0f * Ti) - 2.0f * Td / Ts - 1.0f);
    q2 = Kp * Td / Ts;
}

float PrecisionMotor::calculate_pid_control(float measured_rpm) {
    // Convert RPM to percentage for error calculation
    float measured_percent = (measured_rpm / MAX_RPM) * 100.0f;
    
    // Shift error history
    error[2] = error[1];  // e(k-2) = e(k-1)
    error[1] = error[0];  // e(k-1) = e(k)
    error[0] = setpoint_percentage - measured_percent;  // e(k) = setpoint - measurement
    
    // Calculate control output using difference equation
    // u(k) = q0*e(k) + q1*e(k-1) + q2*e(k-2) + u(k-1)
    // Use the stored control_output as u(k-1)
    float new_control_output = q0 * error[0] + q1 * error[1] + q2 * error[2] + control_output;

    // Saturate control output to [-100, 100] percentage
    if (new_control_output > 100.0f) new_control_output = 100.0f;
    if (new_control_output < -100.0f) new_control_output = -100.0f;

    
    
    return new_control_output;
}

void PrecisionMotor::set_motor(float desired_speed) {
    float measured_speed = 0.0f; // Placeholder, should be set by caller
    float revs = 0.0f;
    
    calculate_rpm(&revs, &measured_speed); // Update measured_speed with current

    set_setpoint(desired_speed);
    
    // Calculate control signal using PID
    float u = calculate_pid_control(measured_speed);
    
    // Store the control output for next iteration
    control_output = u;

    // Set direction and apply control
    if (u > 0) {
        moveFwd(u);  // Move forward with positive PWM
    } else if (u < 0) {
        moveBckwd(-u); // Move backward with positive PWM (negate the negative u)
    } else {
        stop();
    }
    
    // The Motor class methods (moveFwd/moveBckwd) already handle PWM setting
    // No need to manually override PWM here
}

void PrecisionMotor::calculate_rpm(float* revs, float* rpm) {
    int32_t ticks_since_last = encoder_ticks - last_ticks;
    last_ticks = encoder_ticks;

    *revs = (float)encoder_ticks / TICKS_PER_REV;

    float raw_rpm = ((float)ticks_since_last / TICKS_PER_REV) * (60.0f / dt) * (1.0f / GEAR_RATIO);
    *rpm = apply_low_pass_filter(raw_rpm);
}

float PrecisionMotor::apply_low_pass_filter(float raw_rpm) {
    filtered_rpm = alpha * raw_rpm + (1.0f - alpha) * filtered_rpm;
    return filtered_rpm;
}

void PrecisionMotor::set_setpoint(float rpm) {
    if (rpm > MAX_RPM) rpm = MAX_RPM;
    if (rpm < -MAX_RPM) rpm = -MAX_RPM;
    setpoint_percentage = (rpm / MAX_RPM) * 100.0f; // Convert RPM to percentage
}

void PrecisionMotor::reset_encoder_ticks() {
    encoder_ticks = 0;
    last_ticks = 0;
    filtered_rpm = 0.0f;
}

void PrecisionMotor::reset_pid_state() {
    // Clear error history
    error[0] = 0.0f;
    error[1] = 0.0f;
    error[2] = 0.0f;
    
    // Reset control output (prevents integral windup)
    control_output = 0.0f;
    
    // Clear filtered velocity
    filtered_rpm = 0.0f;
}

float PrecisionMotor::get_setpoint() const { 
    return setpoint_percentage; 
}

float PrecisionMotor::get_control_output() const { 
    return control_output; // Return the actual control output with proper sign
}

float PrecisionMotor::get_filtered_rpm() const { 
    return filtered_rpm; 
}

float PrecisionMotor::get_error() const { 
    return error[0]; 
}

int32_t PrecisionMotor::get_encoder_ticks() const {
    return encoder_ticks;
}

float PrecisionMotor::get_position_degrees() const {
    // Calculate position in degrees based on encoder ticks, ticks per revolution, and gear ratio
    float motor_revolutions = (float)encoder_ticks / (TICKS_PER_REV * GEAR_RATIO);
    return motor_revolutions * 360.0f; // Convert to degrees
}

float PrecisionMotor::get_dt() const {
    return dt;
}