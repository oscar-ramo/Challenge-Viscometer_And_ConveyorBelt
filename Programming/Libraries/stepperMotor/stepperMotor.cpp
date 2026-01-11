/*
 * Library: stepperMotor
 * File: stepperMotor.cpp
 * Author: Oscar Gadiel Ramo Martínez
 * 
 * Description:
 *   Implementation of stepper motor controller with PID position control.
 *   Uses interrupt-based pulse counting for accurate position feedback.
 *   Implements non-blocking position control with PID regulation.
 * 
 * Date: November - December 2025
 * Course: 5th Semester Challenge - ITESM
 * License: MIT
 */

#include "stepperMotor.h"

StepperMotor::StepperMotor()
{
}

StepperMotor::~StepperMotor()
{
}

void StepperMotor::setup(uint8_t gpio_pins[2], uint8_t pul_pin, uint8_t pul_feedback_pin, uint8_t pul_ch, 
                         TimerConfig *pul_config, float duty_percentage, uint32_t frequency, 
                         uint32_t pulses_per_rev, bool enable)
{
    // Initialize GPIO pins for direction and enable
    stepper_dir.setup(gpio_pins[0], GPIO_MODE_OUTPUT);
    stepper_ena.setup(gpio_pins[1], GPIO_MODE_OUTPUT);
    stepper_pul.setup(pul_pin, pul_ch, pul_config);

    // Store configuration
    _duty_percentage = duty_percentage;
    _pulses_per_rev = pulses_per_rev;
    _gpio_pul = (gpio_num_t)pul_feedback_pin;  // Store GPIO pin for pulse feedback
    stepper_ena.set(enable);
    
    // Initialize pulse counter to 0
    _pulse_count = 0;
    _last_pulse_count = 0;
    
    // Configure GPIO for pulse feedback (INPUT, rising edge trigger)
    // This GPIO receives pulses from TB6600 stepper driver feedback
    gpio_config_t gpio_cfg = {};
    gpio_cfg.pin_bit_mask = (1ULL << pul_feedback_pin);
    gpio_cfg.mode = GPIO_MODE_INPUT;
    gpio_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_cfg.intr_type = GPIO_INTR_POSEDGE;  // Trigger on rising edge (pulse detection)
    gpio_config(&gpio_cfg);
    
    // Enable GPIO interrupt service
    // This must be called at least once to enable global GPIO interrupt handling
    gpio_install_isr_service(0);
    
    // Attach ISR handler to this GPIO pin
    // Lambda captures 'this' to call member function from static ISR context
    // When pulse edge detected, pulseHandler() will be called
    gpio_isr_handler_add(_gpio_pul, [](void* arg) {
        StepperMotor* motor = (StepperMotor*)arg;
        motor->pulseHandler();
    }, this);
    
    // Set PWM duty to 0% to ensure motor is stopped initially
    stepper_pul.setDuty(0);
    
    _frequency = frequency;
    _current_frequency = 0;
    
    printf("Stepper setup complete: Motor stopped, pulse counter ready (GPIO %u)\n", pul_feedback_pin);
}

void IRAM_ATTR StepperMotor::pulseHandler()
{
    // ISR callback: Called on every rising edge of pulse feedback from TB6600
    // This happens in interrupt context - keep it minimal and fast!
    // IRAM_ATTR places this function in fast on-chip RAM for low latency
    
    // Increment pulse counter atomically (64-bit increment is atomic on ESP32)
    _pulse_count = _pulse_count + 1;
}

void StepperMotor::setupPID(float gains[3], float dt)
{
    pid.setup(gains, dt);
}

void StepperMotor::setDirection(bool direction)
{
    stepper_pul.setDuty(_duty_percentage);  // Ensure duty cycle for square wave
    if (direction) 
    {
        stepper_dir.set(1); // CW
    } 
    else 
    {
        stepper_dir.set(0); // CCW
    }
    printf("Setting stepper frequency to %lu Hz\n", _frequency);
    stepper_pul.setFrequency(_frequency);
    _current_frequency = _frequency;  // Track for virtual encoder
    _direction = direction;
    _is_moving = true;
}

void StepperMotor::setCW()
{
    stepper_pul.setDuty(_duty_percentage);  // Ensure duty cycle for square wave
    stepper_dir.set(1); // CW
    stepper_pul.setFrequency(_frequency);
    _current_frequency = _frequency;  // Track for virtual encoder
    _direction = true;  // CW direction
    _is_moving = true;
}

void StepperMotor::setCCW() 
{
    stepper_pul.setDuty(_duty_percentage);  // Ensure duty cycle for square wave
    stepper_dir.set(0); // CCW
    stepper_pul.setFrequency(_frequency);
    _current_frequency = _frequency;  // Track for virtual encoder
    _direction = false;  // CCW direction
    _is_moving = true;
}

void StepperMotor::setFrequency(uint32_t frequency)
{
    stepper_pul.setDuty(_duty_percentage);  // Ensure duty cycle for square wave
    stepper_pul.setFrequency(frequency);
    _current_frequency = frequency;
    if (frequency > 0) {
        _is_moving = true;
    } else {
        _is_moving = false;
    }
    printf("Stepper frequency set to %lu Hz\n", frequency);
}

uint32_t StepperMotor::readFrequency()
{
    return _current_frequency;
}

float StepperMotor::readSpeed()
{
    // Convert current frequency to speed in degrees/second
    // speed (deg/s) = frequency (Hz) * degrees_per_pulse
    if (_pulses_per_rev == 0) {
        return 0.0f;  // Safety check
    }
    float degrees_per_pulse = 360.0f / _pulses_per_rev;
    return (float)_current_frequency * degrees_per_pulse;
}

float StepperMotor::getFrequencyFromSpeed(float speed_degs)
{
    // Convert desired speed (deg/s) to PWM frequency (Hz)
    // frequency = speed / degrees_per_step
    if (_pulses_per_rev == 0) {
        printf("ERROR: _pulses_per_rev is 0!\n");
        return 0.0f;  // Safety: return 0 instead of dividing by zero
    }
    
    float degrees_per_step = 360.0f / _pulses_per_rev;
    if (degrees_per_step == 0.0f) {
        printf("ERROR: degrees_per_step is 0!\n");
        return 0.0f;
    }
    
    return speed_degs / degrees_per_step;
}

float StepperMotor::desiredAngle(float angle)
{
    // Update pulse-based position first
    updatePosition();
    
    // Calculate position error - ALWAYS MOVE CLOCKWISE (CW) DIRECTION ONLY
    // If target is behind current position, go the long way around (CW)
    float error = angle - _current_angle;
    
    // Force CW movement: if target is "behind" (negative error), wrap around
    // Example: current=350°, target=10° → error=-340° → wrap to +20° (CW path)
    if (error < 0.0f) {
        error += 360.0f;  // Convert CCW path to CW path
    }
    
    // Check if at target (within acceptable tolerance)
    // Increased tolerance to 20 pulses (~9°) to prevent oscillation
    float tolerance = 20.0f * (360.0f / _pulses_per_rev);  // 20 pulses = ~9° for 800 ppr
    if (error <= tolerance) {
        // Only stop if we're actually moving (prevents repeated PID calls)
        if (_is_moving) {
            stop();
            printf("Stepper reached target: current=%.2f°, target=%.2f°, error=%.2f°\n", _current_angle, angle, error);
        }
        return 0.0f;  // Close enough - within tolerance
    }
    
    // PID control: calculate desired speed in deg/s
    float desired_speed = pid.calculate(error);  // deg/s
    
    // Convert speed to frequency
    float frequency = getFrequencyFromSpeed(fabs(desired_speed));
    
    // Clamp to hardware limits
    if (frequency > _max_frequency) {
        frequency = _max_frequency;
    }
    
    // CRITICAL: Enforce minimum frequency to avoid LEDC errors
    // With 10-bit resolution and 5000Hz base, minimum is ~100Hz
    if (frequency < 100.0f) {
        frequency = 100.0f;  // Clamp to safe minimum
    }
    
    // Always move CW (direction = true)
    setDirection(true);  // Force CW direction
    stepper_pul.setFrequency((uint32_t)frequency);
    _current_frequency = (uint32_t)frequency;
    _is_moving = true;
    
    return error;
}

void StepperMotor::stop() 
{
    stepper_pul.setDuty(0.0f);  // Stop motor by setting duty to 0%
    _current_frequency = 0;  // Reset virtual encoder tracking
    _is_moving = false;
}

void StepperMotor::homePosition()
{
    updatePosition();  // Update to latest position first
    // Reset current angle to zero (without physical movement)
    _current_angle = 0.0f;
    // Reset pulse counters to zero
    _pulse_count = 0;
    _last_pulse_count = 0;
}

float StepperMotor::readAngle()
{
    updatePosition();  // Sync pulse counter to calculate latest angle
    return _current_angle;
}

bool StepperMotor::isMoving()
{
    return _is_moving;
}

void StepperMotor::updatePosition()
{
    // REAL PULSE COUNTER: Calculate position from actual pulses detected via GPIO interrupt
    // ISR increments _pulse_count on every rising edge of pulse feedback from TB6600
    
    // Calculate change in pulse count since last read
    int64_t pulse_delta = _pulse_count - _last_pulse_count;
    
    if (pulse_delta == 0) {
        return;  // No new pulses since last update
    }
    
    // Update the last pulse count for next iteration
    _last_pulse_count = _pulse_count;
    
    // Convert pulses to degrees
    // Each pulse = 360° / pulses_per_rev
    float degrees_per_pulse = 360.0f / _pulses_per_rev;
    
    // Apply direction to the pulse delta
    // ISR just counts all pulses, but we need to know direction for position
    // _direction: true = CW (positive angle), false = CCW (negative angle)
    float degrees_moved;
    if (_direction) {
        degrees_moved = pulse_delta * degrees_per_pulse;  // CW: positive
    } else {
        degrees_moved = -(pulse_delta * degrees_per_pulse);  // CCW: negative
    }
    
    // Update angle with signed movement
    _current_angle += degrees_moved;
    
    // Wrap angle to 0-360 range
    // Ensures angle is always in valid stepper position range
    while (_current_angle >= 360.0f) _current_angle -= 360.0f;
    while (_current_angle < 0.0f) _current_angle += 360.0f;
}