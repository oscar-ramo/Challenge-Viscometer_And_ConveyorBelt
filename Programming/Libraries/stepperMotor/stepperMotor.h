/*
 * Library: stepperMotor
 * File: stepperMotor.h
 * Author: Oscar Gadiel Ramo Martínez
 * 
 * Description:
 *   Stepper motor controller with closed-loop position control.
 *   Designed for TB6600 driver with pulse/direction interface.
 *   Includes ISR-based pulse counting for position feedback and PID control
 *   for precise angle positioning.
 *   
 *   Features:
 *     - Pulse/Direction control (compatible with TB6600, DRV8825, etc.)
 *     - CW/CCW direction control
 *     - Position control with PID (angle-based)
 *     - Speed control (frequency-based)
 *     - ISR-based pulse counting for feedback
 *     - Homing routine
 *     - Real-time angle and speed reading
 * 
 * Usage:
 *   StepperMotor stepper;
 *   stepper.setup(pins, pul_pin, feedback_pin, channel, &config, duty, freq, ppr);
 *   stepper.setupPID(gains, dt);          // For closed-loop
 *   stepper.desiredAngle(90.0f);          // Move to 90°
 *   stepper.setFrequency(200);            // Open-loop speed
 *   float angle = stepper.readAngle();
 * 
 * Date: November - December 2025
 * Course: 5th Semester Challenge - ITESM
 * License: MIT
 */

#ifndef _STEPPERMOTOR_H_
#define _STEPPERMOTOR_H_

#include "SimpleGPIO.h"
#include "SimplePWM.h"
#include "PIDcontroller.h"
#include "esp_timer.h"
#include <driver/gpio.h>
#include <esp_attr.h>
#include <cmath>

class StepperMotor 
{
public:
    StepperMotor();
    ~StepperMotor();

    void setup(uint8_t gpio_pins[2], uint8_t pul_pin, uint8_t pul_feedback_pin, uint8_t pul_ch, TimerConfig *pul_config,
               float duty_percentage, uint32_t frequency, uint32_t pulses_per_rev, bool enable = false // False for TB6600
              );
    void setupPID(float gains[3], float dt);

    void setDirection(bool direction);
    void setCW();
    void setCCW();
    void setFrequency(uint32_t frequency);
    float desiredAngle(float angle);
    void stop();
    void homePosition();

    float readAngle();
    uint32_t readFrequency(); // Read current PWM frequency
    float readSpeed(); // Read current speed in degrees/second

private:
    void updatePosition(); // Update virtual encoder position (called internally)
    float getFrequencyFromSpeed(float speed_degs);
    bool isMoving(); // Check if motor is currently moving
    void IRAM_ATTR pulseHandler();  // ISR for counting pulses (like Ultrasonic::handler)
    
    SimplePWM stepper_pul;
    SimpleGPIO stepper_dir;
    SimpleGPIO stepper_ena;
    PIDcontroller pid;
    
    // Interrupt-based pulse counter (like Ultrasonic echo timing)
    gpio_num_t _gpio_pul;  // GPIO pin for pulse input
    volatile int64_t _pulse_count = 0;  // Total pulses counted by ISR
    volatile int64_t _last_pulse_count = 0;  // Previous pulse count for delta calculation

    uint32_t _frequency;
    uint32_t _pulses_per_rev;
    uint32_t _pulse_delay_us;
    float _current_angle = 0.0f;  // Track position (virtual encoder)
    bool _is_moving = false;       // Track movement state
    bool _direction = true;        // true = CW, false = CCW (INITIALIZED)
    
    // Virtual encoder tracking
    uint32_t _current_frequency = 0;  // Current PWM frequency (Hz)
    uint64_t _last_update_time = 0;   // Last position update timestamp (us)
    float _pulse_accumulator = 0.0f;  // Fractional pulse tracking
    float _m_slope;
    float _b_intercept;
    bool _enable;
    float _max_frequency = 1000.0f;  // Max stepper frequency (Hz) - tune for your motor!
    float _duty_percentage;
};

#endif // _STEPPERMOTOR_H_