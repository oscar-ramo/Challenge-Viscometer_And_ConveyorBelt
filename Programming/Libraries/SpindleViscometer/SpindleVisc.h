/*
 * Library: SpindleVisc
 * File: SpindleVisc.h
 * Author: Oscar Gadiel Ramo Martínez
 * 
 * Description:
 *   High-level spindle motor controller with integrated viscosity sensing.
 *   Combines DC motor control with current-based viscosity measurement for
 *   soap production quality control. Uses exponential model for viscosity
 *   estimation based on motor current.
 *   
 *   Features:
 *     - DC motor speed control (open-loop and PID closed-loop)
 *     - Real-time viscosity measurement via motor current
 *     - Exponential viscosity model (viscosity = A * e^(B * water%))
 *     - Multiple viscosity reading averaging
 *     - Current conditioning for measurement accuracy
 *     - Configuration struct for easy setup
 * 
 * Viscosity Model:
 *   Based on exponential relationship between water percentage and viscosity.
 *   Calibration coefficients A and B determined experimentally.
 * 
 * Usage:
 *   SpindleVisc spindle;
 *   spindle.setup(spindle_config);
 *   spindle.setupPID(gains, dt);
 *   spindle.setSpeed(30.0f);  // 30 RPM
 *   float visc = spindle.measure_viscosity_readings();
 * 
 * Date: November - December 2025
 * Course: 5th Semester Challenge - ITESM
 * License: MIT
 */

#ifndef __SPINDLEVISC_H__
#define __SPINDLEVISC_H__

// Libraries
#include "dcMotorController.h"
#include "SimpleADC.h"
#include <math.h>

// Configuration structure for SpindleVisc setup
struct SpindleConfig {
    // Motor/H-Bridge configuration
    uint8_t *hbridge_pins;      // [DIR_PIN, PWM_PIN]
    uint8_t *hbridge_channels;  // [PWM_CH, unused]
    TimerConfig *motor_config;
    
    // Encoder configuration
    uint8_t *encoder_pins;      // [ENC_A, ENC_B]
    float degrees_per_edge;
    int64_t encoder_timeout_us;
    
    // Viscosity sensor configuration
    uint64_t viscosity_adc_pin;
    float max_voltage;          // mV at max concentration
    float min_voltage;          // mV at min concentration
    float max_water_percentage;
    float min_water_percentage;

    // Slope and intercept for feedforward control
    float m_slope;
    float b_intercept;

    // Slope and intercept for current conditioning
    float m_conditioning;
    float b_conditioning;
};

class SpindleVisc 
{
public:
    SpindleVisc();
    ~SpindleVisc();

    void setup(const SpindleConfig &config);

    void setupPID(float gains[3], float dt);

    void setPWM(float duty ); // Open loop control
    float setSpeed(float rpm); // Closed loop control
    void setPosition(float angle); // Position control

    float getAngle();
    float getSpeed();
    int getDirection();
    float getCurrent();

    float measure_viscosity_readings();
    void resetViscosityMeasurement();  // Reset averaging for new measurement cycle

private:
    dcMotorController spindle;
    SimpleADC viscosity_reading;
    float _max_voltage;
    float _min_voltage;
    float _max_water_percentage;
    float _min_water_percentage;
    
    // Viscosity measurement state
    float _viscosity_sum = 0.0f;
    int _sample_count = 0;
    int64_t _prev_visc_time = 0;

    float _current_voltage = 0.0f;
    float _viscosity_cP = 0.0f;
    float _m_conditioning = 0.0f; // Slope for current conditioning
    float _b_conditioning = 0.0f; // Intercept for current conditioning
};

#endif // __SPINDLEVISC_H__