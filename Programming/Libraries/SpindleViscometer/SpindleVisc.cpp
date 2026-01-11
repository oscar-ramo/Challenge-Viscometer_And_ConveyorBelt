/*
 * Library: SpindleVisc
 * File: SpindleVisc.cpp
 * Author: Oscar Gadiel Ramo Martínez
 * 
 * Description:
 *   Implementation of spindle motor controller with viscosity measurement.
 *   Integrates motor control with current sensing for real-time viscosity estimation.
 * 
 * Date: November - December 2025
 * Course: 5th Semester Challenge - ITESM
 * License: MIT
 */

#include "SpindleVisc.h"

SpindleVisc::SpindleVisc() 
{
}

SpindleVisc::~SpindleVisc() 
{
}

void SpindleVisc::setup(const SpindleConfig &config)
{
    spindle.setup(config.hbridge_pins, config.hbridge_channels, config.motor_config, 
                  config.encoder_pins, config.degrees_per_edge, config.encoder_timeout_us, 
                  config.m_slope, config.b_intercept);
    viscosity_reading.setup(config.viscosity_adc_pin);
    _max_voltage = config.max_voltage;
    _min_voltage = config.min_voltage;
    _max_water_percentage = config.max_water_percentage;
    _min_water_percentage = config.min_water_percentage;
    _m_conditioning = config.m_conditioning;
    _b_conditioning = config.b_conditioning;
}

void SpindleVisc::setupPID(float gains[3], float dt)
{
    spindle.setupPID(gains, dt);
}

void SpindleVisc::setPWM(float duty)
{
    spindle.setPWM(duty);
}

float SpindleVisc::setSpeed(float rpm)
{
    return spindle.setSpeed(rpm);
}

void SpindleVisc::setPosition(float angle)
{
    spindle.setPosition(angle);
}

float SpindleVisc::getAngle()
{
    return spindle.getAngle();
}

float SpindleVisc::getSpeed()
{
    return spindle.getSpeed();
}

int SpindleVisc::getDirection() 
{
    return spindle.getDirection();
}

float SpindleVisc::getCurrent()
{
    return (_current_voltage - _b_conditioning) / _m_conditioning;
}

// Measure viscosity readings
float SpindleVisc::measure_viscosity_readings()
{
    const int64_t dt_us = 50000; // Sample every 50 ms

    // Calibration coefficients (100% soap = 2350 mV, 75% soap = 100 mV)
    float max_volt_cP = 1975.6f * expf(-0.071f * _min_water_percentage);
    float min_volt_cP = 1975.6f * expf(-0.071f * _max_water_percentage);

    // Calculate exponential model: cP = A * exp(B * voltage)
    float B = logf(max_volt_cP / min_volt_cP) / (_max_voltage - _min_voltage);
    float A = min_volt_cP / expf(B * _min_voltage);

    // Time polling for sampling rate
    int64_t current_time = esp_timer_get_time();
    if (current_time - _prev_visc_time >= dt_us) 
    {
        _prev_visc_time = current_time;
        
        // Read voltage and calculate instantaneous viscosity
        _current_voltage = viscosity_reading.read(ADC_READ_MV);
        _viscosity_cP = A * expf(B * _current_voltage);
        
        // Accumulate for running average
        _viscosity_sum += _viscosity_cP;
        _sample_count++;
        
        float average_viscosity = _viscosity_sum / _sample_count;
        
        // Debug output
        printf("ADC: %d | Voltage: %.1f mV | Viscosity: %.2f cP | Average: %.2f cP (n=%d)\n", 
               viscosity_reading.read(ADC_READ_RAW), _current_voltage, _viscosity_cP, 
               average_viscosity, _sample_count);
        
        return average_viscosity;
    }
    
    // Return previous average if not time to sample yet
    return (_sample_count > 0) ? (_viscosity_sum / _sample_count) : 0.0f;
}

// Reset viscosity measurement averaging (call before starting new measurement cycle)
void SpindleVisc::resetViscosityMeasurement()
{
    _viscosity_sum = 0.0f;
    _sample_count = 0;
    _prev_visc_time = 0;
    printf("Viscosity measurement reset - starting new cycle\n");
}
