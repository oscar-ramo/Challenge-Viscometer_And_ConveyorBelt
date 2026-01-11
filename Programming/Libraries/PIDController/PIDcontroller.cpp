/*
 * Library: PIDcontroller
 * File: PIDcontroller.cpp
 * Author: Oscar Gadiel Ramo Martínez
 * 
 * Description:
 *   Implementation of PID controller with safety checks and anti-windup.
 *   Calculates control output based on error signal using P, I, and D terms.
 * 
 * Date: November - December 2025
 * Course: 5th Semester Challenge - ITESM
 * License: MIT
 */

#include "PIDcontroller.h"
#include <cmath>
#include <cstdio>

PIDcontroller::PIDcontroller() 
{
}

void PIDcontroller::setup(float gains[3], float dt) 
{
    _Kp = gains[0];
    _Ki = gains[1];
    _Kd = gains[2];
    _dt = dt;
}

float PIDcontroller::calculate(float error) 
{
    // SAFETY: Reject invalid inputs immediately
    if (std::isnan(error) || std::isinf(error)) {
        error = 0.0f;  // Treat as zero error if invalid
    }
    
    // Proportional
    float p = _Kp * error;

    // Derivative
    float d = _Kd * (error - _prev_error) / _dt;
    // SAFETY: Check derivative for NaN (from dt=0)
    if (std::isnan(d) || std::isinf(d)) d = 0.0f;

    // Integral
    _integral += (_dt/2) * (error + _prev_error);
    _prev_error = error;
    // SAFETY: Clamp integral to prevent runaway
    const float I_MAX = 200.0f;  // Tune based on your system
    if (_integral > I_MAX) _integral = I_MAX;
    if (_integral < -I_MAX) _integral = -I_MAX;
    if (std::isnan(_integral) || std::isinf(_integral)) _integral = 0.0f;  // Reset if corrupted
    float i = _Ki * _integral;

    u = p + i + d;

    return u;
}
