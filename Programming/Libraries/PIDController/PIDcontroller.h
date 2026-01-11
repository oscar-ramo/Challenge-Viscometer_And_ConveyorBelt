/*
 * Library: PIDcontroller
 * File: PIDcontroller.h
 * Author: Oscar Gadiel Ramo Martínez
 * 
 * Description:
 *   PID (Proportional-Integral-Derivative) controller implementation
 *   for closed-loop control systems. Includes safety checks for NaN/Inf
 *   and anti-windup protection.
 *   
 *   Features:
 *     - Standard PID algorithm
 *     - NaN/Inf safety checks
 *     - Integral anti-windup
 *     - Configurable gains (Kp, Ki, Kd)
 *     - Auto-tuning placeholder
 * 
 * Usage:
 *   PIDcontroller pid;
 *   pid.setup(gains, dt);
 *   float control = pid.calculate(error);  // Call each control cycle
 * 
 * Date: November - December 2025
 * Course: 5th Semester Challenge - ITESM
 * License: MIT
 */

#ifndef __PIDCONTROLLER_H__
#define __PIDCONTROLLER_H__

class PIDcontroller 
{
    public:
        PIDcontroller();
        void setup(float gains[3], float dt);
        float calculate(float error);
        float autoTune(float setpoint); // Simple auto-tuning method
    private:
        float _Kp, _Ki, _Kd, _dt, u;
        float _prev_error = 0.0f;
        float _integral = 0.0f;
};

#endif // __PIDCONTROLLER_H__