/*
 * Library: dcMotorController
 * File: dcMotorController.h
 * Author: Oscar Gadiel Ramo Martínez
 * 
 * Description:
 *   Comprehensive DC motor controller with multiple control modes.
 *   Integrates H-bridge driver, quadrature encoder feedback, and PID control
 *   for precise speed and position control.
 *   
 *   Control Modes:
 *     - Open-loop: Direct PWM duty cycle control
 *     - Speed control: Closed-loop RPM/deg/s regulation with PID
 *     - Position control: Angle-based positioning (future implementation)
 *   
 *   Features:
 *     - Feedforward compensation (m*setpoint + b)
 *     - Real-time encoder feedback
 *     - Velocity and position reading
 *     - Anti-windup PID
 * 
 * Usage:
 *   dcMotorController motor;
 *   motor.setup(hbridge_pins, channels, &config, encoder_pins, deg_per_edge);
 *   motor.setupPID(gains, dt);
 *   motor.setSpeed(target_rpm);  // Closed-loop
 *   motor.setPWM(duty);          // Open-loop
 * 
 * Date: November - December 2025
 * Course: 5th Semester Challenge - ITESM
 * License: MIT
 */

#ifndef __DCMOTORCONTROLLER_H__
#define __DCMOTORCONTROLLER_H__

// libraries
#include "HBridge.h"
#include "QuadratureEncoder.h"
#include "PIDcontroller.h"

// Control modes
enum ControlMode {
    MODE_OPEN_LOOP = 0,   // Direct PWM (no PID)
    MODE_SPEED = 1,       // Speed control (RPM or deg/s)
    MODE_POSITION = 2     // Position control (angle)
};

class dcMotorController
{
public:
    dcMotorController();
    ~dcMotorController();

    void setup(uint8_t hbridge_pins[2], uint8_t hbridge_ch[2], TimerConfig *motor_config,
               uint8_t encoder_pins[2], float degrees_per_edge,
               int64_t encoder_timeout_us = 100000,
               float m_slope = 0.0548f, float b_intercept = 8.6803f);
    
    void setupPID(float gains[3], float dt);

    void setPWM(float duty ); // Open loop control
    float setSpeed(float rpm); // Closed loop control
    void setPosition(float angle); // Position control

    float getAngle();
    float getSpeed();
    int getDirection();

private:
    HBridge motor;
    QuadratureEncoder encoder;
    PIDcontroller pid;
    float _m_slope;
    float _b_intercept;
};

#endif // __DCMOTORCONTROLLER_H__