/*
 * Library: dcMotorController
 * File: dcMotorController.cpp
 * Author: Oscar Gadiel Ramo Martínez
 * 
 * Description:
 *   Implementation of DC motor controller with PID-based speed control
 *   and feedforward compensation for improved tracking performance.
 * 
 * Date: November - December 2025
 * Course: 5th Semester Challenge - ITESM
 * License: MIT
 */

#include "dcMotorController.h"

dcMotorController::dcMotorController()
{
}

dcMotorController::~dcMotorController()
{
}

void dcMotorController::setup(uint8_t hbridge_pins[2], uint8_t hbridge_ch[2], TimerConfig *motor_config,
                               uint8_t encoder_pins[2], float degrees_per_edge,
                               int64_t encoder_timeout_us,
                               float m_slope, float b_intercept)
{
    motor.setup(hbridge_pins, hbridge_ch, motor_config);
    encoder.setup(encoder_pins, degrees_per_edge, encoder_timeout_us);
    _m_slope = m_slope;
    _b_intercept = b_intercept;
}

void dcMotorController::setupPID(float gains[3], float dt)
{
    pid.setup(gains, dt);
    motor.setDuty(0.0f);
}

void dcMotorController::setPWM(float duty)
{
    motor.setDuty(duty);
}

// Corrected setSpeed function
float dcMotorController::setSpeed(float rpm)
{
    float desired_rpm_speed = rpm;
    float current_rpm_speed = encoder.getSpeed() * 60.0f / 360.0f;  // Convert deg/s to RPM

    float error_rpm = desired_rpm_speed - current_rpm_speed;
    
    // FEEDFORWARD: Use calibration data to get baseline PWM for desired speed
    float desired_speed_degs = rpm * 360.0f / 60.0f;  // Convert RPM to deg/s
    float pwm_feedforward = _m_slope * desired_speed_degs + _b_intercept; // EXCEL linear fit (deg/s to PWM)
    
    // FEEDBACK: PID correction for disturbances (load changes, friction, etc.)
    float pwm_correction = pid.calculate(error_rpm);
    
    // Combine feedforward (open-loop prediction) + feedback (closed-loop correction)
    float u_pwm = pwm_feedforward + pwm_correction;
    
    // Clamp PWM to valid range [-100, 100]
    if (u_pwm > 100.0f) u_pwm = 100.0f;
    if (u_pwm < -100.0f) u_pwm = -100.0f;
    
    motor.setDuty(u_pwm);
    
    return error_rpm;
}

void dcMotorController::setPosition(float angle)
{
    float current_angle = encoder.getAngle();
    float error = angle - current_angle;

    // Control
    float u = pid.calculate(error);
    motor.setDuty(u);
}

float dcMotorController::getAngle()
{
    return encoder.getAngle();
}

float dcMotorController::getSpeed()
{
    return encoder.getSpeed();
}

int dcMotorController::getDirection() 
{
    return encoder.getDirection();
}