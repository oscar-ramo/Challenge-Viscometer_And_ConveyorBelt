/*
 * Library: HBridge
 * File: HBridge.h
 * Author: Oscar Gadiel Ramo Martínez
 * 
 * Description:
 *   H-bridge motor driver abstraction for bidirectional DC motor control.
 *   Uses two PWM channels to control motor direction and speed.
 *   
 *   Features:
 *     - Bidirectional control (positive/negative duty cycle)
 *     - Smooth braking (both channels to 0)
 *     - Dual SimplePWM channel management
 *   
 * Usage:
 *   HBridge motor;
 *   motor.setup(pins, channels, &timer_config);
 *   motor.setDuty(50.0f);   // Forward at 50%
 *   motor.setDuty(-30.0f);  // Reverse at 30%
 *   motor.setStop();         // Brake
 * 
 * Date: November - December 2025
 * Course: 5th Semester Challenge - ITESM
 * License: MIT
 */

#ifndef _HBRIDGE_H
#define _HBRIDGE_H

#include "SimplePWM.h"

class HBridge
{
public:
    HBridge();
    void setup(uint8_t pin[2], uint8_t ch[2], TimerConfig *config);
    void setDuty(float duty);
    void setStop();
    
private:
    SimplePWM pwm[2];
};

#endif // _HBRIDGE_H