/*
 * Library: HBridge
 * File: HBridge.cpp
 * Author: Oscar Gadiel Ramo Martínez
 * 
 * Description:
 *   Implementation of H-bridge motor driver with bidirectional control.
 *   Manages PWM duty cycle polarity for forward/reverse operation.
 * 
 * Date: November - December 2025
 * Course: 5th Semester Challenge - ITESM
 * License: MIT
 */

#include "HBridge.h"

HBridge::HBridge()
{

}

void HBridge::setup(uint8_t pin[2], uint8_t ch[2], TimerConfig *config)
{
    for (size_t i = 0; i < 2; i++)
        pwm[i].setup(pin[i], ch[i], config);
}

void HBridge::setDuty(float duty)
{
    if (duty < 0)  // Forward direction
    {
        pwm[0].setDuty(-duty);
        pwm[1].setDuty(0.0f);
    }
    else  // Reverse direction (dir == 0)
    {
        pwm[0].setDuty(0.0f);
        pwm[1].setDuty(duty);
    }
}

void HBridge::setStop()
{
    pwm[0].setDuty(0.0f);
    pwm[1].setDuty(0.0f);
}