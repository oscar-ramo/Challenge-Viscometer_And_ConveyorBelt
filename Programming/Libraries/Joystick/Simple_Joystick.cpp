/*
 * Library: Simple_Joystick
 * File: Simple_Joystick.cpp
 * Author: Oscar Gadiel Ramo Martínez
 * 
 * Description:
 *   Implementation of 2-axis analog joystick controller.
 *   Handles ADC reading, calibration, and button debouncing.
 * 
 * Date: November - December 2025
 * Course: 5th Semester Challenge - ITESM
 * License: MIT
 */

#include "Simple_Joystick.h"

Joystick2D::Joystick2D()
{
    // Defaults
    minX = 4095; maxX = 0;
    minY = 4095; maxY = 0;
    neutralMin = 0;   // optional, not used by your original logic
    neutralMax = 0;   // optional, not used by your original logic
    button_active_low = 1; // your example treated pressed as get()==0

    delta_time_us = JOY_DELTA_TIME_US;
    cal_time_us   = JOY_CAL_TIME_US;

    _last_sample_us = 0;
    _cal_start_us   = 0;

    _pinX = (int)JOY_X_PIN;
    _pinY = (int)JOY_Y_PIN;
    _pinBtn = (int)JOY_BTN_PIN;
}

void Joystick2D::setup(int pinX, int pinY, int pinBtn)
{
    _pinX = pinX;
    _pinY = pinY;
    _pinBtn = pinBtn;

    _x.setup(_pinX);
    _y.setup(_pinY);
    _btn.setup(_pinBtn, GPIO_MODE_INPUT);

    // reset ranges for a fresh calibration
    minX = 4095; maxX = 0;
    minY = 4095; maxY = 0;
}

void Joystick2D::startCalibration()
{
    _cal_start_us   = esp_timer_get_time();
    _last_sample_us = 0;

    // reset ranges again at start
    minX = 4095; maxX = 0;
    minY = 4095; maxY = 0;
}

int Joystick2D::calibrationStep()
{
    uint64_t now = esp_timer_get_time();

    // still within calibration window?
    if (now - _cal_start_us <= (uint64_t)cal_time_us)
    {
        // sample at intervals
        if (_last_sample_us == 0 || (now - _last_sample_us) >= (uint64_t)delta_time_us)
        {
            int vx = _x.read(ADC_READ_RAW);
            int vy = _y.read(ADC_READ_RAW);

            if (vx < minX) minX = vx;
            if (vx > maxX) maxX = vx;
            if (vy < minY) minY = vy;
            if (vy > maxY) maxY = vy;

            _last_sample_us = now;
        }
        return 0; // still calibrating
    }

    // optional deadzone if you want it later (not required by your sample code)
    // int midX = (minX + maxX) / 2;
    // int spanX = (maxX - minX);
    // neutralMin = midX - spanX/8;
    // neutralMax = midX + spanX/8;

    return 1; // done
}

int Joystick2D::readXRaw()
{
    return _x.read(ADC_READ_RAW);
}

int Joystick2D::readYRaw()
{
    return _y.read(ADC_READ_RAW);
}

int Joystick2D::isPressed()
{
    int v = _btn.get();
    if (button_active_low) return (v == 0);
    return (v == 1);
}

int Joystick2D::dirX()
{
    int vx = _x.read(ADC_READ_RAW);
    if (vx < minX) return -1;  // left
    if (vx > maxX) return +1;  // right
    return 0;                  // neutral
}

int Joystick2D::dirY()
{
    int vy = _y.read(ADC_READ_RAW);
    if (vy < minY) return -1;  // down
    if (vy > maxY) return +1;  // up
    return 0;                  // neutral
}

