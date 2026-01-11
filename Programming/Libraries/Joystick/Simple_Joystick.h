/*
 * Library: Simple_Joystick
 * File: Simple_Joystick.h
 * Author: Oscar Gadiel Ramo Martínez
 * 
 * Description:
 *   2-axis analog joystick with button interface.
 *   Provides X/Y axis reading with automatic calibration and button state.
 *   Supports configurable timing for non-blocking operation.
 *   
 *   Features:
 *     - Dual-axis ADC reading (X, Y)
 *     - Digital button input
 *     - Auto-calibration routine
 *     - Configurable sampling rate
 *     - Active-low/high button support
 * 
 * Usage:
 *   Joystick2D joystick;
 *   joystick.setup(pinX, pinY, pinBtn);
 *   joystick.calibrate();  // Optional calibration
 *   int x = joystick.readX();
 *   int y = joystick.readY();
 *   bool pressed = joystick.isButtonPressed();
 * 
 * Date: November - December 2025
 * Course: 5th Semester Challenge - ITESM
 * License: MIT
 */

#ifndef _JOYSTICK_H_
#define _JOYSTICK_H_

#include "SimpleADC.h"
#include "SimpleGPIO.h"
#include <esp_timer.h>
#include <esp_task_wdt.h>

// Default pins (change if you want)
#define JOY_X_PIN            GPIO_NUM_15
#define JOY_Y_PIN            GPIO_NUM_2
#define JOY_BTN_PIN          GPIO_NUM_4

// Default timing (microseconds)
#define JOY_DELTA_TIME_US    5000
#define JOY_CAL_TIME_US      50000

class Joystick2D
{
public:
    // Public ranges and thresholds (no const)
    int minX, maxX, minY, maxY;
    int neutralMin, neutralMax;   // Optional deadzone (not required by your original code)
    int button_active_low;        // 1 = active-low (your example), 0 = active-high

    // Timing (microseconds)
    unsigned int delta_time_us;
    unsigned int cal_time_us;

    Joystick2D();

    // Setup pins and defaults
    void setup(int pinX, int pinY, int pinBtn);

    // Start a new calibration window
    void startCalibration();

    // Run calibration step; returns 1 when finished, 0 while still calibrating
    int calibrationStep();

    // Raw reads
    int readXRaw();
    int readYRaw();

    // Button read with active-low support
    int isPressed();

    // Direction from calibrated extremes (same logic as your example):
    // X: -1 = left (< minX), 0 = neutral, +1 = right (> maxX)
    // Y: -1 = down  (< minY), 0 = neutral, +1 = up    (> maxY)
    int dirX();
    int dirY();

private:
    SimpleADC _x;
    SimpleADC _y;
    SimpleGPIO _btn;

    uint64_t _last_sample_us;
    uint64_t _cal_start_us;
    int _pinX, _pinY, _pinBtn;
};

#endif // _JOYSTICK_H_
