/*
 * Library: PistonDC
 * File: PistonDC.h
 * Author: Oscar Gadiel Ramo Martínez
 * 
 * Description:
 *   Pneumatic/DC piston controller with direction control.
 *   Uses two GPIO pins for up/down movement control via relay or driver.
 *   Tracks movement timing for position estimation.
 *   
 *   Features:
 *     - Up/Down/Stop control
 *     - Homing routine
 *     - Movement timing tracking
 *     - Simple 2-pin interface
 * 
 * Usage:
 *   PistonDC piston;
 *   piston.setup(gpio_pins, modes);
 *   piston.moveUp();
 *   piston.moveDown();
 *   piston.stop();
 *   piston.home();
 * 
 * Date: November - December 2025
 * Course: 5th Semester Challenge - ITESM
 * License: MIT
 */

#ifndef _PISTONDC_H
#define _PISTONDC_H

#include "SimpleGPIO.h"

class PistonDC
{
public:
    PistonDC();
    ~PistonDC();
    void setup(uint64_t gpio_num[], gpio_mode_t gpio_mode[]);
    void moveUp();
    void moveDown();
    void stop();
    void home();

private:
    void updatePosition();
    float _initial_distance;
    uint64_t _move_start_time;
    enum Direction { UP, DOWN, STOP };
    Direction _direction = STOP;
    SimpleGPIO gpio[2];
    
};

#endif // _PISTONDC_H