/*
 * Library: PistonDC
 * File: PistonDC.cpp
 * Author: Oscar Gadiel Ramo Martínez
 * 
 * Description:
 *   Implementation of pneumatic/DC piston controller.
 *   Controls piston movement through GPIO-based relay/driver interface.
 * 
 * Date: November - December 2025
 * Course: 5th Semester Challenge - ITESM
 * License: MIT
 */

#include "PistonDC.h"

PistonDC::PistonDC()
{
}

PistonDC::~PistonDC()
{
}

void PistonDC::setup(uint64_t gpio_num[], gpio_mode_t gpio_mode[])
{
    for (size_t i = 0; i < 2; i++)
    {
        gpio[i].setup(gpio_num[i], gpio_mode[i]);
    }
}

void PistonDC::moveUp()
{
    gpio[0].set(0);
    gpio[1].set(1);
    _move_start_time = esp_timer_get_time();
}

void PistonDC::moveDown()
{
    gpio[0].set(1);
    gpio[1].set(0);
    _move_start_time = esp_timer_get_time();
}

void PistonDC::stop()
{
    gpio[0].set(0);
    gpio[1].set(0);
    _direction = STOP;
}

void PistonDC::home()
{
    // Retract piston fully (move down) for a known safe time
    // Assumes piston is physically stopped by mechanical limit
    moveDown();
    uint64_t start = esp_timer_get_time();
    while (esp_timer_get_time() - start < 15000000) // 5 seconds
    {
        // Polling delay - just wait
    }
    
    stop();
}