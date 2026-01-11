/*
 * Library: AxelF_CrazyFrog
 * File: AxelF_CrazyFrog.cpp
 * Author: Oscar Gadiel Ramo Martínez
 * 
 * Description:
 *   Implementation of non-blocking music player for "Axel F" theme.
 *   Manages note timing and PWM frequency generation for piezo buzzer.
 * 
 * Date: November - December 2025
 * Course: 5th Semester Challenge - ITESM
 * License: MIT
 */

#include "AxelF_CrazyFrog.h"

AxelF_CrazyFrog::AxelF_CrazyFrog()
{
}

AxelF_CrazyFrog::~AxelF_CrazyFrog()
{
}

void AxelF_CrazyFrog::setup(uint8_t pin, uint8_t channel, TimerConfig *timer_config)
{
    buzzer.setup(pin, channel, timer_config);
}

void AxelF_CrazyFrog::playSong()
{
    uint64_t current_time = esp_timer_get_time();
    
    // Check if we need to advance to the next note
    if (current_time - last_time >= song[current_note].duration * 1000) {
        current_note++;
        if (current_note >= song_length) {
            current_note = 0; // Loop the song
        }
        last_time = current_time; // Reset timer for the new note
    }

    // Apply output for the current note every call
    if (song[current_note].frequency == NO_TONE) {
        // Rest: ensure buzzer is silent for the duration
        buzzer.setDuty(0.0f);
    } else {
        // Tone: set frequency and drive the buzzer
        buzzer.setFrequency(song[current_note].frequency);
        buzzer.setDuty(50.0f);
    }
}

void AxelF_CrazyFrog::stopSong()
{
    // Silence buzzer
    buzzer.setDuty(0.0f);
    // Reset to beginning of song
    current_note = 0;
    // Reset timer so next playSong() starts fresh
    last_time = esp_timer_get_time();
}