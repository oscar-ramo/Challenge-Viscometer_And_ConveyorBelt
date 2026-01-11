/*
 * Library: AxelF_CrazyFrog
 * File: AxelF_CrazyFrog.h
 * Author: Oscar Gadiel Ramo Martínez
 * 
 * Description:
 *   Non-blocking music player for piezo buzzer using PWM.
 *   Plays the "Axel F" theme (Crazy Frog) note by note without blocking
 *   the main control loop. Uses time polling to advance through musical notes.
 *   
 *   Features:
 *     - Non-blocking playback (polled on each cycle)
 *     - Musical note definitions (C4-F5)
 *     - 120 BPM timing
 *     - Automatic looping
 *     - Start/stop control
 * 
 * Usage:
 *   AxelF_CrazyFrog buzzer;
 *   buzzer.setup(pin, channel, &timer_config);
 *   // In main loop:
 *   buzzer.playSong();  // Call repeatedly to advance music
 *   buzzer.stopSong();  // Stop playback
 * 
 * Date: November - December 2025
 * Course: 5th Semester Challenge - ITESM
 * License: MIT
 */

#ifndef __AXELF_CRAZYFROG_H__
#define __AXELF_CRAZYFROG_H__

// Libraries
#include "SimplePWM.h"
#include "SimpleGPIO.h"
#include <esp_task_wdt.h>

// Tones kHz
#define NO_TONE 10
#define TONE_C4  261
#define TONE_D4  294
#define TONE_D_SHARP_4  311
#define TONE_E4  330
#define TONE_F4  349
#define TONE_F_SHARP_4  370
#define TONE_G4  392
#define TONE_G_SHARP_4  415
#define TONE_A4  440
#define TONE_A_SHARP_4  466
#define TONE_B4  494
#define TONE_C5  523
#define TONE_C_SHARP_5  554
#define TONE_D5  587
#define TONE_E5  659
#define TONE_F5  698

// La Touche Musicale (120 BPM, 4/4)
// F4, G#4 dot, F4, F4, A#4, F4, D#4 | 
// F4, C5 dot, F4, F4, C#5, C5, G#4 | 
// F4, C5, F5, F4, D#4, D#4, C4, G4, F4 | 
// 0, 0, F4, F4

constexpr uint16_t BPM = 120;
constexpr uint16_t ms_duration = 15000 / BPM; // Duration of a quarter note in milliseconds
constexpr uint16_t ms_dot_duration = ms_duration + (ms_duration / 2); // Duration of a dotted quarter note
constexpr uint16_t ms_half_duration = ms_duration / 2; // Duration of a half note
constexpr uint16_t no_tone_duration = 75; // Duration of a rest note

struct Note 
{
    uint16_t frequency;
    uint16_t duration; // in milliseconds
};

static const Note song[] = 
{
    {TONE_F4, ms_duration},
    {NO_TONE, 200},
    {TONE_G_SHARP_4, ms_dot_duration},
    {NO_TONE, no_tone_duration},
    {TONE_F4, ms_duration},
    {NO_TONE, 25},
    {TONE_F4, ms_duration},
    {NO_TONE, no_tone_duration},
    {TONE_A_SHARP_4, ms_duration},
    {NO_TONE, no_tone_duration},
    {TONE_F4, ms_duration},
    {NO_TONE, no_tone_duration},
    {TONE_D_SHARP_4, ms_duration}, // Bar line
    {NO_TONE, no_tone_duration},
    {TONE_F4, ms_duration},
    {NO_TONE, 200},
    {TONE_C5, ms_dot_duration},
    {NO_TONE, no_tone_duration},
    {TONE_F4, ms_duration},
    {NO_TONE, 25},
    {TONE_F4, ms_duration},
    {NO_TONE, no_tone_duration},
    {TONE_C_SHARP_5, ms_duration},
    {NO_TONE, no_tone_duration},
    {TONE_C5, ms_duration},
    {NO_TONE, no_tone_duration},
    {TONE_G_SHARP_4, ms_duration}, // Bar line
    {NO_TONE, no_tone_duration},
    {TONE_F4, ms_duration},
    {NO_TONE, no_tone_duration},
    {TONE_C5, ms_duration},
    {NO_TONE, no_tone_duration},
    {TONE_F5, ms_duration},
    {NO_TONE, no_tone_duration},  
    {TONE_F4, ms_duration},
    {NO_TONE, no_tone_duration},
    {TONE_D_SHARP_4, ms_duration},
    {NO_TONE, 25},
    {TONE_D_SHARP_4, ms_duration},
    {NO_TONE, no_tone_duration},
    {TONE_C4, ms_duration},
    {NO_TONE, no_tone_duration},
    {TONE_G4, ms_duration},
    {NO_TONE, no_tone_duration},
    {TONE_F4, ms_duration}, // Bar line
    {NO_TONE, no_tone_duration},
    {TONE_F4, ms_duration},
    {NO_TONE, 1000},
    {TONE_F4, ms_duration},
    {NO_TONE, 1000}
};

class AxelF_CrazyFrog
{
    public:
        AxelF_CrazyFrog();
        ~AxelF_CrazyFrog();
        void setup(uint8_t pin, uint8_t channel, TimerConfig *timer_config);
        void playSong();
        void stopSong();
    private:
        SimplePWM buzzer;
        const int song_length = sizeof(song) / sizeof(song[0]); // Calculate number of notes in the song
        int current_note = 0; // Index of the current note being played
        uint64_t last_time = esp_timer_get_time();
};

#endif // __AXELF_CRAZYFROG_H__