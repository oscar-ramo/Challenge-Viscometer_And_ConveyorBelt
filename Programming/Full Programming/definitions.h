/*
 * Project: Automated Soap Viscosity Control System
 * File: definitions.h
 * Author: Oscar Gadiel Ramo Martínez
 * 
 * Description:
 *   Global definitions, enums, configuration structs, and hardware instances
 *   for the viscosity control system. Includes state machine definitions for
 *   both manual and automatic operation modes, as well as dosing state machine.
 * 
 * Portfolio Note:
 *   Added DosingState enum and DosingResult enum for non-blocking water dosing
 *   implementation (refactored January 2026 for portfolio).
 * 
 * Date: November 2025 - January 2026
 * Course: 5th Semester Challenge - ITESM
 */

#ifndef _DEFINITIONS_H_
#define _DEFINITIONS_H_

// ============================================================================
// SYSTEM INCLUDES
// ============================================================================
#include <esp_timer.h>
#include <esp_task_wdt.h>
#include <math.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstring>
#include <cstdio>

// ============================================================================
// LIBRARY INCLUDES
// ============================================================================
#include "SimpleADC.h"
#include "SimpleGPIO.h"
#include "SimplePWM.h"
#include "SimpleRGB.h"
#include "PistonDC.h"
#include "dcMotorController.h"
#include "stepperMotor.h"
#include "esp_system.h"
#include "SpindleVisc.h"
#include "SimpleUART.h"
#include "AxelF_CrazyFrog.h"
#include "HBridge.h"
#include "SimpleI2C.h"
#include "Ultrasonic.h"
#include "RGBColorSensorTCS34725.h"
#include "SimpleSerialBT.h"

// ============================================================================
// ENUM & LABVIEW INPUT/OUTPUT DEFINITIONS
// ============================================================================

// Machine states of the manual control system
enum ManualState
{
    MANUAL_IDLE = 0,
    PISTON_UP = 1, // Button
    PISTON_DOWN = 2, // Button
    STEPPER_CW = 3, // Button
    STEPPER_CCW = 4, // Button
    STEPPER_SPECIFIC = 5, // UART
    STEPPER_FREQ = 6,
    SPINDLE_PWM = 7, // Knob
    SPINDLE_RPMs = 8, // Knob
    SPINDLE_MEASURE_VISCOSITY = 9, // Button
    PUMP_WATER = 10, // Button
    DRYER_ON = 11, // Button
}; 

// Switch case state variable 
ManualState manual_case_state = MANUAL_IDLE;
ManualState previous_state = MANUAL_IDLE; // Track previous state for state transitions

// Machine states of the automatic control system
enum AutomaticState 
{
    HOMING_ROUTINE = 0,
    AUTOMATIC_IDLE = 1,
    CONTAINER_POSITIONING = 2,
    DETECT_TAG_COLOR = 3,
    MEASURE_VISCOSITY = 4,
    DOSSING_ADJUSTMENT = 5,
    CLEANING_STATION = 6,
    DRYING_STATION = 7
};

// Switch case state variable
AutomaticState automatic_case_state = AUTOMATIC_IDLE;

// Water dosing state machine
enum DosingState 
{
    DOSING_IDLE = 0,
    DOSING_CALC = 1,
    DOSING_PUMP = 2,
    DOSING_STIR = 3,
    DOSING_MEASURE = 4,
    DOSING_EVALUATE = 5,
    DOSING_COMPLETE = 6
};

// Dosing state variables (persistent across calls)
DosingState dosing_state = DOSING_IDLE;
uint64_t dosing_start_time = 0;
float dosing_target_viscosity = 0.0f;
float dosing_pump_time_s = 0.0f;
int dosing_current_iteration = 0;
float dosing_current_dose_percentage = 0.0f;

// Dosing result enum for clear return values
enum DosingResult 
{
    DOSING_IN_PROGRESS = 0,  // Still working, call again
    DOSING_SUCCESS = 1,       // Quality passed
    DOSING_CONTINUE = 2,      // Need next iteration
    DOSING_FAIL_OVERSHOOT = 3,  // Too much water
    DOSING_FAIL_ITERATIONS = 4  // Max iterations reached
};

// LabVIEW inputs
int btn_piston_up = 0;
int btn_piston_down = 0;
int btn_stepper_cw = 0;
int btn_stepper_ccw = 0;
float knob_stepper_angle = 0.0f;
float knob_stepper_frequency = 0.0f;
float knob_spindle_pwm = 0.0f;
float knob_spindle_rpm = 0.0f;
int btn_measure_viscosity = 0;
int btn_water_pump = 0;
int btn_dryer_on = 0;
int state_machine_case = 0;

// LabVIEW outputs
float piston_position = 0.0f;
float stepper_angle = 0.0f;
float stepper_speed = 0.0f;
float spindle_rpms = 0.0f;
float spindle_current = 0.0f;
float viscosity_avrg = 0.0f;
uint16_t r_sensor_output = 0, g_sensor_output = 0, b_sensor_output = 0, c_sensor_output = 0;
int message_box_case = 0;

// ============================================================================
// PIN & CLASSES DEFINITIONS
// ============================================================================

// UART
SimpleUART uart(115200);

// BT
SerialBT bt;

// ================================ ACTUATORS =================================

// DC Motor Timer Configuration (shared by Spindle, Water Pump, Dryer Motor)
TimerConfig dcMotorTimerConfig = 
{
    .timer = LEDC_TIMER_0,
    .bit_resolution = LEDC_TIMER_8_BIT,
    .mode = LEDC_HIGH_SPEED_MODE
};

// Spindle DC motor
uint8_t dcMotor_pins[2] = {32, 33}; // IN1, IN2
// Spindle uses DC motor timer 0 (high speed) and channels 0,1
uint8_t dcMotor_channels[2] = {0, 1};

uint8_t encoder_pins[2] = {2, 13}; // ENC_A (Amarillo), ENC_B (Verde)

float pulses_per_rev = 291.0f; // datasheet = 341.2f, real = 291.0f

float degrees_per_edge = 360.0f / (pulses_per_rev * 2);

int64_t encoder_timeout_us = 100000; // 100 ms

#define VISCOSITY_ADC_PIN 36 // C-V - Changed from pin 2 to avoid conflicts
float max_voltage = 1900.0f; // mV at 100% soap
float min_voltage = 33.0f;    // mV at 0% soap
float max_water_percentage = 25.0f;
float min_water_percentage = 0.0f;

float gains[3] = {0.1156f, 43.2543f, 0.0f}; // PID gains: Kp, Ki, Kd. DEFINE WITH TF
float spindle_dt_s = 0.001f; // dcMotorController internal timing (NOT main loop dt)

float m_slope = 0.0542f;    // Feedforward slope (deg/s to PWM)
float b_intercept = 14.289f; // Feedforward intercept (PWM at 0 deg/s)

float m_conditioning = 1.0f; // Slope for current conditioning
float b_conditioning = 2.51f; // Intercept for current conditioning

// Spindle configuration struct - groups all related parameters
SpindleConfig spindle_config = {
    .hbridge_pins = dcMotor_pins,
    .hbridge_channels = dcMotor_channels,
    .motor_config = &dcMotorTimerConfig,
    .encoder_pins = encoder_pins,
    .degrees_per_edge = degrees_per_edge,
    .encoder_timeout_us = encoder_timeout_us,
    .viscosity_adc_pin = VISCOSITY_ADC_PIN,
    .max_voltage = max_voltage,
    .min_voltage = min_voltage,
    .max_water_percentage = max_water_percentage,
    .min_water_percentage = min_water_percentage,
    .m_slope = m_slope,
    .b_intercept = b_intercept,
    .m_conditioning = m_conditioning,
    .b_conditioning = b_conditioning
};

SpindleVisc spindle;

// Stepper Motor - Controls the X-movement
uint8_t steppper_gpio_pins[2] = {26, 27}; // DIR, ENA
uint8_t stepper_pul_pin = 25; // PUL
uint8_t stepper_pul_feedback_pin = 35; // Pulse feedback from TB6600 (GPIO 35 - ADC input)
uint8_t stepper_pwm_channel = 6; // Changed from 2 to 6 to avoid conflict with water pump
float stepper_duty_percentage = 50.0f; // 50% duty cycle for square wave

TimerConfig stepper_vel_config = 
{
    .timer = LEDC_TIMER_1,
    .frequency = 5000,  // Higher base frequency for better PWM control
    .bit_resolution = LEDC_TIMER_10_BIT,  // Reduced from 12-bit to prevent duty errors
    .mode = LEDC_HIGH_SPEED_MODE
};

uint32_t stepper_frequency = 100; // Initial frequency

// Calculated from test: 1600 gave 270° for 180° cmd, so 1600 × (180/270) = 1067
uint32_t pulses_per_rev_stepper = 800; // Empirically calibrated

float gains_stepper[3] = {1.0f, 0.0f, 0.0f}; // PID gains: Kp, Ki, Kd. Increased Kp for faster response

int stepper_case = 0; // test

StepperMotor stepper;

// H-Bridge - Y-axis piston control
uint64_t PistonPins[2] = {14, 12}; // IN1, IN2
gpio_mode_t PistonModes[2] = {GPIO_MODE_OUTPUT, GPIO_MODE_OUTPUT};

PistonDC piston;

// Water Pump
uint8_t water_pump_pins[2] = {19, 255};
// Water pump shares DC motor timer 0 (high speed), uses channels 2,3
uint8_t water_channels[2] = {2, 3};

HBridge water_pump;

// Drying motor
uint8_t dryer_pins[2] = {23, 255};
// Dryer motor shares DC motor timer 0 (high speed), uses channels 4,5
uint8_t dryer_channels[2] = {4, 5};

HBridge dryer_motor;

// RGB LED Configuration
uint8_t RGB_pins[3] = {18, 17, 16};
// RGB has dedicated timer 2 low speed, channels 0,1,2 (three channels)
uint8_t RGB_channels[3] = {0, 1, 2};
TimerConfig RGB_config = 
{
    .timer = LEDC_TIMER_2,
    .frequency = 40,
    .bit_resolution = LEDC_TIMER_12_BIT,
    .mode = LEDC_LOW_SPEED_MODE
};

SimpleRGB RGB_led;

// ================================ SENSORS ================================

// Ultrasonic Sensor
#define TRIG_PIN 4
#define ECHO_PIN 34
// Ultrasonic has dedicated timer 3 low speed, channel 0
uint8_t ultrasonic_trig_channel = 0; 
TimerConfig ultrasonic_timer_config = 
{
    .timer = LEDC_TIMER_3,
    .frequency = 40,
    .bit_resolution = LEDC_TIMER_12_BIT,
    .mode = LEDC_LOW_SPEED_MODE
};

Ultrasonic ultrasonic;

// Start routine IR Sensor
#define START_IR_SENSOR_PIN 39

SimpleGPIO start_routine_sensor;

// End routine IR Sensor
#define IR_SENSOR_PIN 5

SimpleGPIO end_routine_sensor;

// Homing IR Sensor
#define HOMING_IR_SENSOR_PIN 1

SimpleGPIO homing_ir_sensor;

// Dispose Sensor
#define DISPOSE_IR_SENSOR_PIN 3  // GPIO 3 (RX0, safe if not using Serial for debugging) 

SimpleGPIO dispose_sensor;

// Color Sensor
uint8_t color_sensor_pins[2] = {21, 22}; // SDA, SCL

TCS3475_RGB color_sensor;

// Communication
// IR para cuando acabe el proceso
// GPIO para el arduino en caso de que uno no jale

// ================================ OTHERS ================================

// Axel F - Buzzer Class
#define BUZZER_PIN 15
// Buzzer shares timer 3 low speed with ultrasonic, channel 1
uint8_t buzzer_channel = 1;
TimerConfig buzzer_timer_config = 
{
    .timer = LEDC_TIMER_3,
    .frequency = 440,
    .bit_resolution = LEDC_TIMER_8_BIT,
    .mode = LEDC_LOW_SPEED_MODE
};

AxelF_CrazyFrog crazy_frog;  // Renamed from 'song' to avoid conflict


// ============================================================================
// SYSTEM VARIABLES & TIMING
// ============================================================================

// UART communication variables
char buffer[256];
char msg_buffer[256];
uint64_t message_length;
const char* txt_msg;
int len = 0;

// Time polling variables
uint64_t prev_time, current_time;
uint64_t dt_us = 100000; // 100ms = 10Hz update rate (matches working test code)

// Viscosity time polling variables
uint64_t visc_prev_time, visc_current_time;
uint64_t visc_dt_us = 5000000; // 5 s

// RGB LED colors
int r = 0, g = 0, b = 0;

// Automatic Control System Variables
float piston_target_height_cm = 0.0f;
int start_automation = 0;
float min_viscosity = 0.0f;
float max_viscosity = 0.0f;

// RPMs test
float speed = 0.0f, angle = 0.0f, RPM = 0.0f, error = 0.0f, reference = 0.0f;

#endif //_DEFINITIONS_H_