/*
 * Project: Automated Soap Viscosity Control System
 * File: SpindleVisc_tests.cpp
 * Author: Oscar Gadiel Ramo Martínez
 * 
 * Description:
 *   Unit tests for SpindleVisc DC motor control library.
 *   Tests include:
 *     - PWM open-loop control (duty cycle testing)
 *     - RPM closed-loop control (PID speed regulation)
 *     - Viscosity measurement at constant speed
 *     - Encoder feedback validation
 *   
 *   Each test is commented out for selective compilation.
 *   Uncomment the desired test case to validate specific functionality.
 * 
 * Hardware:
 *   - DC motor with H-bridge driver
 *   - Quadrature encoder for speed feedback
 *   - Current sensor (ADC) for viscosity measurement
 * 
 * Date: November - December 2025
 * Course: 5th Semester Challenge - ITESM
 * License: MIT
 */

#include "definitions.h"

/*
// Test RPMs
extern "C" void app_main()
{
    esp_task_wdt_deinit();

    //setup
    spindle.setup(spindle_config);
    spindle.setupPID(gains, spindle_dt_s);


    // Time polling
    uint64_t prev_time = esp_timer_get_time();
    int print_dt_us = 100000; // 10 ms
    while(true) 
    {
        uint64_t current_time = esp_timer_get_time();
        if (current_time - prev_time >= print_dt_us) 
        {
            prev_time = current_time;

            error = spindle.setSpeed(reference);
            angle = spindle.getAngle();
            speed = spindle.getSpeed();
            RPM = (speed * 60.0f) / 360.0f;

            message_length = sprintf(buffer, "Angle: %8.2f deg | Speed: %6.1f deg/s | RPM: %6.1f |  Error: %6.2f | Reference: %6.2f | Count: %lld\n", angle, speed, RPM, error, reference, (int64_t)(angle / degrees_per_edge));
            uart.write(buffer, message_length);
            int len = uart.available();
            if (len)
            {
                uart.read(buffer, len);
                sscanf(buffer, "%f\n", &reference);
            }
        }
    }
}
*/

/*
// Test PWM
extern "C" void app_main()
{
    esp_task_wdt_deinit();

    //setup
     spindle.setup(spindle_config);
    spindle.setupPID(gains, spindle_dt_s);


    // Time polling
    uint64_t prev_time = esp_timer_get_time();
    int print_dt_us = 100000; // 10 ms
    while(true) 
    {
        uint64_t current_time = esp_timer_get_time();
        if (current_time - prev_time >= print_dt_us) 
        {
            prev_time = current_time;
            spindle.setPWM(reference);
            float angle = spindle.getAngle();
            float speed = spindle.getSpeed();

            message_length = sprintf(buffer, "Angle: %8.2f deg | Speed: %6.1f deg/s | Count: %lld\n", angle, speed, (int64_t)(angle / degrees_per_edge));
            uart.write(buffer, message_length);
            int len = uart.available();
            if (len)
            {
                uart.read(buffer, len);
                sscanf(buffer, "%f\n", &reference);
            }
        }
    }
}
*/

/*
// Test PWM and RPMs for Spindle DC Motor 
extern "C" void app_main() 
{
    esp_task_wdt_deinit();

    // Setup
    spindle.setup(dcMotor_pins, dcMotor_channels, &dcMotor_vel_config,
                encoder_pins, degrees_per_edge,
                encoder_timeout_us,
                VISCOSITY_ADC_PIN,
                max_voltage, min_voltage, max_water_percentage, min_water_percentage);
    
    spindle.setupPID(gains, spindle_dt_s);

    printf("=== SPINDLE PWM TEST ===\n");
    printf("Starting at 0%% PWM\n\n");

    uint64_t prev_time = esp_timer_get_time();
    state = 0; // Start at state 0
    spindle.setPWM(0.0f); // Initial PWM

    while(true) 
    {
        uint64_t current_time = esp_timer_get_time();
        
        // Change state and PWM every 5 seconds
        if (current_time - prev_time >= 5000000) // 5 seconds 
        {
            state = (state + 1) % 3; // Cycle through states 0, 1, 2
            prev_time = current_time;
            
            // Update PWM based on new state
            
            switch(state) 
            {
                case 0:
                    printf("\n=== State 0: PWM = 0%% ===\n");
                    spindle.setPWM(0.0f);
                    break;
                case 1:
                    printf("\n=== State 1: PWM = 50%% ===\n");
                    spindle.setPWM(50.0f);
                    break;
                case 2:
                    printf("\n=== State 2: PWM = 100%% ===\n");
                    spindle.setPWM(-50.0f);
                    break;
            }
            

            // Update RPMs
            switch(state) 
            {
                case 0:
                    printf("\n=== State 0: RPM = 0 ===\n");
                    spindle.setSpeed(0.0f);
                    break;
                case 1:
                    printf("\n=== State 1: RPM = 160 ===\n");
                    spindle.setSpeed(160.0f);
                    break;
                case 2:
                    printf("\n=== State 2: RPM = 80 ===\n");
                    spindle.setSpeed(0.0f);
                    break;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10)); // Small delay to prevent CPU overload
    }
}
*/

/*
// Test - Measure average viscosity
extern "C" void app_main() 
{
    // Variables
    float average_viscosity = 0.0f;

    // Stop watchdog
    esp_task_wdt_deinit();

    // setup
    spindle.setup(spindle_config);
    spindle.setupPID(gains, spindle_dt_s);

    // Time polling
    uint64_t prev_time = esp_timer_get_time();
    int print_dt_us = 10000; // 1 ms

    // Prepare for viscosity measurement
    printf("=== SPINDLE VISCOSITY MEASUREMENT TEST ===\n");
    spindle.setSpeed(30.0f);
    vTaskDelay(10000 / portTICK_PERIOD_MS); // Wait 10 second for stabilization

    while(true)
    {
        uint64_t current_time = esp_timer_get_time();
        if (current_time - prev_time >= print_dt_us) 
        {
            prev_time = current_time;
            spindle.setSpeed(30.0f);
            printf("Measuring viscosity at 30 RPM...\n");

            average_viscosity = spindle.measure_viscosity_readings();  
            printf("Average Viscosity: %.2f cP\n", average_viscosity);
        };
    }       
}
*/