/*
 * Project: Automated Soap Viscosity Control System
 * File: stepperMotor_tests.cpp
 * Author: Oscar Gadiel Ramo Martínez
 * 
 * Description:
 *   Unit tests for stepperMotor control library (TB6600 driver).
 *   Tests include:
 *     - Position control to specific angles (0°, 90°, 180°, 270°)
 *     - Frequency control (speed testing at 50 Hz, 100 Hz)
 *     - PID closed-loop position control
 *     - Direction control (CW/CCW)
 *     - Homing routine validation
 *   
 *   Each test is commented out for selective compilation.
 *   Uncomment the desired test case to validate specific functionality.
 * 
 * Hardware:
 *   - Stepper motor (200 steps/rev typical)
 *   - TB6600 stepper driver
 *   - Optional: feedback sensor for closed-loop control
 * 
 * Date: November - December 2025
 * Course: 5th Semester Challenge - ITESM
 * License: MIT
 */

#include "definitions.h"

/*
// Move stepper to specific angles test
extern "C" void app_main() 
{
    esp_task_wdt_deinit();

    // SETUP
    stepper_motor.setup(stepper_pins, 1200, 800); // 1200 steps/rev, 800μs per step
    stepper_motor.setEnable(0); // Enable stepper, 0 = enabled for TB6600

    printf("Homing stepper to 0°...\n");
    stepper_motor.homePosition();
    printf("Current angle: %.2f°\n\n", stepper_motor.readAngle());

    uint64_t prev_time = esp_timer_get_time();
    state = 0; // Start at state 0

    while(true) 
    {
        uint64_t current_time = esp_timer_get_time();
        
        // Change state every 2 seconds
        if (current_time - prev_time >= 5000000) // 5 seconds
        {
            state = (state + 1) % 4; // Cycle through states 0, 1, 2, 3
            prev_time = current_time;
            
            // Execute movement based on state
            switch(state) 
            {
                case 0:
                    printf("\n=== State 0: Moving to 0° ===\n");
                    stepper_motor.desiredAngle(0.0);
                    break;
                case 1:
                    printf("\n=== State 1: Moving to 90° ===\n");
                    stepper_motor.desiredAngle(90.0);
                    break;
                case 2:
                    printf("\n=== State 2: Moving to 180° ===\n");
                    stepper_motor.desiredAngle(180.0);
                    break;
                case 3:
                    printf("\n=== State 3: Moving to 270° ===\n");
                    stepper_motor.desiredAngle(270.0);
                    break;
            }
        }
        
        // Print angle periodically while moving
        static uint64_t last_print = 0;
        if (current_time - last_print >= 500000) // Print every 500ms
        {
            if (stepper_motor.isMoving()) 
            {
                printf("  Moving... Current angle: %.2f°\n", stepper_motor.readAngle());
            }
            else 
            {
                printf("  Stopped at: %.2f°\n", stepper_motor.readAngle());
            }
            last_print = current_time;
        }
        
        vTaskDelay(pdMS_TO_TICKS(10)); // Small delay to prevent CPU overload
    }
}
*/

/*
// Stepper Motor Test - setFrequency, stop, readFrequency, readAngle
extern "C" void app_main() 
{
    esp_task_wdt_deinit();

    // setup
    stepper.setup(steppper_gpio_pins, stepper_pul_pin, stepper_pwm_channel, &stepper_vel_config,
                   stepper_duty_percentage, stepper_frequency, pulses_per_rev_stepper, 0);

    printf("\n=== STEPPER MOTOR TEST: setFrequency + stop ===\n");
    printf("Case 1: 50 Hz for 5 seconds\n");
    printf("Case 2: 100 Hz for 5 seconds\n");
    printf("Case 3: STOP for 5 seconds\n\n");

    uint64_t prev_time = esp_timer_get_time();
    uint64_t test_start_time = esp_timer_get_time();
    int print_dt_us = 200000; // 200 ms updates
    
    int case_num = 1;
    int case_duration_s = 5;

    while(true) 
    {
        uint64_t current_time = esp_timer_get_time();
        float elapsed_total = (current_time - test_start_time) / 1e6;  // Total elapsed
        float elapsed_case = fmod(elapsed_total, case_duration_s);     // Elapsed in current case
        
        // Determine which case (0-4 second = case 1, 5-9 = case 2, 10-14 = case 3, repeat)
        case_num = (int)(elapsed_total / case_duration_s) % 3 + 1;
        
        // Apply command based on case
        switch(case_num) {
            case 1:
                stepper.setFrequency(50);
                break;
            case 2:
                stepper.setFrequency(100);
                break;
            case 3:
                stepper.stop();  // STOP
                break;
        }
        
        // Print every 200ms
        if (current_time - prev_time >= print_dt_us) 
        {
            prev_time = current_time;
            
            printf("Case %d (%.1f/%.0fs) | Freq: %lu Hz | Angle: %.2f deg\n",
                   case_num,
                   elapsed_case,
                   (float)case_duration_s,
                   stepper.readFrequency(),
                   stepper.readAngle());
        }
    }
}
*/

/*
// Stepper motor test - test setupPID, desiredAngle, setDirection, readAngle, readFrequency
extern "C" void app_main()
{
    esp_task_wdt_deinit();

    //setup
    stepper.setup(steppper_gpio_pins, stepper_pul_pin, stepper_pul_feedback_pin, stepper_pwm_channel, 
                   &stepper_vel_config, stepper_duty_percentage, stepper_frequency, pulses_per_rev_stepper, 0);

    stepper.setupPID(gains_stepper, spindle_dt_s);


    // Time polling
    uint64_t prev_time = esp_timer_get_time();
    int print_dt_us = 20000; // 20 ms
    while(true) 
    {
        uint64_t current_time = esp_timer_get_time();
        if (current_time - prev_time >= print_dt_us) 
        {
            prev_time = current_time;

            switch (stepper_case) 
            {
                case 0:
                    error = stepper.desiredAngle(reference);
                    break;
                case 1:
                    stepper.setDirection(reference);
                    break;
                case 2:
                    stepper.setFrequency((uint32_t)reference);
                    break;
                case 3:
                    stepper.stop();
                    break;
            }
            
            angle = stepper.readAngle();
            speed = stepper.readSpeed();
            float frequency = stepper.readFrequency();

            message_length = sprintf(buffer, "Angle: %8.2f deg | Speed: %6.1f deg/s | Frequency: %6.1f Hz | Error: %6.2f | Reference: %6.2f | Count: %lld\n", angle, speed, frequency, error, reference, (int64_t)(angle / (360.0f / pulses_per_rev_stepper)));
            uart.write(buffer, message_length);
            int len = uart.available();
            if (len)
            {
                uart.read(buffer, len);
                sscanf(buffer, "%f,%d\n", &reference, &stepper_case);
            }
        }
    }
}
*/