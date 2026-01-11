/*
 * Project: Automated Soap Viscosity Control System
 * File: Integration_tests.cpp
 * Author: Oscar Gadiel Ramo Martínez
 * 
 * Description:
 *   Integration tests for multi-subsystem operation.
 *   Tests include:
 *     - Joystick-controlled piston and stepper movement (XY control)
 *     - Full system integration with LabVIEW communication
 *     - Manual control state machine testing
 *     - RGB LED feedback and LCD display integration
 *     - UART data exchange validation
 *   
 *   These tests validate that multiple hardware subsystems work
 *   together correctly before implementing the final automatic control.
 *   
 *   Each test is commented out for selective compilation.
 *   Uncomment the desired test case to validate specific functionality.
 * 
 * Hardware:
 *   - Piston with position feedback
 *   - Stepper motor for rotational positioning
 *   - Joystick for manual XY control
 *   - RGB LED, LCD display, buzzer
 *   - UART communication with LabVIEW
 * 
 * Date: November - December 2025
 * Course: 5th Semester Challenge - ITESM
 * License: MIT
 */

#include "definitions.h"

/*
// Move piston and stepper with joystick
extern "C" void app_main() 
{
    esp_task_wdt_deinit();

    // SETUP
    // Joystick
    joystick_X.setup(JOYSTICK_X_PIN);
    joystick_Y.setup(JOYSTICK_Y_PIN);
    joystick_press.setup(JOYSTICK_PRESS_PIN, GPIO_MODE_INPUT);
    // Piston Y-axis control
    piston_control.setup(PistonPins, PistonModes); // 5 mm/s, 5 cm distance
    // Stepper X-axis control
    stepper_motor.setup(stepper_pins, 200, 10000); // 200 steps/rev, 10000μs per step = slower speed
    stepper_motor.setEnable(0); // Enable stepper, 0 = enabled for TB6600

    uint64_t prev_time = esp_timer_get_time();
    int prev_stepper_direction = 0; // 0 = stopped, 1 = CW, -1 = CCW

    printf("Homing piston...\n");
    piston_control.home(); // Home at start

    // MAIN LOOP
    while(true) 
    {
        uint64_t current_time = esp_timer_get_time();
        if (current_time - prev_time >= 1000) // 1 ms
        {
            int y_value = joystick_Y.read(ADC_READ_RAW);
            int x_value = joystick_X.read(ADC_READ_RAW);

            // Piston control
            if (y_value < down_y)
            {
                // Move down
                printf("DOWN\n");
                piston_control.moveDown();
            }
            else if (y_value > up_y)
            {
                // Move up
                printf("UP\n");
                piston_control.moveUp();
            }
            else
            {
                // Stop
                piston_control.stop();
            }

            // Stepper control - only start timer when direction changes
            if (x_value < left_x)
            {
                // Move left (CCW)
                if (prev_stepper_direction != -1) {
                    printf("LEFT - Starting CCW. Angle: %.2f°\n", stepper_motor.readAngle());
                    stepper_motor.moveCCW();
                    prev_stepper_direction = -1;
                }
            }
            else if (x_value > right_x)
            {
                // Move right (CW)
                if (prev_stepper_direction != 1) {
                    printf("RIGHT - Starting CW. Angle: %.2f°\n", stepper_motor.readAngle());
                    stepper_motor.moveCW();
                    prev_stepper_direction = 1;
                }
            }
            else
            {
                // Neutral position - stop if moving
                if (prev_stepper_direction != 0) {
                    stepper_motor.stop();
                    printf("STEP STOP. Angle: %.2f°\n", stepper_motor.readAngle());
                    prev_stepper_direction = 0;
                }
            }
            prev_time = current_time;
        }
    }

}
*/

/*
// ============================================================================
// MAIN APPLICATION
// ============================================================================

extern "C" void app_main() 
{
    esp_task_wdt_deinit(); // Disable the watchdog timer

    // ========== INIT SETUP & VARIABLES ==========

    // Spindle DC motor
    spindle.setup(spindle_config);

    // Stepper X-axis control
    stepper_motor.setup(stepper_pins, 200, 10000); // 200 steps/rev, 10000μs per step = slower speed
    stepper_motor.setEnable(0); // Enable stepper, 0 = enabled for TB6600

    // Piston Y-axis control
    piston_control.setup(PistonPins, PistonModes); // 5 mm/s, 5 cm distance

    // Water Pump - Pending
    Water_pump.setup(WaterPump_pins, WaterPump_gpio_mode, WaterPump_vel_channel, &WaterPump_vel_config);

    // LED RGB
    RGB_led.setup(RGB_pins, RGB_channels, &RGB_config, 1);
    RGB_led.setColor(0, 0, 0); // Start with LED off
    r = 0; g = 0; b = 0;

    // LCD Display
    i2c_lcd.setup(LCD_I2C_ADDR, 400000, LCD_SDA, LCD_SCL); // 400 kHz
    lcd.setup(LCD_I2C_ADDR);
    
    // Display welcome message
    lcd.display_lcd_now("Viscosity System Initializing...");

    // Buzzer - Pending

    // Viscosity sensor
    viscosity_reading.setup(VISCOSITY_ADC_PIN);

    // ========== BEGIN MAIN LOOP ==========

    prev_time = esp_timer_get_time(); // Initiate time polling

    while(true) 
    {
        current_time = esp_timer_get_time();
        if (current_time - prev_time >= dt_us) 
        {
            prev_time = current_time;

            // Debug: print current state
            static int last_printed_state = -1;
            if (state != last_printed_state) {
                printf("Current State: %d\n", state);
                last_printed_state = state;
            }

            switch(state) 
            {
                case IDLE:
                    // No operation, system idle
                    r = 0; g = 255; b = 0; // Green
                    RGB_led.setColor(r, g, b);
                    lcd.display_lcd_now("System Ready");
                    break;
                    
                case PISTON_UP:
                    r = 255; g = 255; b = 0; // Yellow
                    RGB_led.blinkColor(r, g, b, 500);
                    piston_control.moveUp();
                    piston_position = piston_control.readPosition();
                    lcd.display_lcd_now("Piston: UP");
                    break;
                    
                case PISTON_DOWN:
                    r = 255; g = 255; b = 0; // Yellow
                    RGB_led.blinkColor(r, g, b, 500);
                    piston_control.moveDown();
                    piston_position = piston_control.readPosition();
                    lcd.display_lcd_now("Piston: DOWN");
                    break;
                    
                case PISTON_STOP:
                    r = 0; g = 255; b = 0; // Green
                    RGB_led.setColor(r, g, b);
                    piston_control.stop();
                    piston_position = piston_control.readPosition();
                    snprintf(lcd_line1, sizeof(lcd_line1), "Piston STOP %.1f", piston_position);
                    lcd.display_lcd_now(lcd_line1);
                    break;
                    
                case WATER_PUMP:
                    r = 0; g = 0; b = 255; // Blue
                    RGB_led.blinkColor(r, g, b, 500);
                    Water_pump.direction(1);
                    Water_pump.duty_cycle(80.0f);
                    motor_current_mA = simulate_motor_current(80.0f);
                    lcd.display_lcd_now("Pump ON 80%");
                    break;
                    
                case WATER_PUMP_STOP:
                    r = 0; g = 255; b = 0; // Green
                    RGB_led.setColor(r, g, b);
                    Water_pump.duty_cycle(0.0f);
                    motor_current_mA = 0.0f;
                    lcd.display_lcd_now("Pump OFF");
                    break;
                    
                case STEPPER_CW:
                    r = 255; g = 0; b = 255; // Magenta
                    RGB_led.blinkColor(r, g, b, 500);
                    stepper_motor.moveCW();
                    stepper_position = stepper_motor.readAngle();
                    snprintf(lcd_line1, sizeof(lcd_line1), "Stepper CW %.0f", stepper_position);
                    lcd.display_lcd_now(lcd_line1);
                    break;
                    
                case STEPPER_CCW:
                    r = 255; g = 0; b = 255; // Magenta
                    RGB_led.blinkColor(r, g, b, 500);
                    stepper_motor.moveCCW();
                    stepper_position = stepper_motor.readAngle();
                    snprintf(lcd_line1, sizeof(lcd_line1), "Stepper CCW %.0f", stepper_position);
                    lcd.display_lcd_now(lcd_line1);
                    break;
                    
                case STEPPER_SPECIFIC:
                    r = 255; g = 0; b = 255; // Magenta
                    RGB_led.blinkColor(r, g, b, 500);
                    stepper_motor.desiredAngle(input_stepper_angle);
                    stepper_position = stepper_motor.readAngle();
                    snprintf(lcd_line1, sizeof(lcd_line1), "MoveTo %.0f deg", input_stepper_angle);
                    lcd.display_lcd_now(lcd_line1);
                    break;
                    
                case STEPPER_STOP:
                    r = 0; g = 255; b = 0; // Green
                    RGB_led.setColor(r, g, b);
                    stepper_motor.stop();
                    stepper_position = stepper_motor.readAngle();
                    snprintf(lcd_line1, sizeof(lcd_line1), "Stepper STOP %.0f", stepper_position);
                    lcd.display_lcd_now(lcd_line1);
                    break;
                    
                case SPINDLE_RPMs:
                    r = 0; g = 255; b = 255; // Cyan
                    RGB_led.blinkColor(r, g, b, 500);
                    current_rpms = obtain_desired_RPMs(int(knob_spindle_rpm));
                    motor_current_mA = simulate_motor_current(knob_spindle_rpm * 100.0f / 130.0f);
                    snprintf(lcd_line1, sizeof(lcd_line1), "Spindle %.0fRPM", current_rpms);
                    lcd.display_lcd_now(lcd_line1);
                    break;

                case MEASURE_VISCOSITY:
                    r = 255; g = 165; b = 0; // Orange
                    RGB_led.blinkColor(r, g, b, 500);
                    current_viscosity = measure_viscosity_readings();
                    snprintf(lcd_line1, sizeof(lcd_line1), "Meas %.1fcP", current_viscosity);
                    lcd.display_lcd_now(lcd_line1);
                    break;
            }
        }

        // Receive data from LabVIEW via UART
        len = uart.available();
        if (len) 
        {
            uart.read(buffer, len);
            // Format: knob_rpm,input_angle,btn_piston_up,btn_piston_down,
            //         btn_pump,btn_step_cw,btn_step_ccw,btn_visc
            sscanf(buffer, "%f,%f,%d,%d,%d,%d,%d,%d\n", 
                   &knob_spindle_rpm, &input_stepper_angle,
                   &btn_piston_up, &btn_piston_down,
                   &btn_water_pump,
                   &btn_stepper_cw, &btn_stepper_ccw,
                   &btn_measure_viscosity);
            
            // Debug: Print received data
            printf("UART RX: %.1f,%.1f,%d,%d,%d,%d,%d,%d\n",
                   knob_spindle_rpm, input_stepper_angle,
                   btn_piston_up, btn_piston_down, btn_water_pump,
                   btn_stepper_cw, btn_stepper_ccw, btn_measure_viscosity);
            
            // Determine state based on button presses (priority order)
            if (btn_piston_up) 
                state = PISTON_UP;
            else if (btn_piston_down) 
                state = PISTON_DOWN;
            else if (btn_water_pump) 
                state = WATER_PUMP;
            else if (btn_stepper_cw) 
                state = STEPPER_CW;
            else if (btn_stepper_ccw) 
                state = STEPPER_CCW;
            else if (input_stepper_angle != 0.0f) 
                state = STEPPER_SPECIFIC;
            else if (knob_spindle_rpm > 0.0f) 
                state = SPINDLE_RPMs;
            else if (btn_measure_viscosity) 
                state = MEASURE_VISCOSITY;
            else {
                // All buttons released - determine which device to stop
                // Stop piston if it was moving
                if (state == PISTON_UP || state == PISTON_DOWN)
                    state = PISTON_STOP;
                // Stop pump if it was running
                else if (state == WATER_PUMP)
                    state = WATER_PUMP_STOP;
                // Stop stepper if it was moving
                else if (state == STEPPER_CW || state == STEPPER_CCW || state == STEPPER_SPECIFIC)
                    state = STEPPER_STOP;
                // Otherwise idle
                else
                    state = IDLE;
            }
            
            printf("State changed to: %d\n", state);
        }
        
        // Send comprehensive data to LabVIEW:
        // Format: piston_pos,viscosity,motor_current,r,g,b,rpm,stepper_pos\n
        message_length = sprintf(buffer, "%.2f,%.2f,%.2f,%d,%d,%d,%.1f,%.1f\n", 
                                 piston_position, current_viscosity, motor_current_mA,
                                 r, g, b, current_rpms, stepper_position);
        uart.write(buffer, message_length);
    }
}
*/