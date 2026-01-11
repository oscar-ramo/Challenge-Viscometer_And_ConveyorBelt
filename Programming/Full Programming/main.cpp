/*
 * Project: Automated Soap Viscosity Control System
 * File: main.cpp
 * Author: Oscar Gadiel Ramo Martínez
 * 
 * Description:
 *   Implements dual-mode (manual/automatic) control system for soap production
 *   quality control with iterative water dosing to achieve target viscosity.
 *   
 *   Features:
 *     - Manual control: Direct hardware operation via LabVIEW interface
 *     - Automatic control: Full production cycle with quality assurance
 *     - Iterative PID-based viscosity adjustment (85%, 5%, 5% dosing)
 *     - Color-coded container detection for viscosity ranges
 *     - Bluetooth telemetry communication
 *     - Non-blocking state machine architecture for real-time operation
 * 
 * Hardware:
 *   - ESP32 (ESP-IDF framework)
 *   - DC Motor spindle with quadrature encoder
 *   - Stepper motor for rotational positioning
 *   - Pneumatic piston with ultrasonic position feedback
 *   - TCS34725 RGB color sensor
 *   - Water pump, dryer motor, RGB LED, buzzer
 * 
 * Portfolio Note:
 *   This version has been refactored for portfolio presentation with
 *   non-blocking implementations (time polling instead of while loops/vTaskDelay).
 *   Original implementation submitted in final report (Dec 2025) used blocking
 *   calls which worked but didn't meet real-time best practices.
 *   Refactored: January 2026
 * 
 * Date: November 2025 - January 2026
 * Course: 5th Semester Challenge - ITESM
 * License: MIT
 */

#include "definitions.h"

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

// TODO IN FUTURE PROJECTS: 
// - Add comprehensive error handling in classes and functions (return codes, validation)
// - Implement unit tests for critical functions (dosing, PID, sensor readings)
// - Add sensor fault detection and recovery mechanisms

// Initialize all hardware components
bool init_hardware() 
{
    // Install global GPIO ISR service once for all interrupt-based sensors
    // (must be done before any gpio_isr_handler_add calls)
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    
    // Actuators
    spindle.setup(spindle_config);
    spindle.setupPID(gains, spindle_dt_s);

    stepper.setup(steppper_gpio_pins, stepper_pul_pin, stepper_pul_feedback_pin, stepper_pwm_channel, 
                   &stepper_vel_config, stepper_duty_percentage, stepper_frequency, pulses_per_rev_stepper, 0);

    stepper.setupPID(gains_stepper, spindle_dt_s);

    piston.setup(PistonPins, PistonModes);

    water_pump.setup(water_pump_pins, water_channels, &dcMotorTimerConfig);

    dryer_motor.setup(dryer_pins, dryer_channels, &dcMotorTimerConfig);

    RGB_led.setup(RGB_pins, RGB_channels, &RGB_config, 1);

    // Sensors
    ultrasonic.setup(ECHO_PIN, TRIG_PIN, ultrasonic_trig_channel, ultrasonic_timer_config);

    start_routine_sensor.setup(START_IR_SENSOR_PIN, GPIO_MODE_INPUT);
    end_routine_sensor.setup(IR_SENSOR_PIN, GPIO_MODE_OUTPUT);
    homing_ir_sensor.setup(HOMING_IR_SENSOR_PIN, GPIO_MODE_INPUT);
    dispose_sensor.setup(DISPOSE_IR_SENSOR_PIN, GPIO_MODE_OUTPUT);
    
    color_sensor.setup(color_sensor_pins);
    color_sensor.calibrate(TCS34725_INTEGRATIONTIME_101MS, TCS34725_GAIN_16X);

    // Others
    crazy_frog.setup(BUZZER_PIN, buzzer_channel, &buzzer_timer_config);

    bt.begin("OscarRamo");

    prev_time = esp_timer_get_time();
    while (esp_timer_get_time() - prev_time < 5000000) 
    {
         // Wait 5 seconds for hardware to stabilize
    }

    return true;
}

// Piston Closed-Loop Control to Target Height; TODO: Later create a Piston closed-loop class
bool move_piston_to_height(float target_height_mm) 
{
    piston_position = ultrasonic.getDistance();

    error = target_height_mm - piston_position;

    if (fabs(error) <= 5.0f)  // 5mm tolerance
    {
        piston.stop();
        return true; // Target reached
    }
    else if (error > 5.0f) piston.moveUp();
    else if (error < -5.0f) piston.moveDown();
    
    return false; // Still moving to target
}

// Stepper Closed-Loop Homing using IR Sensor; TODO: Later create a Stepper closed-loop class
bool stepper_homing() 
{
    stepper.setFrequency(200.0f);
    homing_ir_sensor.set(1);
    if (homing_ir_sensor.get() == 1) 
    {
        stepper.stop();
        stepper.homePosition();
        return true; // Homing complete
    }
    else 
    {
        stepper.setCW();
    }
    
    return false; // Still homing
}

// ============================================================================
// ITERATIVE WATER DOSING ALGORITHM (NON-BLOCKING)
// ============================================================================
// 
// Portfolio Refactoring Note:
//   This function was converted from blocking (while loops + vTaskDelay) to
//   non-blocking state machine architecture for real-time operation compliance.
//   Original blocking version in final report worked correctly but violated
//   embedded systems best practices for control loop responsiveness.
//
// Function: water_dossing()
// Purpose: Iteratively add water to soap mixture to achieve target viscosity
// Algorithm: 
//   - Iteration 1: 85% of calculated water volume
//   - Iterations 2-3: 5% each for fine-tuning
//   - Quality check: ±10% tolerance, no overshoot allowed
// 
// Returns: DosingResult enum
//   - DOSING_IN_PROGRESS: Still working, call again next cycle
//   - DOSING_SUCCESS: Target achieved, quality passed
//   - DOSING_CONTINUE: Need next iteration (call with iteration+1)
//   - DOSING_FAIL_OVERSHOOT: Too much water added
//   - DOSING_FAIL_ITERATIONS: Max 3 iterations reached without success
// ============================================================================
DosingResult water_dossing(int iteration, float dose_percentage) 
{
    uint64_t current_time = esp_timer_get_time();
    
    switch(dosing_state) 
    {
        case DOSING_IDLE:
        case DOSING_CALC:
        {
            // Initialize for this iteration
            dosing_current_iteration = iteration;
            dosing_current_dose_percentage = dose_percentage;
            
            // Calculate target viscosity
            dosing_target_viscosity = (min_viscosity + max_viscosity) / 2.0f;
            
            // Exponential model coefficients (from SpindleVisc calibration)
            float A = 1975.6f;
            float B = -0.071f;
            
            // Calculate required water percentage
            float required_water_percentage = logf(dosing_target_viscosity / A) / B;
            
            // Clamp to valid range (0-25% water)
            required_water_percentage = fminf(fmaxf(required_water_percentage, 
                                                     min_water_percentage), 
                                              max_water_percentage);
            
            // Calculate water volume for this iteration
            float initial_soap_volume_mL = 100.0f;
            float total_water_needed_mL = initial_soap_volume_mL * 
                                         (required_water_percentage / (100.0f - required_water_percentage));
            float iteration_water_mL = total_water_needed_mL * (dose_percentage / 100.0f);
            
            // Calculate pump time
            float pump_flow_rate_mL_per_s = 10.0f;  // Calibrated value
            dosing_pump_time_s = iteration_water_mL / pump_flow_rate_mL_per_s;
            
            printf("\n=== ITERATION %d: Adding %.1f%% (%.2f mL, %.1f s) ===\n", 
                   iteration, dose_percentage, iteration_water_mL, dosing_pump_time_s);
            printf("Target: %.1f cPs (Range: %.1f - %.1f cPs)\n", 
                   dosing_target_viscosity, min_viscosity, max_viscosity);
            
            // Transition to pumping
            dosing_state = DOSING_PUMP;
            dosing_start_time = current_time;
            water_pump.setDuty(100.0f);
            message_box_case = 12 + iteration; // 13, 14, or 15
            return DOSING_IN_PROGRESS;
        }
        
        case DOSING_PUMP:
        {
            // Check if pumping time elapsed
            if (current_time - dosing_start_time >= (uint64_t)(dosing_pump_time_s * 1e6)) 
            {
                water_pump.setStop();
                printf("Water dispensing complete.\n");
                
                // Transition to stirring
                dosing_state = DOSING_STIR;
                dosing_start_time = current_time;
                spindle.setSpeed(30.0f);
                printf("Stirring and measuring viscosity...\n");
            }
            return DOSING_IN_PROGRESS;
        }
        
        case DOSING_STIR:
        {
            // Stir for 5 seconds
            if (current_time - dosing_start_time >= 5000000) 
            {
                // Transition to measurement
                dosing_state = DOSING_MEASURE;
                viscosity_avrg = spindle.measure_viscosity_readings();
                spindle.setSpeed(0.0f);
                
                // Immediately evaluate (fast operation)
                dosing_state = DOSING_EVALUATE;
            }
            return DOSING_IN_PROGRESS;
        }
        
        case DOSING_MEASURE:
            // Measurement happens in one call, immediately transition
            dosing_state = DOSING_EVALUATE;
            [[fallthrough]]; // Intentional fall-through to evaluation
            
        case DOSING_EVALUATE:
        {
            // Calculate error
            float error_percent = ((viscosity_avrg - dosing_target_viscosity) / 
                                   dosing_target_viscosity) * 100.0f;
            printf("Measured: %.1f cPs | Target: %.1f cPs | Error: %.1f%%\n", 
                   viscosity_avrg, dosing_target_viscosity, error_percent);
            
            // Check quality control criteria
            if (fabs(error_percent) <= 10.0f && error_percent >= 0.0f) 
            {
                // SUCCESS - Within tolerance
                printf("✓ SUCCESS - Within tolerance! Directing to destination.\n");
                message_box_case = 16;
                dosing_state = DOSING_IDLE; // Reset for next use
                return DOSING_SUCCESS;
            }
            else if (error_percent < 0.0f) 
            {
                // OVERSHOOT - Too much water
                printf("✗ OVERSHOOT - Too much water added. Directing to reject warehouse.\n");
                message_box_case = 17;
                dosing_state = DOSING_IDLE; // Reset for next use
                return DOSING_FAIL_OVERSHOOT;
            }
            else if (dosing_current_iteration >= 3) 
            {
                // FAILURE - Max iterations reached
                printf("✗ FAILURE - Target not reached after 3 iterations. Directing to reject warehouse.\n");
                message_box_case = 17;
                dosing_state = DOSING_IDLE; // Reset for next use
                return DOSING_FAIL_ITERATIONS;
            }
            else 
            {
                // CONTINUE - Need another iteration
                printf("→ Proceeding to iteration %d...\n", dosing_current_iteration + 1);
                dosing_state = DOSING_IDLE; // Reset for next iteration
                return DOSING_CONTINUE;
            }
        }
        
        case DOSING_COMPLETE:
        default:
            dosing_state = DOSING_IDLE;
            return DOSING_IN_PROGRESS;
    }
}

// ============================================================================
// TELEMETRY AND COMMUNICATION
// ============================================================================

// Update all telemetry readings from sensors
// TODO: Add viscosity, IR sensor, color sensor readings, current for monitoring
bool update_telemetry() 
{
    piston_position = ultrasonic.getDistance();

    stepper_angle = stepper.readAngle();
    stepper_speed = stepper.readSpeed();

    spindle_rpms = (spindle.getSpeed() / 360.0f) * 60.0f;

    spindle_current = spindle.getCurrent();

    color_sensor.getRawRGBC(&r_sensor_output, &g_sensor_output, &b_sensor_output, &c_sensor_output);

    start_automation = start_routine_sensor.get();
    return true;
}

// Handle UART RX/TX data with LabVIEW
bool RX_TX_data() 
{
    // Scale RGBC to 8-bit (0-255) for LabVIEW; omit C channel if not used
    uint16_t max_rgbc = r_sensor_output;
    if (g_sensor_output > max_rgbc) max_rgbc = g_sensor_output;
    if (b_sensor_output > max_rgbc) max_rgbc = b_sensor_output;
    if (c_sensor_output > max_rgbc) max_rgbc = c_sensor_output;
    if (max_rgbc == 0) max_rgbc = 1; // avoid divide by zero
    uint8_t r8 = (uint8_t)((r_sensor_output * 255u) / max_rgbc);
    uint8_t g8 = (uint8_t)((g_sensor_output * 255u) / max_rgbc);
    uint8_t b8 = (uint8_t)((b_sensor_output * 255u) / max_rgbc);

    // Device -> LabVIEW: numeric fields first, then status text (CSV, one line)
    // Format: piston,stepper,rpms,viscosity,r8,g8,b8,msg
    message_length = snprintf(
        buffer,
        sizeof(buffer),
        "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%u,%u,%u,%d\n",
        piston_position,
        stepper_angle,
        stepper_speed,
        spindle_rpms,
        spindle_current,
        viscosity_avrg,
        static_cast<unsigned>(r8),
        static_cast<unsigned>(g8),
        static_cast<unsigned>(b8),
        message_box_case
    );
    bt.write(buffer, message_length);

    len = bt.available(); // Check for incoming data
    if (len) 
    {
        // Reset ALL input values before parsing new data
        btn_piston_up = 0;
        btn_piston_down = 0;
        btn_stepper_cw = 0;
        btn_stepper_ccw = 0;
        knob_stepper_angle = 0.0f;
        knob_stepper_frequency = 0.0f;
        knob_spindle_pwm = 0.0f;
        knob_spindle_rpm = 0.0f;
        btn_measure_viscosity = 0;
        btn_water_pump = 0;
        btn_dryer_on = 0;
        
        bt.read(buffer, len);
        buffer[len] = '\0';  // Null-terminate the buffer to prevent corruption
        printf("UART received: %s\n", buffer);
        // Format: LabVIEW inputs from top to bottom (CSV, one line)
        sscanf(buffer, "%d,%d,%d,%d,%f,%f,%f,%f,%d,%d,%d,%d\n",
                &btn_piston_up, 
                &btn_piston_down, 
                &btn_stepper_cw, 
                &btn_stepper_ccw,
                &knob_stepper_angle,
                &knob_stepper_frequency,
                &knob_spindle_pwm,
                &knob_spindle_rpm,
                &btn_measure_viscosity,
                &btn_water_pump,
                &btn_dryer_on,
                &state_machine_case);
        
        printf("Parsed: Pist_Up=%d, Pist_Dn=%d, Step_CW=%d, Step_CCW=%d, Angle=%.2f, Freq=%.2f, PWM=%.2f, RPM=%.2f, Visc=%d, Water=%d, Dryer=%d\n", 
               btn_piston_up, btn_piston_down, btn_stepper_cw, btn_stepper_ccw,
               knob_stepper_angle, knob_stepper_frequency, knob_spindle_pwm, knob_spindle_rpm,
               btn_measure_viscosity, btn_water_pump, btn_dryer_on);
        
        if (state_machine_case == 0) 
        {
            // Manual control mode
            // Determine state based on current values (only when new data arrives)
            if (btn_piston_up) manual_case_state = PISTON_UP;
            else if (btn_piston_down) manual_case_state = PISTON_DOWN;
            else if (btn_stepper_cw) manual_case_state = STEPPER_CW;
            else if (btn_stepper_ccw) manual_case_state = STEPPER_CCW;
            else if (knob_stepper_angle > 0.1f) manual_case_state = STEPPER_SPECIFIC;  // Added tolerance 0.1f
            else if (knob_stepper_frequency > 0.1f) manual_case_state = STEPPER_FREQ;  // Added tolerance 0.1f
            else if (knob_spindle_pwm > 0.1f) manual_case_state = SPINDLE_PWM;  // Added tolerance 0.1f
            else if (knob_spindle_rpm > 0.1f) manual_case_state = SPINDLE_RPMs;  // Added tolerance 0.1f
            else if (btn_measure_viscosity) manual_case_state = SPINDLE_MEASURE_VISCOSITY;
            else if (btn_water_pump) manual_case_state = PUMP_WATER;
            else if (btn_dryer_on) manual_case_state = DRYER_ON;
            else manual_case_state = MANUAL_IDLE;  // Default: IDLE when nothing is pressed
        }
        else 
        {
            // Automatic control mode
        }
        
    }

    return true;
}

// =============================================================================
// MANUAL CONTROL SYSTEM STATE MACHINE
// =============================================================================

// Control system state machine
bool manual_system_state_machine() 
{
    // If we are leaving viscosity mode, ensure related actions are stopped once
    if (previous_state == SPINDLE_MEASURE_VISCOSITY && manual_case_state != SPINDLE_MEASURE_VISCOSITY) {
        spindle.setSpeed(0.0f);
        spindle.setPWM(0.0f);
        crazy_frog.stopSong();
    }
    
    switch(manual_case_state) 
    {
        case MANUAL_IDLE:
            // Do nothing
            r = 0; g = 0; b = 0; // Off
            RGB_led.setColor(r, g, b);
            piston.stop();
            stepper.stop();
            spindle.setPWM(0.0f);
            spindle.setSpeed(0.0f);
            spindle.resetViscosityMeasurement();
            water_pump.setStop();
            dryer_motor.setStop();
            crazy_frog.stopSong();
            message_box_case = 1; // System idle
            break;
        case PISTON_UP:
            r = 0; g = 255; b = 0; // Green
            RGB_led.setColor(r, g, b);
            piston.moveUp();
            message_box_case = 2; // Piston moving up
            break;
        case PISTON_DOWN:
            r = 0; g = 255; b = 0; // Green
            RGB_led.blinkColor(r, g, b, 500);
            piston.moveDown();
            message_box_case = 3; // Piston moving down
            break;
        case STEPPER_CW:
            r = 0; g = 0; b = 255; // Blue
            RGB_led.setColor(r, g, b);
            stepper.setCW();
            message_box_case = 4; // Stepper moving clockwise
            break;
        case STEPPER_CCW:
            r = 0; g = 0; b = 255; // Blue
            RGB_led.blinkColor(r, g, b, 500);
            stepper.setCCW();
            message_box_case = 5; // Stepper moving counter-clockwise
            break;
        case STEPPER_SPECIFIC:
            r = 0; g = 0; b = 255; // Blue
            RGB_led.blinkColor(r, g, b, 200);
            error = stepper.desiredAngle(knob_stepper_angle);
            // When target reached (error = 0), transition to IDLE and clear knob value
            if (error == 0.0f) {
                knob_stepper_angle = 0.0f;  // Clear the input to prevent re-triggering
                manual_case_state = MANUAL_IDLE;
            }
            crazy_frog.stopSong();
            message_box_case = 6; // Stepper moving to specific angle
            break;
        case STEPPER_FREQ:
            r = 0; g = 0; b = 255; // Blue
            RGB_led.blinkColor(r, g, b, 100);
            stepper.setFrequency(knob_stepper_frequency);
            message_box_case = 7; // Stepper frequency set
            break;
        case SPINDLE_PWM:
            r = 255; g = 255; b = 0; // Yellow
            RGB_led.setColor(r, g, b);
            spindle.setPWM(knob_spindle_pwm);
            message_box_case = 8; // Spindle PWM set
            break;
        case SPINDLE_RPMs:
            r = 255; g = 255; b = 0; // Yellow
            RGB_led.blinkColor(r, g, b, 500);
            spindle.setSpeed(knob_spindle_rpm);
            message_box_case = 9; // Spindle RPMs set
            break;
        case SPINDLE_MEASURE_VISCOSITY:
            // Reset song on first entry into this state
            if (previous_state != SPINDLE_MEASURE_VISCOSITY) {
                crazy_frog.stopSong();
            }
            r = 255; g = 255; b = 0; // Yellow
            RGB_led.blinkColor(r, g, b, 200);
            spindle.setSpeed(30.0f); // Maintain constant speed for viscosity measurement
            viscosity_avrg = spindle.measure_viscosity_readings();
            spindle_rpms = (spindle.getSpeed() / 360.0f) * 60.0f; // Update RPM display
            crazy_frog.playSong();
            message_box_case = 10; // Measuring viscosity
            break;
        case PUMP_WATER:
            r = 255; g = 0; b = 255; // Magenta
            RGB_led.setColor(r, g, b);
            water_pump.setDuty(100.0f); // Full duty cycle
            message_box_case = 11; // Water pump ON
            break;
        case DRYER_ON:
            r = 255; g = 165; b = 0; // Orange
            RGB_led.setColor(r, g, b);
            dryer_motor.setDuty(100.0f); // Full duty cycle
            message_box_case = 12; // Dryer ON
            break;
    }

    previous_state = manual_case_state; // Remember current state for next iteration
    return true;
}

// =============================================================================
// AUTOMATIC CONTROL SYSTEM STATE MACHINE
// =============================================================================

// Automatic control system state machine
bool automatic_system_state_machine() 
{
    switch(automatic_case_state) 
    {
        case HOMING_ROUTINE:
            r = 255; g = 255; b = 0; // Yellow
            RGB_led.blinkColor(r, g, b, 500);
            move_piston_to_height(300.0f);  // 30cm = 300mm
            stepper_homing();
            automatic_case_state = AUTOMATIC_IDLE;
            end_routine_sensor.set(0); // Deactivate end routine sensor
            dispose_sensor.set(0); // Deactivate dispose sensor
            break;

        case AUTOMATIC_IDLE:
            r = 0; g = 0; b = 0; // Off
            RGB_led.setColor(r, g, b);
            piston.stop();
            stepper.stop();
            spindle.setPWM(0.0f);
            spindle.setSpeed(0.0f);
            spindle.resetViscosityMeasurement();
            water_pump.setStop();
            dryer_motor.setStop();
            crazy_frog.stopSong();
            message_box_case = 1; // System idle
            break;
        case CONTAINER_POSITIONING:
        {
            static bool positioning_started = false;
            static uint64_t positioning_start_time = 0;
            
            if (start_automation == 1)
            {
                if (!positioning_started)
                {
                    r = 0; g = 255; b = 0; // Green
                    RGB_led.setColor(r, g, b);
                    positioning_start_time = esp_timer_get_time();
                    positioning_started = true;
                }
                
                message_box_case = 2; // Container positioned
                
                // Check if 5 seconds have elapsed
                if (esp_timer_get_time() - positioning_start_time >= 5000000)
                {
                    positioning_started = false;
                    automatic_case_state = DETECT_TAG_COLOR;
                }
            }
            break;
        }
            
        case DETECT_TAG_COLOR:
            color_sensor.getRawRGBC(&r_sensor_output, &g_sensor_output, &b_sensor_output, &c_sensor_output);
            if (color_sensor.getColorDetected() == Color::COLOR_RED)
            {
                message_box_case = 3; // Low Viscosity
                min_viscosity = 0.0f;
                max_viscosity = 800.0f;
            }
            else if (color_sensor.getColorDetected() == Color::COLOR_GREEN)
            {
                message_box_case = 4; // Medium Viscosity
                min_viscosity = 800.0f;
                max_viscosity = 1400.0f;
            }
            else if (color_sensor.getColorDetected() == Color::COLOR_BLUE)
            {
                message_box_case = 5; // High Viscosity
                min_viscosity = 1400.0f;
                max_viscosity = 2000.0f;
            }
            else
            {
                message_box_case = 6; // No dominant color detected
            }
            automatic_case_state = MEASURE_VISCOSITY;
            break;

        case MEASURE_VISCOSITY:
        {
            static bool piston_moved = false;
            static bool stirring_started = false;
            static uint64_t stir_start_time = 0;
            
            // Step 1: Move piston to measurement height
            if (!piston_moved)
            {
                if (move_piston_to_height(210.0f))  // Returns true when reached
                {
                    piston_moved = true;
                    message_box_case = 8; // Measuring viscosity
                }
                break; // Stay in this state until piston positioned
            }
            
            // Step 2: Start stirring
            if (!stirring_started)
            {
                spindle.setSpeed(30.0f);
                stir_start_time = esp_timer_get_time();
                stirring_started = true;
            }
            
            // Step 3: Wait for 5 seconds of stirring
            if (esp_timer_get_time() - stir_start_time >= 5000000)
            {
                viscosity_avrg = spindle.measure_viscosity_readings();
                spindle.setSpeed(0.0f);
                printf("Initial viscosity: %.1f cPs\n", viscosity_avrg);
                
                // Reset state variables for next time
                piston_moved = false;
                stirring_started = false;
                
                automatic_case_state = DOSSING_ADJUSTMENT;
            }
            break;
        }
            
        case DOSSING_ADJUSTMENT:
        {
            // Non-blocking dosing with state tracking
            static int current_iteration = 1;
            static DosingResult dosing_result = DOSING_IN_PROGRESS;
            static bool dosing_started = false;
            
            // Start first iteration
            if (!dosing_started) 
            {
                current_iteration = 1;
                dosing_started = true;
                dosing_state = DOSING_IDLE; // Reset state machine
            }
            
            // Call non-blocking dosing function
            if (current_iteration == 1) 
            {
                dosing_result = water_dossing(1, 85.0f);
            }
            else if (current_iteration == 2) 
            {
                dosing_result = water_dossing(2, 5.0f);
            }
            else if (current_iteration == 3) 
            {
                dosing_result = water_dossing(3, 5.0f);
            }
            
            // Process result
            switch(dosing_result) 
            {
                case DOSING_IN_PROGRESS:
                    // Still working, stay in this state
                    break;
                    
                case DOSING_SUCCESS:
                    // Quality passed!
                    message_box_case = 9; // Viscosity within range
                    dosing_started = false;
                    current_iteration = 1;
                    automatic_case_state = CLEANING_STATION;
                    break;
                    
                case DOSING_CONTINUE:
                    // Need next iteration
                    current_iteration++;
                    dosing_state = DOSING_IDLE; // Reset for next iteration
                    if (current_iteration > 3) 
                    {
                        // Safety: shouldn't happen, but handle it
                        printf("ERROR: Exceeded max iterations\n");
                        message_box_case = 10;
                        move_piston_to_height(300.0f);
                        dispose_sensor.set(1);
                        dosing_started = false;
                        current_iteration = 1;
                        automatic_case_state = AUTOMATIC_IDLE;
                    }
                    break;
                    
                case DOSING_FAIL_OVERSHOOT:
                case DOSING_FAIL_ITERATIONS:
                    // Failed - reject
                    message_box_case = 10; // Viscosity out of range, discarding
                    move_piston_to_height(300.0f);
                    dispose_sensor.set(1);
                    dosing_started = false;
                    current_iteration = 1;
                    automatic_case_state = AUTOMATIC_IDLE;
                    break;
            }
            break;
        }
            
        case CLEANING_STATION:
        {
            static bool stepper_positioned = false;
            static bool piston_positioned = false;
            static bool cleaning_started = false;
            static uint64_t cleaning_start_time = 0;
            
            // Step 1: Position stepper
            if (!stepper_positioned)
            {
                float error = stepper.desiredAngle(120.0f);
                if (fabs(error) < 1.0f)  // Within 1 degree
                {
                    stepper_positioned = true;
                }
                break;
            }
            
            // Step 2: Position piston
            if (!piston_positioned)
            {
                if (move_piston_to_height(210.0f))
                {
                    piston_positioned = true;
                }
                break;
            }
            
            // Step 3: Start cleaning timer
            if (!cleaning_started)
            {
                spindle.setPWM(0.0f);
                cleaning_start_time = esp_timer_get_time();
                cleaning_started = true;
            }
            
            message_box_case = 11; // Cleaning station active
            
            // Step 4: Wait for cleaning duration (60 seconds)
            if (esp_timer_get_time() - cleaning_start_time >= 60000000)
            {
                // Move piston up and reset state
                if (move_piston_to_height(300.0f))
                {
                    stepper_positioned = false;
                    piston_positioned = false;
                    cleaning_started = false;
                    automatic_case_state = DRYING_STATION;
                }
            }
            break;
        }

        case DRYING_STATION:
        {
            static bool stepper_positioned = false;
            static bool piston_positioned = false;
            static bool drying_started = false;
            static uint64_t drying_start_time = 0;
            
            // Step 1: Position stepper
            if (!stepper_positioned)
            {
                float error = stepper.desiredAngle(240.0f);
                if (fabs(error) < 1.0f)  // Within 1 degree
                {
                    stepper_positioned = true;
                }
                break;
            }
            
            // Step 2: Position piston
            if (!piston_positioned)
            {
                if (move_piston_to_height(210.0f))
                {
                    piston_positioned = true;
                }
                break;
            }
            
            // Step 3: Start drying
            if (!drying_started)
            {
                dryer_motor.setDuty(100.0f);
                drying_start_time = esp_timer_get_time();
                drying_started = true;
            }
            
            message_box_case = 12; // Drying station active
            spindle.setSpeed(30.0f);
            
            // Step 4: Wait for drying duration (60 seconds)
            if (esp_timer_get_time() - drying_start_time >= 60000000)
            {
                dryer_motor.setStop();
                spindle.setSpeed(0.0f);
                
                // Move piston up and reset state
                if (move_piston_to_height(300.0f))
                {
                    stepper_positioned = false;
                    piston_positioned = false;
                    drying_started = false;
                    start_automation = 0; // Reset automation start flag
                    automatic_case_state = AUTOMATIC_IDLE;
                }
            }
            break;
        }
    }
    return true;
}

// ============================================================================
// MAIN FUNCTION
// ============================================================================

extern "C" void app_main() 
{
    esp_task_wdt_deinit(); // Disable the watchdog timer

    if (!init_hardware()) 
    {
        printf("ERROR: Hardware initialization failed!\n");
        return;
    }
    else 
    {
        printf("Hardware initialization successful!\n");
    }

    prev_time = esp_timer_get_time();

    while (true) 
    {
        current_time = esp_timer_get_time();
        if (current_time - prev_time >= dt_us) 
        {
            // Update all sensor telemetry readings
                // All function calls are commented out for isolation.
                // Update all sensor telemetry readings
                if (!update_telemetry()) 
                {
                    printf("ERROR: Telemetry update failed!\n");
                    return;
                }

                if (state_machine_case == 0) 
                {
                    // Manual control mode
                    if (!manual_system_state_machine()) 
                    {
                        printf("ERROR: Manual system state machine failed!\n");
                        return;
                    }
                } 
                else 
                {
                    // Automatic control mode
                    if (!automatic_system_state_machine()) 
                    {
                        printf("ERROR: Automatic system state machine failed!\n");
                        return;
                    }
                }
                
                if (!RX_TX_data()) 
                {
                    printf("ERROR: RX/TX data handling failed!\n");
                    return;
                }

            prev_time = current_time; // Update previous time   

        }
    }
}
