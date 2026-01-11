/*
 * Library: RGBColorSensorTCS34725
 * File: RGBColorSensorTCS34725.h
 * Author: Oscar Gadiel Ramo Martínez
 * 
 * Description:
 *   Driver for TCS34725 RGB color sensor via I2C.
 *   Provides RGBC (Red, Green, Blue, Clear) light sensing with configurable
 *   integration time and gain. Includes color detection algorithm for
 *   dominant color identification.
 *   
 *   Features:
 *     - I2C communication (address 0x29)
 *     - Configurable integration time (2.4ms - 614ms)
 *     - Configurable gain (1x, 4x, 16x, 60x)
 *     - Raw RGBC reading (16-bit per channel)
 *     - Dominant color detection (RED, GREEN, BLUE, NONE)
 *     - Calibration support
 * 
 * Usage:
 *   TCS3475_RGB sensor;
 *   sensor.setup(pins);
 *   sensor.calibrate(TCS34725_INTEGRATIONTIME_101MS, TCS34725_GAIN_16X);
 *   sensor.getRawRGBC(&r, &g, &b, &c);
 *   Color detected = sensor.getColorDetected();
 * 
 * Date: November - December 2025
 * Course: 5th Semester Challenge - ITESM
 * License: MIT
 */

#ifndef __COLOR_SENSOR_H__
#define __COLOR_SENSOR_H__

#include "SimpleI2C.h"
#include <cstdint>
#include <esp_timer.h>

// ============================================================================
// TCS34725 RGB Color Sensor Definitions
// ============================================================================

// ============================================================================
// TCS34725 RGB Color Sensor Definitions
// ============================================================================

// I2C Address
#define TCS34725_ADDRESS          0x29

// Command Register Bit
#define TCS34725_COMMAND_BIT      0x80

// ============================================================================
// Register Addresses (from datasheet Table 3)
// ============================================================================
#define TCS34725_ENABLE           0x00  // Enables states and interrupts
#define TCS34725_ATIME            0x01  // RGBC time
#define TCS34725_WTIME            0x03  // Wait time
#define TCS34725_AILTL            0x04  // Clear interrupt low threshold low byte
#define TCS34725_AILTH            0x05  // Clear interrupt low threshold high byte
#define TCS34725_AIHTL            0x06  // Clear interrupt high threshold low byte
#define TCS34725_AIHTH            0x07  // Clear interrupt high threshold high byte
#define TCS34725_PERS             0x0C  // Interrupt persistence filter
#define TCS34725_CONFIG           0x0D  // Configuration
#define TCS34725_CONTROL          0x0F  // Control (Gain)
#define TCS34725_ID               0x12  // Device ID
#define TCS34725_STATUS           0x13  // Device status
#define TCS34725_CDATAL           0x14  // Clear data low byte
#define TCS34725_CDATAH           0x15  // Clear data high byte
#define TCS34725_RDATAL           0x16  // Red data low byte
#define TCS34725_RDATAH           0x17  // Red data high byte
#define TCS34725_GDATAL           0x18  // Green data low byte
#define TCS34725_GDATAH           0x19  // Green data high byte
#define TCS34725_BDATAL           0x1A  // Blue data low byte
#define TCS34725_BDATAH           0x1B  // Blue data high byte

// ============================================================================
// Enable Register (0x00) Bit Definitions
// ============================================================================
#define TCS34725_ENABLE_AIEN      0x10  // RGBC Interrupt Enable
#define TCS34725_ENABLE_WEN       0x08  // Wait Enable
#define TCS34725_ENABLE_AEN       0x02  // RGBC Enable
#define TCS34725_ENABLE_PON       0x01  // Power ON

// ============================================================================
// Integration Time Values (ATIME Register)
// ============================================================================
#define TCS34725_INTEGRATIONTIME_2_4MS   0xFF  // 2.4ms  - 1 cycle
#define TCS34725_INTEGRATIONTIME_24MS    0xF6  // 24ms   - 10 cycles
#define TCS34725_INTEGRATIONTIME_50MS    0xEB  // 50ms   - 20 cycles
#define TCS34725_INTEGRATIONTIME_101MS   0xD5  // 101ms  - 42 cycles
#define TCS34725_INTEGRATIONTIME_154MS   0xC0  // 154ms  - 64 cycles
#define TCS34725_INTEGRATIONTIME_700MS   0x00  // 700ms  - 256 cycles

// ============================================================================
// Gain Values (CONTROL Register)
// ============================================================================
#define TCS34725_GAIN_1X          0x00  // 1x gain
#define TCS34725_GAIN_4X          0x01  // 4x gain
#define TCS34725_GAIN_16X         0x02  // 16x gain
#define TCS34725_GAIN_60X         0x03  // 60x gain

// ============================================================================
// Device ID Values
// ============================================================================
#define TCS34725_ID_1             0x44  // TCS34725 ID variant 1
#define TCS34725_ID_2             0x4D  // TCS34725 ID variant 2

// ============================================================================
// TCS34725 RGB Color Sensor Class Definitions
// ============================================================================

// Color detection enum
enum Color
{
    COLOR_NONE = 0,
    COLOR_RED = 1,
    COLOR_GREEN = 2,
    COLOR_BLUE = 3
};

/**
 * @brief TCS34725 RGB Color Sensor Class
 * 
 * Simple interface for TCS34725 color sensor using I2C.
 * Detects red, green, and blue colors from objects.
 */
class TCS3475_RGB
{
public:
    TCS3475_RGB();

    /**
     * @brief Setup and initialize the color sensor
     * 
     * Initializes I2C communication, verifies sensor address (0x29),
     * and configures the sensor with default settings.
     * 
     * @param sda_pin GPIO pin for I2C SDA
     * @param scl_pin GPIO pin for I2C SCL
     * @return true if sensor initialized successfully
     * @return false if sensor not found or initialization failed
     */
    bool setup(uint8_t sensor_pins[2]);

    /**
     * @brief Calibrate sensor for current lighting conditions
     * 
     * Adjusts integration time and gain to optimize for ambient light.
     * Call this method when lighting conditions change.
     * 
     * @param integration_time How long sensor collects light (2.4ms to 700ms)
     *                         Use TCS34725_INTEGRATIONTIME_* defines
     *                         Longer = more sensitive, slower readings
     * @param gain Amplification level (1x, 4x, 16x, 60x)
     *             Use TCS34725_GAIN_* defines
     *             Higher = better in low light, may saturate in bright light
     */
    void calibrate(uint8_t integration_time, uint8_t gain);

    /**
     * @brief Get raw RGBC values from sensor (RX from sensor)
     * 
     * Reads 16-bit values for Red, Green, Blue, and Clear channels
     * from the TCS34725 sensor via I2C.
     * 
     * @param r Pointer to store red channel value (0-65535)
     * @param g Pointer to store green channel value (0-65535)
     * @param b Pointer to store blue channel value (0-65535)
     * @param c Pointer to store clear/brightness value (0-65535)
     */
    void getRawRGBC(uint16_t* r, uint16_t* g, uint16_t* b, uint16_t* c);

    /**
     * @brief Detect which color is dominant
     * 
     * Analyzes RGB values and determines if red, green, blue, or no
     * dominant color is detected using threshold-based algorithm.
     * 
     * @return Color enum (COLOR_RED, COLOR_GREEN, COLOR_BLUE, or COLOR_NONE)
     */
    Color getColorDetected();

private:
    SimpleI2C _i2c;              // I2C communication interface
    bool _initialized;            // Sensor initialization status
    uint8_t _integration_time;    // Current integration time setting
    uint8_t _gain;                // Current gain setting

    // Low-level I2C communication methods
    void writeRegister(uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);
    uint16_t read16(uint8_t reg);
    bool verifyID();
};

#endif // __COLOR_SENSOR_H__
