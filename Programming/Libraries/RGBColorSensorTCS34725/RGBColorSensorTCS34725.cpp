/*
 * Library: RGBColorSensorTCS34725
 * File: RGBColorSensorTCS34725.cpp
 * Author: Oscar Gadiel Ramo Martínez
 * 
 * Description:
 *   Implementation of TCS34725 RGB color sensor driver.
 *   Handles I2C communication, sensor configuration, and color detection.
 * 
 * Date: November - December 2025
 * Course: 5th Semester Challenge - ITESM
 * License: MIT
 */

#include "RGBColorSensorTCS34725.h"

TCS3475_RGB::TCS3475_RGB()
    : _initialized(false)
    , _integration_time(TCS34725_INTEGRATIONTIME_50MS)
    , _gain(TCS34725_GAIN_4X)
{
}

bool TCS3475_RGB::setup(uint8_t sensor_pins[2])
{
    uint8_t sda_pin = sensor_pins[0];
    uint8_t scl_pin = sensor_pins[1];

    // Initialize I2C communication with sensor address 0x29
    _i2c.setup(TCS34725_ADDRESS, 400000, sda_pin, scl_pin);
    
    // Time polling: Wait 10ms for I2C to stabilize
    int64_t start_time = esp_timer_get_time();
    while ((esp_timer_get_time() - start_time) < 10000)
    {
        // Polling - non-blocking wait
    }
    
    // Verify sensor is present at correct I2C address
    if (!verifyID())
    {
        _initialized = false;
        return false;
    }
    
    // Configure sensor with default settings (50ms integration, 4x gain)
    writeRegister(TCS34725_ATIME, _integration_time);
    writeRegister(TCS34725_CONTROL, _gain);
    
    // Power on and enable RGBC sensing
    writeRegister(TCS34725_ENABLE, TCS34725_ENABLE_PON);
    
    // Time polling: Wait 3ms for power-on
    start_time = esp_timer_get_time();
    while ((esp_timer_get_time() - start_time) < 3000)
    {
        // Polling - non-blocking wait
    }
    
    writeRegister(TCS34725_ENABLE, TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN);
    
    // Time polling: Wait 60ms for first integration cycle
    start_time = esp_timer_get_time();
    while ((esp_timer_get_time() - start_time) < 60000)
    {
        // Polling - non-blocking wait
    }
    
    _initialized = true;
    return true;
}

void TCS3475_RGB::calibrate(uint8_t integration_time, uint8_t gain)
{
    // Update integration time (how long sensor collects light)
    _integration_time = integration_time;
    writeRegister(TCS34725_ATIME, _integration_time);
    
    // Update gain (signal amplification)
    _gain = gain;
    writeRegister(TCS34725_CONTROL, _gain);
}

void TCS3475_RGB::getRawRGBC(uint16_t* r, uint16_t* g, uint16_t* b, uint16_t* c)
{
    if (!_initialized)
    {
        *r = *g = *b = *c = 0;
        return;
    }
    
    // Read 16-bit values from sensor registers via I2C
    *c = read16(TCS34725_CDATAL);  // Clear/brightness channel
    *r = read16(TCS34725_RDATAL);  // Red channel
    *g = read16(TCS34725_GDATAL);  // Green channel
    *b = read16(TCS34725_BDATAL);  // Blue channel
}

Color TCS3475_RGB::getColorDetected()
{
    uint16_t r, g, b, c;
    getRawRGBC(&r, &g, &b, &c);
    
    // Detection algorithm: dominant color must exceed sum of others × threshold
    // Thresholds tuned based on Arduino reference code
    if (r > (g + b) * 0.9f)
        return COLOR_RED;
    
    if (g > (r + b) * 0.9f)
        return COLOR_GREEN;
    
    if (b > (r + g) * 0.7f)
        return COLOR_BLUE;
    
    return COLOR_NONE;
}

// ============================================================================
// Low-level I2C communication methods
// ============================================================================

void TCS3475_RGB::writeRegister(uint8_t reg, uint8_t value)
{
    // TX to sensor: Send command bit + register address, then value
    uint8_t data[2] = {static_cast<uint8_t>(TCS34725_COMMAND_BIT | reg), value};
    _i2c.write(data, 2);
}

uint8_t TCS3475_RGB::readRegister(uint8_t reg)
{
    // RX from sensor: Send register address, receive 1 byte
    uint8_t cmd = TCS34725_COMMAND_BIT | reg;
    uint8_t value = 0;
    _i2c.read(&cmd, 1, &value, 1);
    return value;
}

uint16_t TCS3475_RGB::read16(uint8_t reg)
{
    // RX from sensor: Send register address, receive 2 bytes (low + high)
    uint8_t cmd = TCS34725_COMMAND_BIT | reg;
    uint8_t data[2] = {0, 0};
    _i2c.read(&cmd, 1, data, 2);
    return (data[1] << 8) | data[0];  // Combine high and low bytes
}

bool TCS3475_RGB::verifyID()
{
    // Check if sensor responds with correct ID (0x44 or 0x4D)
    uint8_t id = readRegister(TCS34725_ID);
    return (id == TCS34725_ID_1 || id == TCS34725_ID_2);
}
