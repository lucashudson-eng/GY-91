#pragma once

#include <Arduino.h>
#include <SPI.h>

// IDs and basic registers
#define MPU9250_CHIP_ID        0x71
#define MPU9250_REG_WHO_AM_I   0x75

class MPU9250 {
public:
    // Constructor: SPI using hardware default pins (takes only CS)
    explicit MPU9250(uint8_t csPin);

    // Constructor: SPI providing all pins (SCK, MISO, MOSI, CS)
    MPU9250(uint8_t sckPin, uint8_t misoPin, uint8_t mosiPin, uint8_t csPin);

    // Initialize device; returns true if CHIP_ID matches
    bool begin(uint8_t chipId = MPU9250_CHIP_ID);

private:
    // Pin configuration
    bool useHardwareSPI = true;
    uint8_t pinCS = 0xFF;
    uint8_t pinSCK = 0xFF;
    uint8_t pinMISO = 0xFF;
    uint8_t pinMOSI = 0xFF;

    // Internal helpers
    uint8_t spiTransfer(uint8_t value);
    void select();
    void deselect();

    // SPI register access
    uint8_t readRegister(uint8_t reg);
    void writeRegister(uint8_t reg, uint8_t value);
};