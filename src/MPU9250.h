#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

// IDs and basic registers
#define MPU9250_CHIP_ID        0x71
#define MPU9250_REG_WHO_AM_I   0x75

// I2C address
#define MPU9250_I2C_ADDR       0x68
#define MPU9250_I2C_ADDR_ALT   0x69

// Power management
#define MPU9250_REG_PWR_MGMT_1     0x6B
#define MPU9250_REG_PWR_MGMT_2     0x6C
#define MPU9250_REG_USER_CTRL      0x6A

// Configuration registers
#define MPU9250_REG_SMPLRT_DIV     0x19
#define MPU9250_REG_CONFIG         0x1A
#define MPU9250_REG_ACCEL_CONFIG   0x1C
#define MPU9250_REG_ACCEL_CONFIG_2 0x1D
#define MPU9250_REG_GYRO_CONFIG    0x1B

// Data registers
#define MPU9250_REG_ACCEL_XOUT_H   0x3B
#define MPU9250_REG_GYRO_XOUT_H    0x43

// Accelerometer scales
#define MPU9250_ACCEL_FS_2G  0x00
#define MPU9250_ACCEL_FS_4G  0x08
#define MPU9250_ACCEL_FS_8G  0x10
#define MPU9250_ACCEL_FS_16G 0x18

// Gyroscope scales
#define MPU9250_GYRO_FS_250DPS  0x00
#define MPU9250_GYRO_FS_500DPS  0x08
#define MPU9250_GYRO_FS_1000DPS 0x10
#define MPU9250_GYRO_FS_2000DPS 0x18

// DLPF filter bandwidth settings (for CONFIG register)
#define MPU9250_DLPF_250HZ   0x00
#define MPU9250_DLPF_184HZ   0x01
#define MPU9250_DLPF_92HZ    0x02
#define MPU9250_DLPF_41HZ    0x03
#define MPU9250_DLPF_20HZ    0x04
#define MPU9250_DLPF_10HZ    0x05
#define MPU9250_DLPF_5HZ     0x06
#define MPU9250_DLPF_3600HZ  0x07

// DLPF filter bandwidth settings (for ACCEL_CONFIG_2 register)
#define MPU9250_ACCEL_DLPF_460HZ  0x00
#define MPU9250_ACCEL_DLPF_184HZ  0x01
#define MPU9250_ACCEL_DLPF_92HZ   0x02
#define MPU9250_ACCEL_DLPF_41HZ   0x03
#define MPU9250_ACCEL_DLPF_20HZ   0x04
#define MPU9250_ACCEL_DLPF_10HZ   0x05
#define MPU9250_ACCEL_DLPF_5HZ    0x06

struct AccelData {
    float x, y, z;
};

struct GyroData {
    float x, y, z;
};

class MPU9250 {
public:
    // Constructor: I2C using default pins (SDA/SCL default)
    MPU9250();

    // Constructor: I2C providing SDA and SCL pins
    MPU9250(uint8_t sdaPin, uint8_t sclPin);

    // Constructor: SPI using hardware default pins (takes only CS)
    explicit MPU9250(uint8_t csPin);

    // Constructor: SPI providing all pins (SCK, MISO, MOSI, CS)
    MPU9250(uint8_t sckPin, uint8_t misoPin, uint8_t mosiPin, uint8_t csPin);

    // Initialize device; returns true if CHIP_ID matches
    // For I2C: optionally specify I2C address (default 0x68), then chip ID
    bool begin(uint8_t i2cAddr = MPU9250_I2C_ADDR, uint8_t chipId = MPU9250_CHIP_ID);

    // Configuration functions
    void setAccelerometerRange(uint8_t range);
    void setGyroRange(uint8_t range);
    void setFilterBandwidth(uint8_t bandwidth);
    void setGyroFilterBandwidth(uint8_t bandwidth);
    void setAccelFilterBandwidth(uint8_t bandwidth);
    void setSampleRateDivider(uint8_t divider);

    // Read accelerometer data
    AccelData readAccel();
    void readAccel(float &x, float &y, float &z);

    // Read gyroscope data
    GyroData readGyro();
    void readGyro(float &x, float &y, float &z);

private:
    // Communication type flags
    bool _useI2C = false;
    bool _useHardwareSPI = true;

    // I2C configuration
    uint8_t _i2cAddress = MPU9250_I2C_ADDR;
    uint8_t _pinSDA = 0xFF;
    uint8_t _pinSCL = 0xFF;

    // SPI pin configuration
    uint8_t _pinCS = 0xFF;
    uint8_t _pinSCK = 0xFF;
    uint8_t _pinMISO = 0xFF;
    uint8_t _pinMOSI = 0xFF;

    // Sensor configuration
    uint8_t _chipId = 0;
    float _accelScale = 16384.0f;  // LSB/g for ±2g range
    float _gyroScale = 131.0f;     // LSB/°/s for ±250°/s range

    // Internal helpers
    uint8_t spiTransfer(uint8_t value);
    void select();
    void deselect();

    // Register access
    uint8_t read8(uint8_t reg);
    void write8(uint8_t reg, uint8_t value);
    int16_t read16(uint8_t reg);
    void readRegisters(uint8_t reg, uint8_t *buffer, uint8_t length);
};