#include "MPU9250.h"

// Constructor: I2C using default pins
MPU9250::MPU9250() : _useI2C(true), _useHardwareSPI(false) {}

// Constructor: I2C providing SDA and SCL pins
MPU9250::MPU9250(uint8_t sdaPin, uint8_t sclPin)
    : _useI2C(true), _useHardwareSPI(false), _pinSDA(sdaPin), _pinSCL(sclPin) {}

// Constructor: SPI using hardware default pins (CS only)
MPU9250::MPU9250(uint8_t csPin) : _useI2C(false), _useHardwareSPI(true), _pinCS(csPin) {}

// Constructor: SPI providing all pins
MPU9250::MPU9250(uint8_t sckPin, uint8_t misoPin, uint8_t mosiPin, uint8_t csPin)
    : _useI2C(false), _useHardwareSPI(false), _pinCS(csPin), _pinSCK(sckPin), _pinMISO(misoPin), _pinMOSI(mosiPin) {}

bool MPU9250::begin(uint8_t i2cAddr, uint8_t chipId) {
    if (_useI2C) {
        // Initialize I2C
        if (_pinSDA != 0xFF && _pinSCL != 0xFF) {
            // Custom pins (for ESP32, etc.)
            Wire.begin(_pinSDA, _pinSCL);
        } else {
            // Default I2C pins
            Wire.begin();
        }
        _i2cAddress = i2cAddr;
        delay(100);
    } else if (_useHardwareSPI) {
        SPI.begin();
        pinMode(_pinCS, OUTPUT);
        deselect();
    } else {
        // Software SPI (bit-bang) setup
        pinMode(_pinCS, OUTPUT);
        pinMode(_pinSCK, OUTPUT);
        pinMode(_pinMOSI, OUTPUT);
        pinMode(_pinMISO, INPUT);
        deselect();
        digitalWrite(_pinSCK, LOW);
        digitalWrite(_pinMOSI, LOW);
    }

    // Wait for device to be ready
    delay(100);

    // Read and store CHIP_ID
    _chipId = read8(MPU9250_REG_WHO_AM_I);
    
    // Check MPU9250 CHIP_ID
    if (_chipId != chipId) {
        return false;
    }

    // Reset device
    write8(MPU9250_REG_PWR_MGMT_1, 0x80);
    delay(100);

    // Wake up device (exit sleep mode)
    write8(MPU9250_REG_PWR_MGMT_1, 0x00);
    delay(10);

    // Configure sensor settings using configuration functions
    setSampleRateDivider(0);  // 1kHz / (1 + 0) = 1kHz
    setFilterBandwidth(MPU9250_DLPF_184HZ);  // Gyroscope DLPF: 184Hz bandwidth
    setAccelFilterBandwidth(MPU9250_ACCEL_DLPF_184HZ);  // Accelerometer DLPF: 184Hz bandwidth
    setAccelerometerRange(MPU9250_ACCEL_FS_2G);  // ±2g range
    setGyroRange(MPU9250_GYRO_FS_250DPS);  // ±250°/s range

    return true;
}

void MPU9250::setAccelerometerRange(uint8_t range) {
    write8(MPU9250_REG_ACCEL_CONFIG, range);
    
    // Update scale based on range
    switch (range) {
        case MPU9250_ACCEL_FS_2G:
            _accelScale = 16384.0f;  // LSB/g
            break;
        case MPU9250_ACCEL_FS_4G:
            _accelScale = 8192.0f;   // LSB/g
            break;
        case MPU9250_ACCEL_FS_8G:
            _accelScale = 4096.0f;   // LSB/g
            break;
        case MPU9250_ACCEL_FS_16G:
            _accelScale = 2048.0f;   // LSB/g
            break;
        default:
            // Keep current scale if invalid range
            break;
    }
    delay(10);
}

void MPU9250::setGyroRange(uint8_t range) {
    write8(MPU9250_REG_GYRO_CONFIG, range);
    
    // Update scale based on range
    switch (range) {
        case MPU9250_GYRO_FS_250DPS:
            _gyroScale = 131.0f;      // LSB/°/s
            break;
        case MPU9250_GYRO_FS_500DPS:
            _gyroScale = 65.5f;      // LSB/°/s
            break;
        case MPU9250_GYRO_FS_1000DPS:
            _gyroScale = 32.8f;      // LSB/°/s
            break;
        case MPU9250_GYRO_FS_2000DPS:
            _gyroScale = 16.4f;      // LSB/°/s
            break;
        default:
            // Keep current scale if invalid range
            break;
    }
    delay(10);
}

void MPU9250::setFilterBandwidth(uint8_t bandwidth) {
    // DLPF for gyroscope (and temperature) - CONFIG register
    // Bits [2:0] are DLPF_CFG
    uint8_t config = read8(MPU9250_REG_CONFIG);
    config = (config & 0xF8) | (bandwidth & 0x07);  // Preserve upper bits, set lower 3 bits
    write8(MPU9250_REG_CONFIG, config);
    delay(10);
}

void MPU9250::setGyroFilterBandwidth(uint8_t bandwidth) {
    // Alias for setFilterBandwidth (gyroscope filter)
    setFilterBandwidth(bandwidth);
}

void MPU9250::setAccelFilterBandwidth(uint8_t bandwidth) {
    // DLPF for accelerometer - ACCEL_CONFIG_2 register
    // Bits [3:0] are A_DLPF_CFG (bits [2:0] are the actual filter setting)
    uint8_t config = read8(MPU9250_REG_ACCEL_CONFIG_2);
    config = (config & 0xF0) | (bandwidth & 0x07);  // Preserve upper bits, set lower 3 bits
    write8(MPU9250_REG_ACCEL_CONFIG_2, config);
    delay(10);
}

void MPU9250::setSampleRateDivider(uint8_t divider) {
    // Sample rate = 1kHz / (1 + divider)
    write8(MPU9250_REG_SMPLRT_DIV, divider);
    delay(10);
}

void MPU9250::select() {
    digitalWrite(_pinCS, LOW);
}

void MPU9250::deselect() {
    digitalWrite(_pinCS, HIGH);
}

uint8_t MPU9250::spiTransfer(uint8_t value) {
    if (_useHardwareSPI) {
        return SPI.transfer(value);
    }

    // Software SPI (mode 0: CPOL=0, CPHA=0)
    uint8_t received = 0;
    for (uint8_t i = 0; i < 8; i++) {
        // Send MSB first
        digitalWrite(_pinMOSI, (value & 0x80) ? HIGH : LOW);
        value <<= 1;

        // Clock rising edge
        digitalWrite(_pinSCK, HIGH);

        // Read MISO on clock high
        received <<= 1;
        if (digitalRead(_pinMISO)) {
            received |= 0x01;
        }

        // Clock falling edge
        digitalWrite(_pinSCK, LOW);
    }
    return received;
}

// Register read: SPI (address with read bit MSB = 1) or I2C
uint8_t MPU9250::read8(uint8_t reg) {
    if (_useI2C) {
        // I2C read
        Wire.beginTransmission(_i2cAddress);
        Wire.write(reg);
        Wire.endTransmission(false);
        Wire.requestFrom(_i2cAddress, (uint8_t)1);
        if (Wire.available()) {
            return Wire.read();
        }
        return 0;
    }

    // SPI read: address with read bit (MSB) = 1
    uint8_t value = 0;
    if (_useHardwareSPI) {
        static const SPISettings settings(8000000, MSBFIRST, SPI_MODE0);
        SPI.beginTransaction(settings);
        select();
        SPI.transfer(reg | 0x80);
        value = SPI.transfer(0x00);
        deselect();
        SPI.endTransaction();
        return value;
    }

    // Software SPI
    select();
    spiTransfer(reg | 0x80);
    value = spiTransfer(0x00);
    deselect();
    return value;
}

// Register write: SPI (address with write bit MSB = 0) or I2C
void MPU9250::write8(uint8_t reg, uint8_t value) {
    if (_useI2C) {
        // I2C write
        Wire.beginTransmission(_i2cAddress);
        Wire.write(reg);
        Wire.write(value);
        Wire.endTransmission();
        return;
    }

    // SPI write: address with write bit (MSB) = 0
    if (_useHardwareSPI) {
        static const SPISettings settings(8000000, MSBFIRST, SPI_MODE0);
        SPI.beginTransaction(settings);
        select();
        SPI.transfer(reg & 0x7F);
        SPI.transfer(value);
        deselect();
        SPI.endTransaction();
        return;
    }

    // Software SPI
    select();
    spiTransfer(reg & 0x7F);
    spiTransfer(value);
    deselect();
}

// Read multiple registers (optimized for I2C burst read)
void MPU9250::readRegisters(uint8_t reg, uint8_t *buffer, uint8_t length) {
    if (_useI2C) {
        // I2C burst read
        Wire.beginTransmission(_i2cAddress);
        Wire.write(reg);
        Wire.endTransmission(false);
        Wire.requestFrom(_i2cAddress, length);
        for (uint8_t i = 0; i < length && Wire.available(); i++) {
            buffer[i] = Wire.read();
        }
        return;
    }

    // SPI: read registers sequentially
    for (uint8_t i = 0; i < length; i++) {
        buffer[i] = read8(reg + i);
    }
}

// Read 16-bit register (big-endian: MSB first)
int16_t MPU9250::read16(uint8_t reg) {
    if (_useI2C) {
        // Optimized I2C burst read for 2 bytes
        uint8_t buffer[2];
        readRegisters(reg, buffer, 2);
        return (int16_t)((buffer[0] << 8) | buffer[1]);
    }

    // SPI: two separate reads
    uint8_t high = read8(reg);
    uint8_t low = read8(reg + 1);
    return (int16_t)((high << 8) | low);
}

AccelData MPU9250::readAccel() {
    AccelData accel;
    readAccel(accel.x, accel.y, accel.z);
    return accel;
}

void MPU9250::readAccel(float &x, float &y, float &z) {
    if (_useI2C) {
        // Optimized I2C burst read for all 6 bytes
        uint8_t buffer[6];
        readRegisters(MPU9250_REG_ACCEL_XOUT_H, buffer, 6);
        int16_t rawX = (int16_t)((buffer[0] << 8) | buffer[1]);
        int16_t rawY = (int16_t)((buffer[2] << 8) | buffer[3]);
        int16_t rawZ = (int16_t)((buffer[4] << 8) | buffer[5]);
        x = (float)rawX / _accelScale;
        y = (float)rawY / _accelScale;
        z = (float)rawZ / _accelScale;
    } else {
        int16_t rawX = read16(MPU9250_REG_ACCEL_XOUT_H);
        int16_t rawY = read16(MPU9250_REG_ACCEL_XOUT_H + 2);
        int16_t rawZ = read16(MPU9250_REG_ACCEL_XOUT_H + 4);
        x = (float)rawX / _accelScale;
        y = (float)rawY / _accelScale;
        z = (float)rawZ / _accelScale;
    }
}

GyroData MPU9250::readGyro() {
    GyroData gyro;
    readGyro(gyro.x, gyro.y, gyro.z);
    return gyro;
}

void MPU9250::readGyro(float &x, float &y, float &z) {
    if (_useI2C) {
        // Optimized I2C burst read for all 6 bytes
        uint8_t buffer[6];
        readRegisters(MPU9250_REG_GYRO_XOUT_H, buffer, 6);
        int16_t rawX = (int16_t)((buffer[0] << 8) | buffer[1]);
        int16_t rawY = (int16_t)((buffer[2] << 8) | buffer[3]);
        int16_t rawZ = (int16_t)((buffer[4] << 8) | buffer[5]);
        x = (float)rawX / _gyroScale;
        y = (float)rawY / _gyroScale;
        z = (float)rawZ / _gyroScale;
    } else {
        int16_t rawX = read16(MPU9250_REG_GYRO_XOUT_H);
        int16_t rawY = read16(MPU9250_REG_GYRO_XOUT_H + 2);
        int16_t rawZ = read16(MPU9250_REG_GYRO_XOUT_H + 4);
        x = (float)rawX / _gyroScale;
        y = (float)rawY / _gyroScale;
        z = (float)rawZ / _gyroScale;
    }
}