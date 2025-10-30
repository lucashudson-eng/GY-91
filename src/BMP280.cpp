#include "BMP280.h"

// Constructor: SPI using hardware default pins (CS only)
BMP280::BMP280(uint8_t csPin) : useHardwareSPI(true), pinCS(csPin) {}

// Constructor: SPI providing all pins
BMP280::BMP280(uint8_t sckPin, uint8_t misoPin, uint8_t mosiPin, uint8_t csPin)
    : useHardwareSPI(false), pinCS(csPin), pinSCK(sckPin), pinMISO(misoPin), pinMOSI(mosiPin) {}

bool BMP280::begin() {
    if (useHardwareSPI) {
        SPI.begin();
        pinMode(pinCS, OUTPUT);
        deselect();
    } else {
        // Basic software SPI (bit-bang) setup
        pinMode(pinCS, OUTPUT);
        pinMode(pinSCK, OUTPUT);
        pinMode(pinMOSI, OUTPUT);
        pinMode(pinMISO, INPUT);
        deselect();
        digitalWrite(pinSCK, LOW);
        digitalWrite(pinMOSI, LOW);
    }

    // Check BMP280 CHIP_ID
    uint8_t id = readRegister(BMP280_REG_CHIP_ID);
    return id == BMP280_CHIP_ID;
}

void BMP280::select() {
    digitalWrite(pinCS, LOW);
}

void BMP280::deselect() {
    digitalWrite(pinCS, HIGH);
}

uint8_t BMP280::spiTransfer(uint8_t value) {
    if (useHardwareSPI) {
        return SPI.transfer(value);
    }

    // Software SPI (mode 0: CPOL=0, CPHA=0)
    uint8_t received = 0;
    for (uint8_t i = 0; i < 8; i++) {
        // Send MSB first
        digitalWrite(pinMOSI, (value & 0x80) ? HIGH : LOW);
        value <<= 1;

        // Clock rising edge
        digitalWrite(pinSCK, HIGH);

        // Read MISO on clock high
        received <<= 1;
        if (digitalRead(pinMISO)) {
            received |= 0x01;
        }

        // Clock falling edge
        digitalWrite(pinSCK, LOW);
    }
    return received;
}

// Register read (SPI): address with read bit (MSB) = 1
uint8_t BMP280::readRegister(uint8_t reg) {
    // For BMP280: read => reg | 0x80
    uint8_t value = 0;
    if (useHardwareSPI) {
        static const SPISettings settings(8000000, MSBFIRST, SPI_MODE0);
        SPI.beginTransaction(settings);
        select();
        SPI.transfer(reg | 0x80);
        value = SPI.transfer(0x00);
        deselect();
        SPI.endTransaction();
        return value;
    }

    select();
    spiTransfer(reg | 0x80);
    value = spiTransfer(0x00);
    deselect();
    return value;
}

// Register write (SPI): address with write bit (MSB) = 0
void BMP280::writeRegister(uint8_t reg, uint8_t value) {
    if (useHardwareSPI) {
        static const SPISettings settings(8000000, MSBFIRST, SPI_MODE0);
        SPI.beginTransaction(settings);
        select();
        SPI.transfer(reg & 0x7F);
        SPI.transfer(value);
        deselect();
        SPI.endTransaction();
        return;
    }

    select();
    spiTransfer(reg & 0x7F);
    spiTransfer(value);
    deselect();
}