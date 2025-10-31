#include "BMP280.h"
#include <math.h>

BMP280::BMP280(uint8_t csPin) : _useHardwareSPI(true), _pinCS(csPin) {}

BMP280::BMP280(uint8_t sckPin, uint8_t misoPin, uint8_t mosiPin, uint8_t csPin)
    : _useHardwareSPI(false), _pinCS(csPin), _pinSCK(sckPin), _pinMISO(misoPin), _pinMOSI(mosiPin) {}

/*!
 *  @brief  Initialize the BMP280 sensor
 *  
 *  Sets up SPI communication (hardware or software), verifies chip ID,
 *  reads calibration data from sensor, and configures sensor for normal
 *  operation mode with default settings.
 *  
 *  @param  chipId  Expected chip ID (default: 0x58 for BMP280)
 *  @return true if initialization successful and chip ID matches,
 *          false if chip ID mismatch or initialization failed
 */
bool BMP280::begin(uint8_t chipId) {
    if (_useHardwareSPI) {
        SPI.begin();
        pinMode(_pinCS, OUTPUT);
        deselect();
    } else {
        // Basic software SPI (bit-bang) setup
        pinMode(_pinCS, OUTPUT);
        pinMode(_pinSCK, OUTPUT);
        pinMode(_pinMOSI, OUTPUT);
        pinMode(_pinMISO, INPUT);
        deselect();
        digitalWrite(_pinSCK, LOW);
        digitalWrite(_pinMOSI, LOW);
    }

    // Check BMP280 CHIP_ID
    _chipId = read8(BMP280_REG_CHIP_ID);
    if (_chipId != chipId) {
        return false;
    }

    // Read calibration data
    readCalibrationData();

    // Configure sensor for normal mode
    configureSensor();

    return true;
}

void BMP280::select() {
    digitalWrite(_pinCS, LOW);
}

void BMP280::deselect() {
    digitalWrite(_pinCS, HIGH);
}

uint8_t BMP280::spiTransfer(uint8_t value) {
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

// Read 8-bit register (SPI): address with read bit (MSB) = 1
uint8_t BMP280::read8(uint8_t reg) {
    // For BMP280: read => reg | 0x80
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

    select();
    spiTransfer(reg | 0x80);
    value = spiTransfer(0x00);
    deselect();
    return value;
}

// Read 24-bit value (3 bytes) - returns raw 24-bit value (MSB, LSB, XLSB)
uint32_t BMP280::read24(uint8_t reg) {
    uint8_t data[3];
    readBytes(reg, data, 3);
    // BMP280 stores data as MSB, LSB, XLSB (24 bits total, but only 20 bits used)
    return ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | (uint32_t)data[2];
}

// Write 8-bit register (SPI): address with write bit (MSB) = 0
void BMP280::write8(uint8_t reg, uint8_t value) {
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

    select();
    spiTransfer(reg & 0x7F);
    spiTransfer(value);
    deselect();
}

// Read multiple bytes starting from register address
void BMP280::readBytes(uint8_t reg, uint8_t* data, uint8_t length) {
    if (_useHardwareSPI) {
        static const SPISettings settings(8000000, MSBFIRST, SPI_MODE0);
        SPI.beginTransaction(settings);
        select();
        SPI.transfer(reg | 0x80);
        for (uint8_t i = 0; i < length; i++) {
            data[i] = SPI.transfer(0x00);
        }
        deselect();
        SPI.endTransaction();
        return;
    }

    select();
    spiTransfer(reg | 0x80);
    for (uint8_t i = 0; i < length; i++) {
        data[i] = spiTransfer(0x00);
    }
    deselect();
}

// Read calibration data from BMP280
void BMP280::readCalibrationData() {
    uint8_t calibData[24];
    readBytes(BMP280_REG_DIG_T1, calibData, 24);

    // Temperature calibration (little endian)
    _dig_T1 = (uint16_t)calibData[0] | ((uint16_t)calibData[1] << 8);
    _dig_T2 = (int16_t)calibData[2] | ((int16_t)calibData[3] << 8);
    _dig_T3 = (int16_t)calibData[4] | ((int16_t)calibData[5] << 8);

    // Pressure calibration (little endian)
    _dig_P1 = (uint16_t)calibData[6] | ((uint16_t)calibData[7] << 8);
    _dig_P2 = (int16_t)calibData[8] | ((int16_t)calibData[9] << 8);
    _dig_P3 = (int16_t)calibData[10] | ((int16_t)calibData[11] << 8);
    _dig_P4 = (int16_t)calibData[12] | ((int16_t)calibData[13] << 8);
    _dig_P5 = (int16_t)calibData[14] | ((int16_t)calibData[15] << 8);
    _dig_P6 = (int16_t)calibData[16] | ((int16_t)calibData[17] << 8);
    _dig_P7 = (int16_t)calibData[18] | ((int16_t)calibData[19] << 8);
    _dig_P8 = (int16_t)calibData[20] | ((int16_t)calibData[21] << 8);
    _dig_P9 = (int16_t)calibData[22] | ((int16_t)calibData[23] << 8);
}

/*!
 *  @brief  Configure sensor operating parameters
 *  
 *  Configures the BMP280 sensor with specified power mode, oversampling
 *  rates, IIR filter settings, and standby duration. Writes to CONFIG
 *  and CTRL_MEAS registers. Waits 100ms for first measurement to complete.
 *  
 *  @param  mode          Power mode (SLEEP, FORCED, or NORMAL)
 *  @param  tempSampling  Temperature oversampling rate (X1 to X16)
 *  @param  pressSampling Pressure oversampling rate (X1 to X16)
 *  @param  filter        IIR filter coefficient (OFF, X2, X4, X8, X16)
 *  @param  duration      Standby duration for normal mode (1ms to 4000ms)
 */
void BMP280::configureSensor(uint8_t mode, uint8_t tempSampling, uint8_t pressSampling, uint8_t filter, uint8_t duration) {
    // Set config register
    // Bits [7:5] - t_sb (standby duration)
    // Bits [4:2] - filter
    // Bits [1:0] - spi3w_en (always 0 for 4-wire SPI)
    uint8_t config = ((duration << 5) | (filter << 2));
    write8(BMP280_REG_CONFIG, config);

    // Set control register (CTRL_MEAS)
    // Bits [7:5] - osrs_t (temperature oversampling)
    // Bits [4:2] - osrs_p (pressure oversampling)
    // Bits [1:0] - mode
    uint8_t ctrl = ((tempSampling << 5) | (pressSampling << 2) | mode);
    write8(BMP280_REG_CTRL_MEAS, ctrl);

    // Wait for first measurement
    delay(100);
}

/*!
 *  @brief  Read status register
 *  
 *  Reads and returns the status register (0xF3) value which indicates
 *  measurement state and calibration data update status.
 *  
 *  @return Status register value with possible returns:
 *          - 0x00 (0): Inactive (no measurement, no update)
 *          - 0x01 (1): Updating calibration coefficients
 *          - 0x08 (8): Performing measurement
 *          - 0x09 (9): Measuring and updating coefficients simultaneously
 */
uint8_t BMP280::getStatus() {
    return read8(BMP280_REG_STATUS);
}

/*!
 *  @brief  Get stored chip ID
 *  
 *  Returns the chip ID value that was read and stored during initialization
 *  in the begin() function. Does not perform a new register read.
 *  
 *  @return Chip ID value (0x58 for BMP280), or 0 if not initialized
 */
uint8_t BMP280::getChipID() {
    return _chipId;
}

/*!
 *  @brief  Perform soft reset of the sensor
 *  
 *  Sends soft reset command (0xB6) to the reset register (0xE0), which
 *  restores all registers to their default values and returns the sensor
 *  to sleep mode. Waits 5ms for reset to complete (minimum required is 2ms).
 *  
 *  After reset, the sensor must be re-initialized with begin() before use.
 */
void BMP280::reset() {
    write8(BMP280_REG_RESET, BMP280_MODE_SOFT_RESET);
    delay(5); // Wait for reset to complete (minimum 2ms, using 5ms for safety)
}

/*!
 *  @brief  Read temperature from sensor
 *  
 *  Reads the 24-bit temperature data from the sensor, extracts the 20-bit
 *  ADC value, and applies temperature compensation using calibration
 *  coefficients according to the BMP280 datasheet formula.
 *  
 *  @return Temperature in degrees Celsius
 */
float BMP280::readTemperature() {
    // Read 24-bit temperature data, extract 20-bit value (shift right 4 bits)
    int32_t adc_T = (int32_t)(read24(BMP280_REG_TEMP_MSB) >> 4);

    // Temperature compensation formula from BMP280 datasheet
    int32_t var1, var2;
    var1 = ((((adc_T >> 3) - ((int32_t)_dig_T1 << 1))) * ((int32_t)_dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)_dig_T1)) * ((adc_T >> 4) - ((int32_t)_dig_T1))) >> 12) * ((int32_t)_dig_T3)) >> 14;

    _t_fine = var1 + var2;
    float temperature = (_t_fine * 5 + 128) >> 8;
    return temperature / 100.0;
}

/*!
 *  @brief  Read pressure from sensor
 *  
 *  Reads the 24-bit pressure data from the sensor, extracts the 20-bit
 *  ADC value, and applies pressure compensation using calibration
 *  coefficients and temperature fine value. Automatically reads temperature
 *  first to calculate compensation values.
 *  
 *  @return Pressure in Pascals (Pa)
 */
float BMP280::readPressure() {
    // Temperature must be read first to get t_fine for compensation
    readTemperature();

    // Read 24-bit pressure data, extract 20-bit value (shift right 4 bits)
    int32_t adc_P = (int32_t)(read24(BMP280_REG_PRESS_MSB) >> 4);

    // Pressure compensation formula from BMP280 datasheet
    int64_t var1, var2, p;
    var1 = ((int64_t)_t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)_dig_P6;
    var2 = var2 + ((var1 * (int64_t)_dig_P5) << 17);
    var2 = var2 + (((int64_t)_dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)_dig_P3) >> 8) + ((var1 * (int64_t)_dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)_dig_P1) >> 33;

    if (var1 == 0) {
        return 0.0; // Avoid division by zero
    }

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)_dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)_dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)_dig_P7) << 4);

    return (float)p / 256.0;
}

/*!
 *  @brief  Calculate altitude from pressure reading
 *  
 *  Uses the barometric formula to calculate altitude based on current
 *  pressure reading and sea level reference pressure. Reads current
 *  pressure from sensor automatically.
 *  
 *  Formula: altitude = 44330 * (1 - (P/P0)^(1/5.255))
 *  
 *  @param  seaLevelPressure  Sea level reference pressure in Pascals
 *                            (default: 101325 Pa = standard atmosphere)
 *  @return Altitude in meters, or 0.0 if pressure reading failed
 */
float BMP280::readAltitude(float seaLevelPressure) {
    float pressure = readPressure();
    if (pressure == 0.0) {
        return 0.0;
    }
    // Barometric formula: altitude = 44330 * (1 - (P/P0)^(1/5.255))
    float altitude = 44330.0 * (1.0 - pow(pressure / seaLevelPressure, 0.1903));
    return altitude;
}

/*!
 *  @brief  Calculate sea level pressure from altitude and atmospheric pressure
 *  
 *  Converts current atmospheric pressure measured at a given altitude to
 *  equivalent sea level pressure. This is the inverse operation of
 *  readAltitude(). Equation from BMP180 datasheet (page 17).
 *  
 *  Note: Using the equation from Wikipedia can give bad results at high altitude.
 *  
 *  @param  altitude       Altitude in meters where pressure was measured
 *  @param  atmospheric    Current atmospheric pressure in Pascals
 *  @return Sea level pressure in Pascals
 */
float BMP280::seaLevelForAltitude(float altitude, float atmospheric) {
    return atmospheric / pow(1.0 - (altitude / 44330.0), 5.255);
}

/*!
 *  @brief  Calculate water boiling point at given pressure
 *  
 *  Uses Magnus formula to calculate the boiling point of water at a
 *  specific atmospheric pressure. Pressure is converted from Pascals
 *  to hectopascals (hPa) internally for the calculation.
 *  
 *  @param  pressure  Atmospheric pressure in Pascals
 *  @return Water boiling point temperature in degrees Celsius
 */
float BMP280::waterBoilingPoint(float pressure) {
    // Convert Pascals to hPa
    float pressure_hPa = pressure / 100.0;
    return (234.175 * log(pressure_hPa / 6.1078)) / (17.08085 - log(pressure_hPa / 6.1078));
}