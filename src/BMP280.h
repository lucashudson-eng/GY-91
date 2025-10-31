#pragma once

#include <Arduino.h>
#include <SPI.h>

// IDs and basic registers
#define BMP280_CHIP_ID        0x58
#define BMP280_REG_CHIP_ID     0xD0

// Calibration data registers
#define BMP280_REG_DIG_T1      0x88
#define BMP280_REG_DIG_T2      0x8A
#define BMP280_REG_DIG_T3      0x8C
#define BMP280_REG_DIG_P1      0x8E
#define BMP280_REG_DIG_P2      0x90
#define BMP280_REG_DIG_P3      0x92
#define BMP280_REG_DIG_P4      0x94
#define BMP280_REG_DIG_P5      0x96
#define BMP280_REG_DIG_P6      0x98
#define BMP280_REG_DIG_P7      0x9A
#define BMP280_REG_DIG_P8      0x9C
#define BMP280_REG_DIG_P9      0x9E

// Control and status registers
#define BMP280_REG_CTRL_MEAS   0xF4
#define BMP280_REG_CONFIG      0xF5
#define BMP280_REG_STATUS      0xF3
#define BMP280_REG_RESET       0xE0

// Data registers
#define BMP280_REG_PRESS_MSB   0xF7
#define BMP280_REG_PRESS_LSB   0xF8
#define BMP280_REG_PRESS_XLSB  0xF9
#define BMP280_REG_TEMP_MSB    0xFA
#define BMP280_REG_TEMP_LSB    0xFB
#define BMP280_REG_TEMP_XLSB   0xFC

// Oversampling settings (raw values: 0x00-0x05)
#define BMP280_SAMPLING_NONE  0x00
#define BMP280_SAMPLING_X1    0x01
#define BMP280_SAMPLING_X2    0x02
#define BMP280_SAMPLING_X4    0x03
#define BMP280_SAMPLING_X8    0x04
#define BMP280_SAMPLING_X16   0x05

// Power modes
#define BMP280_MODE_SLEEP      0x00
#define BMP280_MODE_FORCED     0x01
#define BMP280_MODE_NORMAL     0x03
#define BMP280_MODE_SOFT_RESET 0xB6

// Filtering levels
#define BMP280_FILTER_OFF  0x00
#define BMP280_FILTER_X2   0x01
#define BMP280_FILTER_X4   0x02
#define BMP280_FILTER_X8   0x03
#define BMP280_FILTER_X16  0x04

// Standby duration (in ms)
#define BMP280_STANDBY_MS_1    0x00
#define BMP280_STANDBY_MS_63   0x01
#define BMP280_STANDBY_MS_125  0x02
#define BMP280_STANDBY_MS_250  0x03
#define BMP280_STANDBY_MS_500  0x04
#define BMP280_STANDBY_MS_1000 0x05
#define BMP280_STANDBY_MS_2000 0x06
#define BMP280_STANDBY_MS_4000 0x07

/*!
 *  @brief  BMP280 barometric pressure and temperature sensor driver
 *  
 *  This class provides an interface to communicate with the BMP280 sensor
 *  via SPI. The BMP280 is a low-power digital barometric pressure and
 *  temperature sensor from Bosch Sensortec.
 */
class BMP280 {
public:
    /*!
     *  @brief  Constructor for hardware SPI
     *  
     *  Initializes the BMP280 with hardware SPI interface using default
     *  hardware SPI pins (MOSI, MISO, SCK). Only requires CS pin.
     *  
     *  @param  csPin  Chip select pin number
     */
    explicit BMP280(uint8_t csPin);

    /*!
     *  @brief  Constructor for software SPI
     *  
     *  Initializes the BMP280 with software SPI (bit-bang) using custom
     *  pin assignments for all SPI pins.
     *  
     *  @param  sckPin   Serial clock pin number
     *  @param  misoPin  Master in slave out pin number
     *  @param  mosiPin  Master out slave in pin number
     *  @param  csPin    Chip select pin number
     */
    BMP280(uint8_t sckPin, uint8_t misoPin, uint8_t mosiPin, uint8_t csPin);

    /*!
     *  @brief  Initialize the BMP280 sensor
     *  
     *  Sets up SPI communication, verifies chip ID, reads calibration
     *  data, and configures the sensor for normal operation mode.
     *  
     *  @param  chipId  Expected chip ID (default: 0x58 for BMP280)
     *  @return true if initialization successful, false if chip ID mismatch
     */
    bool begin(uint8_t chipId = BMP280_CHIP_ID);

    /*!
     *  @brief  Read temperature from sensor
     *  
     *  Reads the 24-bit temperature data from the sensor and applies
     *  compensation using calibration coefficients.
     *  
     *  @return Temperature in degrees Celsius
     */
    float readTemperature();

    /*!
     *  @brief  Read pressure from sensor
     *  
     *  Reads the 24-bit pressure data from the sensor and applies
     *  compensation using calibration coefficients. Temperature must
     *  be read first to calculate compensation values.
     *  
     *  @return Pressure in Pascals (Pa)
     */
    float readPressure();

    /*!
     *  @brief  Calculate altitude from pressure reading
     *  
     *  Uses the barometric formula to calculate altitude based on
     *  current pressure and sea level reference pressure.
     *  
     *  @param  seaLevelPressure  Sea level reference pressure in Pascals
     *                            (default: 101325 Pa = standard atmosphere)
     *  @return Altitude in meters
     */
    float readAltitude(float seaLevelPressure = 101325.0);

    /*!
     *  @brief  Calculate sea level pressure from altitude and atmospheric pressure
     *  
     *  Converts current atmospheric pressure at a given altitude to
     *  equivalent sea level pressure. Equation from BMP180 datasheet.
     *  
     *  @param  altitude       Altitude in meters
     *  @param  atmospheric    Current atmospheric pressure in Pascals
     *  @return Sea level pressure in Pascals
     */
    float seaLevelForAltitude(float altitude, float atmospheric);

    /*!
     *  @brief  Calculate water boiling point at given pressure
     *  
     *  Uses Magnus formula to calculate the boiling point of water
     *  at a specific atmospheric pressure.
     *  
     *  @param  pressure  Atmospheric pressure in Pascals
     *  @return Water boiling point temperature in degrees Celsius
     */
    float waterBoilingPoint(float pressure);

    /*!
     *  @brief  Configure sensor operating parameters
     *  
     *  Sets the power mode, oversampling rates for temperature and pressure,
     *  IIR filter settings, and standby duration for normal mode.
     *  
     *  @param  mode          Power mode (SLEEP, FORCED, or NORMAL)
     *  @param  tempSampling  Temperature oversampling rate (X1 to X16)
     *  @param  pressSampling Pressure oversampling rate (X1 to X16)
     *  @param  filter        IIR filter coefficient (OFF, X2, X4, X8, X16)
     *  @param  duration      Standby duration for normal mode (1ms to 4000ms)
     */
    void configureSensor(uint8_t mode = BMP280_MODE_NORMAL,
                         uint8_t tempSampling = BMP280_SAMPLING_X16,
                         uint8_t pressSampling = BMP280_SAMPLING_X16,
                         uint8_t filter = BMP280_FILTER_OFF,
                         uint8_t duration = BMP280_STANDBY_MS_1);

    /*!
     *  @brief  Read status register
     *  
     *  Returns the status register value which indicates measurement
     *  state and calibration data update status.
     *  
     *  @return Status register value with possible returns:
     *          - 0x00 (0): Inactive (no measurement, no update)
     *          - 0x01 (1): Updating calibration coefficients
     *          - 0x08 (8): Performing measurement
     *          - 0x09 (9): Measuring and updating coefficients simultaneously
     */
    uint8_t getStatus();

    /*!
     *  @brief  Get stored chip ID
     *  
     *  Returns the chip ID that was read during initialization.
     *  
     *  @return Chip ID value (0x58 for BMP280)
     */
    uint8_t getChipID();

    /*!
     *  @brief  Perform soft reset of the sensor
     *  
     *  Sends soft reset command to the sensor, restoring all registers
     *  to their default values. Waits 5ms for reset to complete.
     */
    void reset();

private:
    // Pin configuration
    bool _useHardwareSPI = true;
    uint8_t _pinCS = 0xFF;
    uint8_t _pinSCK = 0xFF;
    uint8_t _pinMISO = 0xFF;
    uint8_t _pinMOSI = 0xFF;

    // Chip ID
    uint8_t _chipId = 0;

    // Calibration data
    uint16_t _dig_T1 = 0;
    int16_t _dig_T2 = 0;
    int16_t _dig_T3 = 0;
    uint16_t _dig_P1 = 0;
    int16_t _dig_P2 = 0;
    int16_t _dig_P3 = 0;
    int16_t _dig_P4 = 0;
    int16_t _dig_P5 = 0;
    int16_t _dig_P6 = 0;
    int16_t _dig_P7 = 0;
    int16_t _dig_P8 = 0;
    int16_t _dig_P9 = 0;

    // Internal temperature (used for pressure compensation)
    int32_t _t_fine = 0;

    // Internal helpers
    uint8_t spiTransfer(uint8_t value);
    void select();
    void deselect();

    // SPI register access
    uint8_t read8(uint8_t reg);
    void write8(uint8_t reg, uint8_t value);
    void readBytes(uint8_t reg, uint8_t* data, uint8_t length);
    uint32_t read24(uint8_t reg);

    // Calibration
    void readCalibrationData();
};