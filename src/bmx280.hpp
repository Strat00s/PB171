#pragma once

#include <avr/io.h>
#include "SPIClass.hpp"
#include "digitalIO.hpp"
#include "SPIClass.hpp"


//TODO error enum/defines

//rw bit value
#define BMX280_READ  1
#define BMX280_WRITE 0
/*----(REGISTER CONFIGURATION VALUES)----*/
//device IDs
#define BMP280_SAMPLE_ID      0x56
#define BMP280_SAMPLE2_ID     0x57
#define BMP280_ID             0x58
#define BME280_ID             0x60

//sensor modes
#define BMX280_SLEEP          0b00000000
#define BMX280_FORCED         0b00000001  //and 0x02
#define BMX280_NORMAL         0b00000011

//humidity oversampling (BME280 only)
#define BME280_HUM_OVER_SKIP  0b00000000
#define BME280_HUM_OVERx1     0b00000001
#define BME280_HUM_OVERx2     0b00000010
#define BME280_HUM_OVERx4     0b00000011
#define BME280_HUM_OVERx8     0b00000100
#define BME280_HUM_OVERx16    0b00000101
//temperature oversampling
#define BMX280_TEMP_OVER_SKIP 0b00000000
#define BMX280_TEMP_OVERx1    0b00100000
#define BMX280_TEMP_OVERx2    0b01000000
#define BMX280_TEMP_OVERx4    0b01100000
#define BMX280_TEMP_OVERx8    0b10000000
#define BMX280_TEMP_OVERx16   0b10100000
//pressure oversampling
#define BMX280_PRES_OVER_SKIP 0b00000000
#define BMX280_PRES_OVERx1    0b00000100
#define BMX280_PRES_OVERx2    0b00001000
#define BMX280_PRES_OVERx4    0b00001100
#define BMX280_PRES_OVERx8    0b00010000
#define BMX280_PRES_OVERx16   0b00010100

//standby in ms
#define BMX280_STANDBY__5     0b00000000
#define BMX280_STANDBY_62_5   0b00100000
#define BMX280_STANDBY_125    0b01000000
#define BMX280_STANDBY_250    0b01100000
#define BMX280_STANDBY_500    0b10000000
#define BMX280_STANDBY_1000   0b10100000
#define BMP280_STANDBY_2000   0b11000000
#define BMP280_STANDBY_4000   0b11100000
#define BME280_STANDBY_10     0b11000000
#define BME280_STANDBY_20     0b11100000

//IIR filter
#define BMX280_IIR_FILTER_OFF 0b00000000
#define BMX280_IIR_FILTER_2   0b00000100
#define BMX280_IIR_FILTER_4   0b00001000
#define BMX280_IIR_FILTER_8   0b00001100
#define BMX280_IIR_FILTER_16  0b00010000

//3wire spi
#define BMX280_3WIRE_ENABLE   0b00000001
#define BMX280_3WIRE_DISABLE  0b00000000


/*----(REGISTERS)----*/
//control and status
#define BMX280_ID_REG         0xD0
#define BMX280_RESET_REG      0xE0
#define BMX280_STATUS_REG     0xF3
#define BMX280_CTRL_MEAS_REG  0xF4
#define BMX280_CONFIG_REG     0XF5
//BME280 only
#define BME280_CTRL_HUM_REG   0xF2

//pressure
#define BMX280_PRESS_MSB_REG  0xF7
#define BMX280_PRESS_LSB_REG  0xF8
#define BMX280_PRESS_XLSB_REG 0xF9

//temperature
#define BMX280_TEMP_MSB_REG   0xFA
#define BMX280_TEMP_LSB_REG   0xFB
#define BMX280_TEMP_XLSB_REG  0xFC

//humidity (BME280 only)
#define BME280_HUM_MSB_REG    0xFD
#define BME280_HUM_LSB_REG    0xFE

//calibration registers
#define BMX280_CALIB00_REG    0x88
#define BMX280_CALIB01_REG    0x89
#define BMX280_CALIB02_REG    0x8A
#define BMX280_CALIB03_REG    0x8B
#define BMX280_CALIB04_REG    0x8C
#define BMX280_CALIB05_REG    0x8D
#define BMX280_CALIB06_REG    0x8E
#define BMX280_CALIB07_REG    0x8F
#define BMX280_CALIB08_REG    0x90
#define BMX280_CALIB09_REG    0x91
#define BMX280_CALIB10_REG    0x92
#define BMX280_CALIB11_REG    0x93
#define BMX280_CALIB12_REG    0x94
#define BMX280_CALIB13_REG    0x95
#define BMX280_CALIB14_REG    0x96
#define BMX280_CALIB15_REG    0x97
#define BMX280_CALIB16_REG    0x98
#define BMX280_CALIB17_REG    0x99
#define BMX280_CALIB18_REG    0x9A
#define BMX280_CALIB19_REG    0x9B
#define BMX280_CALIB20_REG    0x9C
#define BMX280_CALIB21_REG    0x9D
#define BMX280_CALIB22_REG    0x9E
#define BMX280_CALIB23_REG    0x9F
#define BMX280_CALIB24_REG    0xA0
#define BMX280_CALIB25_REG    0xA1
//BME280 only calibration registers
#define BME280_CALIB26_REG    0xE1
#define BME280_CALIB27_REG    0xE2
#define BME280_CALIB28_REG    0xE3
#define BME280_CALIB29_REG    0xE4
#define BME280_CALIB30_REG    0xE5
#define BME280_CALIB31_REG    0xE6
#define BME280_CALIB32_REG    0xE7
#define BME280_CALIB33_REG    0xE8
#define BME280_CALIB34_REG    0xE9
#define BME280_CALIB35_REG    0xEA
#define BME280_CALIB36_REG    0xEB
#define BME280_CALIB37_REG    0xEC
#define BME280_CALIB38_REG    0xED
#define BME280_CALIB39_REG    0xEE
#define BME280_CALIB40_REG    0xEF
#define BME280_CALIB41_REG    0xF0


class BMX280 {
private:
    uint8_t chip_id;
    uint8_t temperature[3];
    uint8_t pressure[3];
    uint8_t humidity[2];

    uint8_t calib_regs[42]; //26 for temperature + pressure, 16 for humidity

    uint8_t cs;
    SPIClass *spi;

    void measure();

public:
    BMX280(uint8_t cs);
    BMX280(SPIClass *spi, uint8_t cs);
    ~BMX280(){};

    uint8_t readRegister(uint8_t addr);
    void readRegisterBurst(uint8_t addr, uint8_t *data, uint8_t length);
    void writeRegister(uint8_t addr, uint8_t data);
    void setRegister(uint8_t addr, uint8_t data, uint8_t mask_lsb = 0, uint8_t mask_msb = 7);

    uint8_t init();
    uint8_t init(SPIClass *spi);

    uint8_t getVersion();
    uint8_t getStatus();

    void setMode(uint8_t mode);
    void setTemperatureOversampling(uint8_t oversampling);
    void setPressureOversampling(uint8_t oversampling);
    void setHumidityOversampling(uint8_t oversampling);
    void setIIRFilter(uint8_t filter);
    void setStandby(uint8_t standby);
    void set3WireSPI(uint8_t enable);


    /** @brief Calculate pressure
     * 
     * @param raw_pressure 
     * @return uint32_t result in Q24.8 notation
     */
    uint32_t calculatePressure(uint8_t *raw_pressure);
    
    /** @brief Calculate temperature
     * 
     * @param raw_temperature 
     * @return int16_t result in Q8.8 notation
     */
    int16_t calculateTemperature(uint8_t *raw_temperature);
    
    /** @brief Calculate humidity
     * 
     * @param raw_humidity 
     * @return uint16_t result in Q8.8 notation
     */
    uint16_t calculateHumidity(uint8_t *raw_humidity);

    /** @brief Measure and calculate pressure, temperature and humidity (if BME280).
     * Forces measurement if chip is in sleep mode.
     * 
     * @param pressure variable to store calculated pressure
     * @param temperature variable to store calculated temperature
     * @param humidity variable to store calculated humidity (if BME280)
     */
    void getAll(uint32_t *pressure, uint16_t *temperature, uint16_t *humidity = nullptr);


    /**
     * @brief 
     * 
     * @param data array for storing raw data. Must be at least 3 bytes long
     */
    void getPressureRaw(uint8_t *data);

    /** @brief 
     * 
     * @param data array for storing raw data. Must be at least 3 bytes long
     */
    void getTemperatureRaw(uint8_t *data);

    /** @brief 
     * 
     * @param data array for storing raw data. Must be at least 2 bytes long
     */
    void getHumidityRaw(uint8_t *data);

    /** @brief Retrieves all possible raw data for specific chip
     * If chip is in normal mode, copyies current register value
     * If chip is in sleep mode, forces one time measurement
     * 
     * @param data pointer to an array of at least 8 bytes 
     */
    void getAllRaw(uint8_t *data);
};

