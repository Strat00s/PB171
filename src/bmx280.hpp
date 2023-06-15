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

#define BMX280_RESET          0xB6

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

//calibration registers (26)
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
//BME280 only calibration registers (16)
#define BME280_CALIB26_REG    0xE1
#define BME280_CALIB27_REG    0xE2
#define BME280_CALIB28_REG    0xE3
#define BME280_CALIB29_REG    0xE4
#define BME280_CALIB30_REG    0xE5
#define BME280_CALIB31_REG    0xE6
#define BME280_CALIB32_REG    0xE7
//unused (9)
#define BME280_CALIB33_REG    0xE8
#define BME280_CALIB34_REG    0xE9
#define BME280_CALIB35_REG    0xEA
#define BME280_CALIB36_REG    0xEB
#define BME280_CALIB37_REG    0xEC
#define BME280_CALIB38_REG    0xED
#define BME280_CALIB39_REG    0xEE
#define BME280_CALIB40_REG    0xEF
#define BME280_CALIB41_REG    0xF0


typedef struct {
    uint16_t dig_T1;    //0x88 / 0x89      | dig_T1 [7:0] / [15:8] | unsigned shor
    int16_t  dig_T2;    //0x8A / 0x8B      | dig_T2 [7:0] / [15:8] | signed short
    int16_t  dig_T3;    //0x8C / 0x8D      | dig_T3 [7:0] / [15:8] | signed short
    uint16_t dig_P1;    //0x8E / 0x8F      | dig_P1 [7:0] / [15:8] | unsigned shor
    int16_t  dig_P2;    //0x90 / 0x91      | dig_P2 [7:0] / [15:8] | signed short
    int16_t  dig_P3;    //0x92 / 0x93      | dig_P3 [7:0] / [15:8] | signed short
    int16_t  dig_P4;    //0x94 / 0x95      | dig_P4 [7:0] / [15:8] | signed short
    int16_t  dig_P5;    //0x96 / 0x97      | dig_P5 [7:0] / [15:8] | signed short
    int16_t  dig_P6;    //0x98 / 0x99      | dig_P6 [7:0] / [15:8] | signed short
    int16_t  dig_P7;    //0x9A / 0x9B      | dig_P7 [7:0] / [15:8] | signed short
    int16_t  dig_P8;    //0x9C / 0x9D      | dig_P8 [7:0] / [15:8] | signed short
    int16_t  dig_P9;    //0x9E / 0x9F      | dig_P9 [7:0] / [15:8] | signed short

    //0xA1,0xA0 reserved in BMP280, A0 not used in BME280
    uint8_t  dig_H1;    //0xA1             | dig_H1 [7:0]          | unsigned char
    int16_t  dig_H2;    //0xE1 / 0xE2      | dig_H2 [7:0] / [15:8] | signed short
    uint8_t  dig_H3;    //0xE3             | dig_H3 [7:0]          | unsigned char
    int16_t  dig_H4;    //0xE4 / 0xE5[3:0] | dig_H4 [11:4] / [3:0] | signed short
    int16_t  dig_H5;    //0xE5[7:4] / 0xE6 | dig_H5 [3:0] / [11:4] | signed short
    int8_t   dig_H6;    //0xE7             | dig_H6                | signed char
} comp_parameters_t;


class BMX280 {
private:
    uint8_t chip_id;
    uint8_t temperature[3];
    uint8_t pressure[3];
    uint8_t humidity[2];

    //uint8_t calib_regs[42] = {0}; //26 for temperature + pressure, 16 for humidity

    uint8_t cs;
    SPIClass *spi = nullptr;

    void measure();

public:
    comp_parameters_t comp_params;

    BMX280(uint8_t cs);
    BMX280(SPIClass *spi, uint8_t cs);
    ~BMX280(){};

    uint8_t readRegister(uint8_t addr);
    void readRegisterBurst(uint8_t addr, uint8_t *data, uint8_t length);
    void writeRegister(uint8_t addr, uint8_t data);
    void setRegister(uint8_t addr, uint8_t data, uint8_t mask_lsb = 0, uint8_t mask_msb = 7);

    uint8_t init();
    uint8_t init(SPIClass *spi);

    void reset();

    uint8_t getId();
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
    uint32_t calculatePressure(uint8_t *raw_p);
    
    /** @brief Calculate temperature
     * 
     * @param raw_temperature 
     * @return int16_t result in Q8.8 notation
     */
    int32_t calculateTemperature(uint8_t *raw_t);
    
    /** @brief Calculate humidity
     * 
     * @param raw_humidity 
     * @return uint16_t result in Q8.8 notation
     */
    uint16_t calculateHumidity(uint8_t *raw_h);

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
     * If oversampling is disabled, enables x1 for everything
     * @param data pointer to an array of at least 8 bytes 
     */
    void getAllRaw(uint8_t *data);
};

