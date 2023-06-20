/**
 * @file bmx280.cpp
 * @author Lukáš Baštýř (l.bastyr@seznam.cz)
 * @brief 
 * @version 0.1
 * @date 16-06-2023
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "bmx280.hpp"
#include "digitalIO.hpp"


BMX280::BMX280(uint8_t cs) {
    this->cs = cs;
    this->chip_id = 0;

    pinMode(this->cs, OUTPUT);
    digitalWrite(this->cs, LOW);    //set SPI mode
    digitalWrite(this->cs, HIGH);
}
BMX280::BMX280(SPIClass *spi, uint8_t cs) : BMX280(cs) {
    this->spi = spi;
}

BMX280::~BMX280() {};


uint8_t BMX280::readRegister(uint8_t addr) {
    digitalWrite(this->cs, LOW);
    uint8_t reg = this->spi->readRegister(addr, BMX280_READ);
    digitalWrite(this->cs, HIGH);
    return reg;
}

void BMX280::readRegisterBurst(uint8_t addr, uint8_t *data, uint8_t length) {
    digitalWrite(this->cs, LOW);
    this->spi->readRegisterBurst(addr, data, length, BMX280_READ);
    digitalWrite(this->cs, HIGH);
}

void BMX280::writeRegister(uint8_t addr, uint8_t data) {
    digitalWrite(this->cs, LOW);
    this->spi->writeRegister(addr, data, BMX280_WRITE);
    digitalWrite(this->cs, HIGH);
}

void BMX280::setRegister(uint8_t addr, uint8_t data, uint8_t mask_lsb, uint8_t mask_msb) {
    uint8_t reg = readRegister(addr);

    uint8_t mask = ~(0xFF >> (8 - mask_lsb) | 0xFF << (mask_msb + 1));
    data = (reg & ~mask) | (data & mask);

    writeRegister(addr, data);
}


uint8_t BMX280::begin() {
    if (this->spi == nullptr)
        return 2;

    this->spi->begin();
    
    switch (getId()) {
        case BMP280_SAMPLE_ID:
        case BMP280_SAMPLE2_ID:
        case BMP280_ID: this->chip_id = BMP280_ID; break;
        case BME280_ID: this->chip_id = BME280_ID; break;
        default: this->chip_id = 0; return 1;
    }

    //store calibrations
    uint8_t c_reg[33] = {0};
    readRegisterBurst(BMX280_CALIB00_REG, c_reg, 26);
    if (this->chip_id == BME280_ID)
        readRegisterBurst(BME280_CALIB26_REG, c_reg + 26, 7);

    comp_params.dig_T1 = ((uint16_t)(c_reg[1]) << 8) | (c_reg[0]);          //0x88 / 0x89      | dig_T1 [7:0] / [15:8] | unsigned short
    comp_params.dig_T2 = ((int16_t)(c_reg[3])  << 8) | (c_reg[2]);          //0x8A / 0x8B      | dig_T2 [7:0] / [15:8] | signed short
    comp_params.dig_T3 = ((int16_t)(c_reg[5])  << 8) | (c_reg[4]);          //0x8C / 0x8D      | dig_T3 [7:0] / [15:8] | signed short
    
    comp_params.dig_P1 = ((uint16_t)(c_reg[7]) << 8) | (c_reg[6]);          //0x8E / 0x8F      | dig_P1 [7:0] / [15:8] | unsigned short
    comp_params.dig_P2 = ((int16_t)(c_reg[9])  << 8) | (c_reg[8]);          //0x90 / 0x91      | dig_P2 [7:0] / [15:8] | signed short
    comp_params.dig_P3 = ((int16_t)(c_reg[11]) << 8) | (c_reg[10]);         //0x92 / 0x93      | dig_P3 [7:0] / [15:8] | signed short
    comp_params.dig_P4 = ((int16_t)(c_reg[13]) << 8) | (c_reg[12]);         //0x94 / 0x95      | dig_P4 [7:0] / [15:8] | signed short
    comp_params.dig_P5 = ((int16_t)(c_reg[15]) << 8) | (c_reg[14]);         //0x96 / 0x97      | dig_P5 [7:0] / [15:8] | signed short
    comp_params.dig_P6 = ((int16_t)(c_reg[17]) << 8) | (c_reg[16]);         //0x98 / 0x99      | dig_P6 [7:0] / [15:8] | signed short
    comp_params.dig_P7 = ((int16_t)(c_reg[19]) << 8) | (c_reg[18]);         //0x9A / 0x9B      | dig_P7 [7:0] / [15:8] | signed short
    comp_params.dig_P8 = ((int16_t)(c_reg[21]) << 8) | (c_reg[20]);         //0x9C / 0x9D      | dig_P8 [7:0] / [15:8] | signed short
    comp_params.dig_P9 = ((int16_t)(c_reg[23]) << 8) | (c_reg[22]);         //0x9E / 0x9F      | dig_P9 [7:0] / [15:8] | signed short
    comp_params.dig_H1 = (uint8_t)c_reg[25];                                //0xA1             | dig_H1 [7:0]          | unsigned char
    comp_params.dig_H2 = ((int16_t)(c_reg[27]) << 8) | (c_reg[26]);         //0xE1 / 0xE2      | dig_H2 [7:0] / [15:8] | signed short
    comp_params.dig_H3 = (uint8_t)c_reg[28];                                //0xE3             | dig_H3 [7:0]          | unsigned char
    comp_params.dig_H4 = ((int16_t)(c_reg[29]) << 4) | (c_reg[30] & 0x0F);  //0xE4 / 0xE5[3:0] | dig_H4 [11:4] / [3:0] | signed short
    comp_params.dig_H5 = ((int16_t)(c_reg[31]) << 4) | (c_reg[30] >> 4);    //0xE5[7:4] / 0xE6 | dig_H5 [3:0] / [11:4] | signed short
    comp_params.dig_H6 = (int8_t)c_reg[32];                                 //0xE7             | dig_H6                | signed char

    setMode(BMX280_SLEEP);
    return 0;
}
uint8_t BMX280::begin(SPIClass *spi) {
    this->spi = spi;
    return begin();
}


void BMX280::reset() {
    writeRegister(BMX280_RESET_REG, BMX280_RESET);
    setMode(BMX280_SLEEP);
}

uint8_t BMX280::getId() {
    return readRegister(BMX280_ID_REG);
}

uint8_t BMX280::getStatus() {
    return readRegister(BMX280_STATUS_REG);
}


void BMX280::setMode(uint8_t mode) {
    setRegister(BMX280_CTRL_MEAS_REG, mode, 0, 1);
}

void BMX280::setTemperatureOversampling(uint8_t oversampling) {
    setRegister(BMX280_CTRL_MEAS_REG, oversampling, 5, 7);
}
void BMX280::setPressureOversampling(uint8_t oversampling) {
    setRegister(BMX280_CTRL_MEAS_REG, oversampling, 2, 4);
}
void BMX280::setHumidityOversampling(uint8_t oversampling) {
    setRegister(BME280_CTRL_HUM_REG, oversampling, 0, 2);
}

void BMX280::setIIRFilter(uint8_t filter) {
    setRegister(BMX280_CONFIG_REG, filter, 2, 4);
}
void BMX280::setStandby(uint8_t standby) {
    setRegister(BMX280_CONFIG_REG, standby, 5, 7);
}
void BMX280::set3WireSPI(uint8_t enable) {
    setRegister(BMX280_CONFIG_REG, enable, 0, 0);
}


int16_t BMX280::calculateTemperature(uint8_t *raw_t) {
    int32_t var1, var2;
    int32_t raw_t32 = ((uint32_t)raw_t[0] << 12) |
                      ((uint32_t)raw_t[1] << 4)  |
                      ((uint32_t)raw_t[2] >> 4);
    var1 = ((((raw_t32 >> 3) - ((int32_t)this->comp_params.dig_T1 << 1))) * ((int32_t)this->comp_params.dig_T2)) >> 11;
    var2 = (((((raw_t32 >> 4) - ((int32_t)this->comp_params.dig_T1)) * ((raw_t32 >> 4) - ((int32_t)this->comp_params.dig_T1))) >> 12) * ((int32_t)this->comp_params.dig_T3)) >> 14;
    this->t_fine = var1 + var2;
    return (this->t_fine * 5 + 128) >> 8;
}

uint32_t BMX280::calculatePressure(uint8_t *raw_p) {
    int32_t raw_p32 = ((uint32_t)raw_p[0] << 12) |
                      ((uint16_t)raw_p[1] << 4) |
                                (raw_p[2] >> 4);
    int32_t var1, var2;
    uint32_t p;
    var1 = (((int32_t)this->t_fine) >> 1) - (int32_t)64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11 ) * ((int32_t)this->comp_params.dig_P6);
    var2 = var2 + ((var1 * ((int32_t)this->comp_params.dig_P5)) << 1);
    var2 = (var2 >> 2) + (((int32_t)this->comp_params.dig_P4) << 16);
    var1 = (((this->comp_params.dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13 )) >> 3) + ((((int32_t)this->comp_params.dig_P2) * var1) >> 1)) >> 18;
    var1 =((((32768 + var1)) * ((int32_t)this->comp_params.dig_P1)) >> 15);

    //divison by zero
    if (var1 == 0)
        return 0; // avoid exception caused by division by zero

    p = (((uint32_t)(((int32_t)1048576) - raw_p32) - (var2 >> 12))) * 3125;
    if (p < 0x80000000)
        p = (p << 1) / ((uint32_t)var1);
    else
        p = (p / (uint32_t)var1) * 2;
    var1 = (((int32_t)this->comp_params.dig_P9) * ((int32_t)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
    var2 = (((int32_t)(p >> 2)) * ((int32_t)this->comp_params.dig_P8)) >> 13;
    p = (uint32_t)((int32_t)p + ((var1 + var2 + this->comp_params.dig_P7) >> 4));
    return p;
}

uint16_t BMX280::calculateHumidity(uint8_t *raw_h) {
    int16_t raw_h16 = ((uint16_t)raw_h[0] << 8) | raw_h[1];
    //TODO
}


int16_t BMX280::getTemperature() {
    //set temperature oversampling
    if (!(readRegister(BMX280_CTRL_MEAS_REG) & 0b11100000))
        setTemperatureOversampling(BMX280_TEMP_OVERx1);
    
    measure();

    uint8_t data[3] = {0};
    getTemperatureRaw(data);
    return calculateTemperature(data);
}

uint32_t BMX280::getPressure() {
    uint8_t reg = readRegister(BMX280_CTRL_MEAS_REG);
    //enable temperature oversampling
    if (!(reg & 0b11100000))
        setTemperatureOversampling(BMX280_TEMP_OVERx1);
    //enable pressure oversampling
    if (!(reg & 0b00011100))
        setPressureOversampling(BMX280_PRES_OVERx1);

    measure();

    uint8_t data[6] = {0};
    readRegisterBurst(BMX280_PRESS_MSB_REG, data, 6);
    calculateTemperature(data + 3);
    return calculatePressure(data);
}

uint16_t BMX280::getHumidity() {
    if (this->chip_id != BME280_ID)
        return 0;

    //enable temperature oversampling
    if (!(readRegister(BMX280_CTRL_MEAS_REG) & 0b11100000))
        setTemperatureOversampling(BMX280_TEMP_OVERx1);
    //enable humidity oversampling on BME280
    if (!(readRegister(BME280_CTRL_HUM_REG) & 0b00000111))
        setHumidityOversampling(BME280_HUM_OVERx1);

    measure();

    uint8_t data[5] = {0};
    readRegisterBurst(BMX280_TEMP_MSB_REG, data, 5);
    calculateTemperature(data);
    return calculateHumidity(data + 3);
}

void BMX280::getAll(int16_t *temperature, uint32_t *pressure, uint16_t *humidity) {
    uint8_t raw_data[8];
    getAllRaw(raw_data);

    *temperature = calculateTemperature(raw_data + 3);
    *pressure = calculatePressure(raw_data);
    if (this->chip_id == BME280_ID || humidity != nullptr)
        *humidity = calculateHumidity(raw_data + 6);
}


void BMX280::getPressureRaw(uint8_t *data) {
    measure();
    readRegisterBurst(BMX280_PRESS_MSB_REG, data, 3);
}

void BMX280::getTemperatureRaw(uint8_t *data) {
    measure();
    readRegisterBurst(BMX280_TEMP_MSB_REG, data, 3);
}

void BMX280::getHumidityRaw(uint8_t *data) {
    if (this->chip_id != BME280_ID)
        return;

    measure();
    readRegisterBurst(BME280_HUM_MSB_REG, data, 2);
}

void BMX280::getAllRaw(uint8_t *data) {
    uint8_t reg = readRegister(BMX280_CTRL_MEAS_REG);
    
    //enable temperature oversampling
    if (!(reg & 0b11100000))
        setTemperatureOversampling(BMX280_TEMP_OVERx1);
    
    //enable pressure oversampling
    if (!(reg & 0b00011100))
        setPressureOversampling(BMX280_PRES_OVERx1);
    
    //enable humidity oversampling on BME280
    if (this->chip_id == BME280_ID && !(readRegister(BME280_CTRL_HUM_REG) & 0b00000111))
        setHumidityOversampling(BME280_HUM_OVERx1);

    measure();

    if (this->chip_id == BME280_ID)
        readRegisterBurst(BMX280_PRESS_MSB_REG, data, 8);
    else
        readRegisterBurst(BMX280_PRESS_MSB_REG, data, 6);
}

void BMX280::measure() {
    //start one time measurement
    if (!(readRegister(BMX280_CTRL_MEAS_REG) & 0b00000011))
        setMode(BMX280_FORCED);

    //wait for a measurement to finish
    while ((readRegister(BMX280_STATUS_REG) & 0b00001000));
}