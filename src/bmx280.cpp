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
    digitalWrite(this->cs, LOW);
    uint8_t reg = this->spi->readRegister(addr, BMX280_READ);
    digitalWrite(this->cs, HIGH);

    uint8_t mask = ~(0xFF >> (8 - mask_lsb) | 0xFF << (mask_msb + 1));
    data = (reg & ~mask) | (data & mask);

    digitalWrite(this->cs, LOW);
    this->spi->writeRegister(addr, data, BMX280_WRITE);
    digitalWrite(this->cs, HIGH);
}


uint8_t BMX280::init() {
    if (this->spi == nullptr)
        return 2;

    //pinMode(this->cs, OUTPUT);
    //digitalWrite(this->cs, LOW);    //set SPI mode
    //digitalWrite(this->cs, HIGH);
    this->spi->init();
    
    switch (getId()) {
        case BMP280_SAMPLE_ID:
        case BMP280_SAMPLE2_ID:
        case BMP280_ID: this->chip_id = BMP280_ID; break;
        case BME280_ID: this->chip_id = BME280_ID; break;
        default: this->chip_id = 0; return 1;
    }

    //store calibrations
    readRegisterBurst(BMX280_CALIB00_REG, calib_regs, 26);
    if (this->chip_id == BME280_ID)
        readRegisterBurst(BME280_CALIB26_REG, calib_regs + 26, 16);

    setMode(BMX280_SLEEP);
    return 0;
}
uint8_t BMX280::init(SPIClass *spi) {
    this->spi = spi;
    return init();
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
    setRegister(BMX280_CTRL_MEAS_REG, oversampling, 2, 4);
}
void BMX280::setPressureOversampling(uint8_t oversampling) {
    setRegister(BMX280_CTRL_MEAS_REG, oversampling, 5, 7);
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



uint32_t BMX280::calculatePressure(uint8_t *raw_pressure) {

}

int16_t BMX280::calculateTemperature(uint8_t *raw_temperature) {

}

uint16_t BMX280::calculateHumidity(uint8_t *raw_humidity) {

}

void BMX280::getAll(uint32_t *pressure, uint16_t *temperature, uint16_t *humidity) {
    uint8_t raw_data[8];
    getAllRaw(raw_data);

    *pressure = calculatePressure(raw_data);
    *temperature = calculateTemperature(raw_data + 3);
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
    measure();

    if (this->chip_id == BME280_ID)
        readRegisterBurst(BMX280_PRESS_MSB_REG, data, 8);
    else
        readRegisterBurst(BMX280_PRESS_MSB_REG, data, 6);
}

void BMX280::measure() {
    //start one time measurement
    if (readRegister(BMX280_CTRL_MEAS_REG) & BMX280_SLEEP)
        setMode(BMX280_FORCED);

    //wait for a measurement to finish
    while (!(readRegister(BMX280_STATUS_REG) & 0b00001000));
}