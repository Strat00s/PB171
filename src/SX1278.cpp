/**
 * @file SX1278.cpp
 * @author Lukáš Baštýř (l.bastyr@seznam.cz)
 * @brief SX1278 library
 * @version 0.1
 * @date 24-06-2023
 * 
 * @copyright Copyright (c) 2023
 * 
 */


#include <util/delay.h>
#include "SX1278.hpp"
#include "digitalIO.hpp"

SX1278::SX1278(uint8_t cs, uint8_t rst, uint8_t dio) {
    this->cs   = cs;
    this->rst  = rst;
    this->dio0  = dio0;

    pinMode(this->cs, OUTPUT);
    digitalWrite(this->cs, HIGH);
}

SX1278::~SX1278() {

}

//initialize module with following default settings:
//mode: LoRa
//frequency: 434MHz
//bandwidth: 125KHz
//spreading factor: 9 (512 chips/symbol)
//crc: enabled and set to 4/7
//gain: automatic
//frequency hopping: off
uint8_t SX1278::begin(SPIClass *spi, float frequency, uint8_t sync_word, uint16_t preamble_len) {
    this->spi = spi;
    if (this->spi == nullptr)
        return 2;

    this->spi->begin();

    //pinMode(this->cs, OUTPUT);
    //digitalWrite(this->cs, HIGH);

    if (getVersion() != SX1278_CHIP_VERSION)
        return 1;

    float freq   = 434.0F;
    float bw     = 125.0F;
    uint8_t sf   = 7U;
    uint8_t cr   = 7U;

    //set mode to standby
    setMode(SX1278_STANDBY);

    //turn off frequency hopping
    writeRegister(REG_HOP_PERIOD, HOP_PERIOD_OFF);

    //set lora mode
    setModemMode(SX1278_LORA);

    //set LoRa sync word
    writeRegister(REG_SYNC_WORD, sync_word);

    //set preamble length
    writeRegister(REG_PREAMBLE_MSB, (uint8_t)((preamble_len >> 8) & 0xFF));
    writeRegister(REG_PREAMBLE_LSB, (uint8_t)(preamble_len & 0xFF));

    //125khz bandwidth
    //is already default
    setRegister(REG_MODEM_CONFIG_1, LORA_BANDWIDTH_125kHz, 4, 7);

    //disable low data rate ptimalization, since symbol length is less than 16
    setRegister(REG_MODEM_CONFIG_3, LORA_LOW_DATA_RATE_OPT_OFF, 3, 3);
    
    //set frequency
    uint32_t FRF = (freq * (uint32_t(1) << 19)) / 32.0;
    setRegister(REG_FRF_MSB, (FRF & 0xFF0000) >> 16);
    setRegister(REG_FRF_MID, (FRF & 0x00FF00) >> 8);
    setRegister(REG_FRF_LSB, FRF & 0x0000FF);

    //explicit header
    setRegister(REG_MODEM_CONFIG_1, LORA_EXPLICIT_HEADER, 0, 0);

    //set spreading factor 7
    //is already set by default
    setRegister(REG_MODEM_CONFIG_2, LORA_SPREADING_FACTOR_7, 4, 7);
    setRegister(REG_DETECT_OPTIMIZE, DETECT_OPTIMIZE_SF_7_12, 0, 2);
    setRegister(REG_DETECTION_THRESHOLD, DETECTION_THRESHOLD_SF_7_12);

    //set error coding rate to 4/5
    setRegister(REG_MODEM_CONFIG_1, LORA_CODING_RATE_4_5, 1, 3);

    //module I have does not have RFO pin connected
    //so let's just enable PA_BOOST and set lowest possible power
    writeRegister(REG_PA_CONFIG, SX1278_PA_SELECT_BOOST);

    //enable automatic gain control
    setRegister(REG_MODEM_CONFIG_3, LORA_AGC_AUTO_ON, 2, 2);

    //enable crc
    setRegister(REG_MODEM_CONFIG_2, LORA_RX_PAYLOAD_CRC_ON, 2, 2);

    //disable all IO pins
    writeRegister(REG_DIO_MAPPING_1, 0xFF);
    setRegister(REG_DIO_MAPPING_2, 0b11110000);

    return 0;
}


void SX1278::reset() {
    pinMode(this->rst, OUTPUT);
    digitalWrite(this->rst, LOW);
    _delay_ms(1);
    digitalWrite(this->rst, HIGH);
    _delay_ms(5);
}

uint8_t SX1278::getVersion() {
    reset();
    _delay_ms(10);
    return readRegister(REG_VERSION);
}

void SX1278::setMode(uint8_t mode) {
    setRegister(REG_OP_MODE, mode, 0, 2);
}

void SX1278::setModemMode(uint8_t modem) {
    setMode(SX1278_SLEEP);

    setRegister(REG_OP_MODE, modem, 7, 7);

    setMode(SX1278_STANDBY);
}


uint8_t SX1278::readRegister(uint8_t addr) {
    digitalWrite(this->cs, LOW);
    uint8_t reg = this->spi->readRegister(addr, SX1278_READ);
    digitalWrite(this->cs, HIGH);
    return reg;
}

void SX1278::writeRegister(uint8_t addr, uint8_t data) {
    digitalWrite(this->cs, LOW);
    this->spi->writeRegister(addr, data, SX1278_WRITE);
    digitalWrite(this->cs, HIGH);
}

void SX1278::setRegister(uint8_t addr, uint8_t data, uint8_t mask_lsb, uint8_t mask_msb) {
    digitalWrite(this->cs, LOW);
    uint8_t reg = this->spi->readRegister(addr, SX1278_READ);
    digitalWrite(this->cs, HIGH);

    uint8_t mask = ~(0xFF >> (8 - mask_lsb) | 0xFF << (mask_msb + 1));
    data = (reg & ~mask) | (data & mask);

    digitalWrite(this->cs, LOW);
    this->spi->writeRegister(addr, data, SX1278_WRITE);
    digitalWrite(this->cs, HIGH);
}

uint8_t SX1278::transmit(uint8_t *data, uint8_t length) {
    setMode(SX1278_STANDBY);

    //set IO mapping for dio0 to be end of transmission
    setRegister(REG_DIO_MAPPING_1, DIO0_LORA_TX_DONE, 6, 7);

    //clear interrupt flags
    writeRegister(REG_IRQ_FLAGS, 0xFF);

    //set packet length
    setRegister(REG_PAYLOAD_LENGTH, length);

    //set FIFO pointers (all 256 bytes used for TX)
    setRegister(REG_FIFO_TX_BASE_ADDR, 0);
    setRegister(REG_FIFO_ADDR_PTR, 0);

    //write data to FIFO
    digitalWrite(this->cs, LOW);
    this->spi->writeRegisterBurst(REG_FIFO, data, length, SX1278_WRITE);
    digitalWrite(this->cs, HIGH);

    //start transmission
    setMode(SX1278_TX);
    while(!digitalRead(this->dio0));    //sometimes not enough
    while(!(readRegister(REG_IRQ_FLAGS) & 0b00001000));
    setMode(SX1278_STANDBY);

    //read and clear interrupts
    uint8_t reg = readRegister(REG_IRQ_FLAGS);
    writeRegister(REG_IRQ_FLAGS, 0xFF);
    return reg;
}
