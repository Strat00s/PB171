#include "SX1278.hpp"
#include "digitalIO.hpp"

SX1278::SX1278(uint8_t cs, uint8_t rst, uint8_t irq, uint8_t gpio) {
    this->cs   = cs;
    this->rst  = rst;
    this->irq  = irq;
    this->gpio = gpio;
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
uint8_t SX1278::init(SPIClass *spi, uint8_t sync_word, uint16_t preamble_len, int8_t power) {
    this->spi = spi;
    this->spi->init();

    pinMode(this->cs, OUTPUT);
    digitalWrite(this->cs, HIGH);

    if (getVersion() != CHIP_VERSION)
        return 0;

    float freq   = 434.0F;
    float bw     = 125.0F;
    uint8_t sf   = 9U;
    uint8_t cr   = 7U;

    //set mode to standby
    setMode(STANDBY);

    //turn off frequency hopping
    writeRegister(REG_HOP_PERIOD, HOP_PERIOD_OFF);

    //set lora mode
    setModemMode(LORA);

    //set LoRa sync word
    writeRegister(REG_SYNC_WORD, sync_word);

    setCurrentLimit(60);

    writeRegister(REG_PREAMBLE_MSB, (uint8_t)((preamble_len >> 8) & 0xFF));
    writeRegister(REG_PREAMBLE_LSB, (uint8_t)(preamble_len & 0xFF));

    //125khz bandwidth
    //is already default
    setRegister(REG_MODEM_CONFIG_1, 0b01110000, 4, 7);

    //434 frequency
    //default already is 434
    uint32_t FRF = (freq * (uint32_t(1) << 19)) / 32.0;
    setRegister(REG_FRF_MSB, (FRF & 0xFF0000) >> 16);
    setRegister(REG_FRF_MID, (FRF & 0x00FF00) >> 8);
    setRegister(REG_FRF_LSB, FRF & 0x0000FF);

    //explicit header
    setRegister(REG_MODEM_CONFIG_1, HEADER_EXPL_MODE, 0, 0);

    //set spreading factor;
    //is also default already
    setRegister(REG_MODEM_CONFIG_2, SF_9 | TX_MODE_SINGLE, 4, 7);
    setRegister(REG_DETECT_OPTIMIZE, DETECT_OPTIMIZE_SF_7_12, 0, 2);
    setRegister(REG_DETECTION_THRESHOLD, DETECTION_THRESHOLD_SF_7_12);

    //set error coding rate to 4/7
    setRegister(REG_MODEM_CONFIG_1, CR_4_7, 1, 3);

    //state = setOutputPower(power);
    setRegister(REG_PA_CONFIG, PA_SELECT_BOOST, 7, 7);
    setRegister(REG_PA_CONFIG, MAX_POWER | (power - 2), 0, 6);
    setRegister(REG_PA_DAC, PA_BOOST_OFF, 0, 2);

    //enable automatic gain control
    setRegister(REG_MODEM_CONFIG_3, AGC_AUTO_ON, 2, 2);

    //enable crc
    setRegister(REG_MODEM_CONFIG_2, RX_CRC_MODE_ON, 2, 2);

    //configure low data rate
    float symbolLength = (float)(uint32_t(1) << sf) / (float)bw;
    if(symbolLength >= 16.0) {
        setRegister(REG_MODEM_CONFIG_3, LOW_DATA_RATE_OPT_ON, 3, 3);
    }
    else {
        setRegister(REG_MODEM_CONFIG_3, LOW_DATA_RATE_OPT_OFF, 3, 3);
    }

    return 1;
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
    setMode(SLEEP);

    setRegister(REG_OP_MODE, modem, 7, 7);

    setMode(STANDBY);
}

void SX1278::setCurrentLimit(uint8_t current_limit) {
    setMode(STANDBY);

    uint8_t raw;
    uint8_t reg = readRegister(REG_OCP);

    if(current_limit == 0) {
        // limit set to 0, disable OCP
        reg = (reg & 0b11011111) | OCP_OFF;
    }
    else if(current_limit <= 120) {
        raw = (current_limit - 45) / 5;
        reg = (reg & 0b11000000) | OCP_ON | raw;
    }
    else if(current_limit <= 240) {
        raw = (current_limit + 30) / 10;
        reg = (reg & 0b11000000) | OCP_ON | raw;
    }

    writeRegister(REG_OCP, reg);
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


void SX1278::startTransmission(uint8_t *data, uint8_t length, uint8_t addr) {
    setMode(STANDBY);

    //set IO mapping for irq to be end of transmission
    setRegister(REG_DIO_MAPPING_1, DIO0_LORA_TX_DONE, 6, 7);

    //clear interrupt flags
    writeRegister(REG_IRQ_FLAGS, 0xFF);

    //set packet length
    setRegister(REG_PAYLOAD_LENGTH, length);

    //set FIFO pointers
    setRegister(REG_FIFO_TX_BASE_ADDR, FIFO_TX_BASE_ADDR_MAX);
    setRegister(REG_FIFO_ADDR_PTR, FIFO_TX_BASE_ADDR_MAX);

    //write data to FIFO
    digitalWrite(this->cs, LOW);
    this->spi->writeRegisterBurst(REG_FIFO, data, length, SX1278_WRITE);
    digitalWrite(this->cs, HIGH);

    //start transmission
    setMode(TX);
}
uint8_t SX1278::finishTransmission() {
    uint8_t reg = readRegister(REG_IRQ_FLAGS);
    writeRegister(REG_IRQ_FLAGS, 0xFF);
    setMode(STANDBY);
    return reg;
}

uint8_t SX1278::transmit(uint8_t *data, uint8_t length, uint8_t addr, uint8_t timeout) {
    setMode(STANDBY);
    startTransmission(data, length, addr);
    while(!digitalRead(this->irq));
    return finishTransmission();
}
