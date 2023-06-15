#include "SPIClass.hpp"
#include <avr/io.h>

#define MSB_0(x) (x & 0x7F)
#define MSB_1(x) (x | 0x80)

SPIClass SPI;

//Create SPI object with default configuration:
//speed: 2MHz
//msb first
//mode 0
SPIClass::SPIClass() {
    initialized = 0;
}

SPIClass::~SPIClass() {
}


void SPIClass::init() {
    if (initialized)
        return;
    //Set MOSI and SCK and SS as output
    DDRB = (1 << PB3) | (1 << PB5) | (1 << PB2);

    //Enable SPI, set it to master mode, set clock rate to 2MHz (fck/8), msb first, mode 0
    SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0);
    SPSR = 1 << SPI2X;

    initialized++;
}


uint8_t SPIClass::transfer(uint8_t data) {
    SPDR = data;
    asm volatile("nop");
    while (!(SPSR & (1 << SPIF)));
    return SPDR;
}

void SPIClass::setReadWrite(uint8_t addr, uint8_t rw) {
    if (rw)
        transfer(MSB_1(addr));
    else
        transfer(MSB_0(addr));
}

uint8_t SPIClass::readRegister(uint8_t addr, uint8_t rw) {
    setReadWrite(addr, rw);
    return transfer(0x00);
}

void SPIClass::writeRegister(uint8_t addr, uint8_t data, uint8_t rw) {
    setReadWrite(addr, rw);
    transfer(data);
}
void SPIClass::writeRegisterBurst(uint8_t addr, uint8_t* data, uint8_t length, uint8_t rw) {
    setReadWrite(addr, rw);
    for (int i = 0; i < length; i++) {
        transfer(data[i]);
    }
}
void SPIClass::readRegisterBurst(uint8_t addr, uint8_t* data, uint8_t length, uint8_t rw) {
    setReadWrite(addr, rw);
    for (int i = 0; i < length; i++) {
        data[i] = transfer(0x00);
    }
}

