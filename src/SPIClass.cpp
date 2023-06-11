#include "SPIClass.hpp"
#include <avr/io.h>

#define READ(x) (x & 0x7F)
#define WRITE(x) (x | 0x80)

SPIClass SPI;

//Create SPI object with default configuration:
//speed: 2MHz
//msb first
//mode 0
SPIClass::SPIClass() {
    //// Set MOSI and SCK as output, others as input
    //DDRB = (1<<PB3)|(1<<PB5);
//
    //// Enable SPI, Master mode, set clock rate fck/8 (2 MHz), msb first, mode 0
    //SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);
    //SPSR = (1<<SPI2X);
}

SPIClass::~SPIClass() {
}


void SPIClass::init() {
    //Set MOSI and SCK and SS as output
    DDRB = (1<<PB3)|(1<<PB5)|(1<<PB2);

    //Enable SPI, set it to master mode, set clock rate to 2MHz (fck/8), msb first, mode 0
    SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);
    SPSR = (1<<SPI2X);
}


uint8_t SPIClass::transfer(uint8_t data) {
    SPDR = data;
    asm volatile("nop");
    while (!(SPSR & (1 << SPIF))) ; // wait
    return SPDR;
}

uint8_t SPIClass::readRegister(uint8_t addr) {
    transfer(READ(addr));
    return transfer(0x00);
}

void SPIClass::writeRegister(uint8_t addr, uint8_t data) {
    transfer(WRITE(addr));
    transfer(data);
}
void SPIClass::writeRegisterBurst(uint8_t addr, uint8_t* data, uint8_t length) {
    transfer(WRITE(addr));
    for (int i = 0; i < length; i++) {
        transfer(data[i]);
    }
}
//void SPIClass::setRegister(uint8_t addr, uint8_t data, uint8_t mask_lsb, uint8_t mask_msb) {
//    uint8_t reg = readRegister(addr);
//
//    uint8_t mask = ~(0xFF >> (8 - mask_lsb) | 0xFF << (mask_msb + 1));
//    data = (reg & ~mask) | (data & mask);
//
//    writeRegister(addr, data);
//}

