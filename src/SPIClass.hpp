#pragma once
//simple SPI class for comunicating with connected devices
#include <avr/io.h>

class SPIClass {
private:
    uint8_t transfer(uint8_t data);
    uint8_t initialized;

public:
    SPIClass();
    ~SPIClass();

    void init();

    uint8_t readRegister(uint8_t addr);

    void writeRegister(uint8_t addr, uint8_t data);
    void writeRegisterBurst(uint8_t addr, uint8_t* data, uint8_t length);

    //void setRegister(uint8_t addr, uint8_t data, uint8_t mask_lsb = 0, uint8_t mask_msb = 7);
    //uint8_t getRegister(uint8_t addr, uint8_t mask_lsb, uint8_t mask_msb);

};
