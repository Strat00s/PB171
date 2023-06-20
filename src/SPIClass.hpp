#pragma once
//simple SPI class for comunicating with connected devices
#include <avr/io.h>

class SPIClass {
private:
    void setReadWrite(uint8_t addr, uint8_t rw);
    uint8_t transfer(uint8_t data);

public:
    SPIClass();
    ~SPIClass();

    void begin();
    void end();

    uint8_t readRegister(uint8_t addr, uint8_t rw);
    void readRegisterBurst(uint8_t addr, uint8_t* data, uint8_t length, uint8_t rw);

    void writeRegister(uint8_t addr, uint8_t data, uint8_t rw);
    void writeRegisterBurst(uint8_t addr, uint8_t* data, uint8_t length, uint8_t rw);
};
