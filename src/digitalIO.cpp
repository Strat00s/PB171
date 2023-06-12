#include "digitalIO.hpp"
#include "pinmap.hpp"

void pinMode(uint8_t pin, uint8_t mode) {
    if (mode == INPUT) {
        unsetBit(getDir(pin), getPin(pin));
    }
    else {
        setBit(getDir(pin), getPin(pin));
    }
}

uint8_t digitalRead(uint8_t pin) {
    return readBit(getIn(pin), getPin(pin));
}

void digitalWrite(uint8_t pin, uint8_t value) {
    if (value == LOW) {
        unsetBit(getPort(pin), getPin(pin));
        // clear bit
        //port &= ~(1 << pin);
    } else {
        // set bit
        setBit(getPort(pin), getPin(pin));
        //port |= (1 << pin);
    }
}