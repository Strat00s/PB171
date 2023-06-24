/**
 * @file digitalIO.cpp
 * @author Lukáš Baštýř (l.bastyr@seznam.cz)
 * @brief Digital IO functions
 * @version 0.1
 * @date 24-06-2023
 * 
 * @copyright Copyright (c) 2023
 * 
 */


#include <avr/io.h>
#include "digitalIO.hpp"
#include "pinmap.hpp"

//TODO check for valid pins

#if defined (__AVR_ATmega328P__)

void pinMode(uint8_t pin, uint8_t mode) {
    if (mode == INPUT)
        unsetBit(getDir(pin), getPin(pin));
    else
        setBit(getDir(pin), getPin(pin));
}

uint8_t digitalRead(uint8_t pin) {
    return readBit(getIn(pin), getPin(pin));
}

void digitalWrite(uint8_t pin, uint8_t value) {
    if (value == LOW)
        unsetBit(getPort(pin), getPin(pin));
    else
        setBit(getPort(pin), getPin(pin));
}

#elif defined (__AVR_ATtiny1624__)

void pinMode(uint8_t pin, uint8_t mode) {
    if (mode == INPUT)
        getPort(pin)->DIRCLR = (1 << getPin(pin));
    else
        getPort(pin)->DIRSET = (1 << getPin(pin));
}

uint8_t digitalRead(uint8_t pin) {
    return getPort(pin)->IN & (1 << getPin(pin));
}

void digitalWrite(uint8_t pin, uint8_t value) {
    if (value == LOW)
        getPort(pin)->OUTCLR = (1 << getPin(pin));
    else
        getPort(pin)->OUTSET = (1 << getPin(pin));
}

#endif