#pragma once

#include <avr/io.h>

#define LOW 0
#define HIGH 1

#define INPUT 0
#define OUTPUT 1



void pinMode(uint8_t pin, uint8_t mode);
uint8_t digitalRead(uint8_t pin);
void digitalWrite(uint8_t pin, uint8_t value);


/*
#include <avr/io.h>

#define LOW 0
#define HIGH 1

#define INPUT 0
#define OUTPUT 1


void pinMode(volatile uint8_t& ddr, uint8_t pin, uint8_t mode) {
    if (mode == INPUT) {
        // set bit to 0 for input
        ddr &= ~(1 << pin);
    }
    else {
        // set bit to 1 for output
        ddr |= (1 << pin);
    }
}

uint8_t digitalRead(volatile uint8_t& in_port, uint8_t pin) {
    return (in_port & (1 << pin)) ? HIGH : LOW;
}

void digitalWrite(volatile uint8_t& port, uint8_t pin, uint8_t value) {
    if (value == LOW) {
        // clear bit
        port &= ~(1 << pin);
    } else {
        // set bit
        port |= (1 << pin);
    }
}

*/
