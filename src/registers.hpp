#if defined (__AVR_ATmega328P__)

#pragma once
#include <avr/io.h>

//all ports that I care about for simplicity
#define _PORTD 0
#define _PORTB 1
#define _PORTC 2
#define _DDRD  3
#define _DDRB  4
#define _DDRC  5
#define _PIND  6
#define _PINB  7
#define _PINC  8

//register adress cannot be passed by reference
void unsetBit(uint8_t reg, uint8_t bit);
void setBit(uint8_t reg, uint8_t bit);
uint8_t readBit(uint8_t reg, uint8_t bit);

#endif
