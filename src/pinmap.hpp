#pragma once

//Arduino UNO pin mapping

#include <avr/io.h>
#include "registers.hpp"

//digital pins
#define D0   0  //PD0
#define D1   1  //PD1
#define D2   2  //PD2
#define D3   3  //PD3
#define D4   4  //PD4
#define D5   5  //PD5
#define D6   6  //PD6
#define D7   7  //PD7
#define D8   8  //PB0
#define D9   9  //PB1
#define D10 10  //PB2
#define D11 11  //PB3
#define D12 12  //PB4
#define D13 13  //PB5
//analog pins
#define A0  14  //PC0
#define A1  15  //PC1
#define A2  16  //PC2
#define A3  17  //PC3
#define A4  18  //PC4
#define A5  19  //PC5

#define _PORTD 0
#define _PORTB 1
#define _PORTC 2
#define _DDRD  3
#define _DDRB  4
#define _DDRC  5
#define _PIND  6
#define _PINB  7
#define _PINC  8


typedef struct pinmap_t {
    uint8_t pin;
    uint8_t port;
    uint8_t ddr;
    uint8_t in;
} pinmap_t;

uint8_t getPort(uint8_t pin);

uint8_t getDir(uint8_t pin);

uint8_t getIn(uint8_t pin);

uint8_t getPin(uint8_t pin);
