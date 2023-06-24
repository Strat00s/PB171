/**
 * @file pinmap.cpp
 * @author Lukáš Baštýř (l.bastyr@seznam.cz)
 * @brief Pin maps
 * @version 0.1
 * @date 24-06-2023
 * 
 * @copyright Copyright (c) 2023
 * 
 */


#include "pinmap.hpp"
#if defined (__AVR_ATmega328P__)

pinmap_t pinmap[20] {
    {PD0, _PORTD, _DDRD, _PIND},   //D0
    {PD1, _PORTD, _DDRD, _PIND},   //D1
    {PD2, _PORTD, _DDRD, _PIND},   //D2
    {PD3, _PORTD, _DDRD, _PIND},   //D3
    {PD4, _PORTD, _DDRD, _PIND},   //D4
    {PD5, _PORTD, _DDRD, _PIND},   //D5
    {PD6, _PORTD, _DDRD, _PIND},   //D6
    {PD7, _PORTD, _DDRD, _PIND},   //D7
    {PB0, _PORTB, _DDRB, _PINB},   //D8
    {PB1, _PORTB, _DDRB, _PINB},   //D9
    {PB2, _PORTB, _DDRB, _PINB},   //D10
    {PB3, _PORTB, _DDRB, _PINB},   //D11
    {PB4, _PORTB, _DDRB, _PINB},   //D12
    {PB5, _PORTB, _DDRB, _PINB},   //D13

    {PC0, _PORTC, _DDRC, _PINC},   //A0
    {PC1, _PORTC, _DDRC, _PINC},   //A1
    {PC2, _PORTC, _DDRC, _PINC},   //A2
    {PC3, _PORTC, _DDRC, _PINC},   //A3
    {PC4, _PORTC, _DDRC, _PINC},   //A4
    {PC5, _PORTC, _DDRC, _PINC}    //A5
};

uint8_t getDir(uint8_t pin) {
    return pinmap[pin].ddr;
}

uint8_t getPort(uint8_t pin) {
    return pinmap[pin].port;
}

uint8_t getIn(uint8_t pin) {
    return pinmap[pin].in;
}


#elif defined (__AVR_ATtiny1624__)

pinmap_t pinmap[12] {
    {0, &PORTA},   //A0
    {1, &PORTA},   //A1
    {2, &PORTA},   //A2
    {3, &PORTA},   //A3
    {4, &PORTA},   //A4
    {5, &PORTA},   //A5
    {6, &PORTA},   //A6
    {7, &PORTA},   //A7
    {0, &PORTB},   //B0
    {1, &PORTB},   //B1
    {2, &PORTB},   //B2
    {3, &PORTB}    //B3
};


PORT_t *getPort(uint8_t pin) {
    return pinmap[pin].port;
}

#endif

uint8_t getPin(uint8_t pin) {
    return pinmap[pin].pin;
}