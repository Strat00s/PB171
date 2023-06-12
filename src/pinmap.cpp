#include "pinmap.hpp"

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

uint8_t getPort(uint8_t pin) {
    return pinmap[pin].port;
}

uint8_t getDir(uint8_t pin) {
    return pinmap[pin].ddr;
}

uint8_t getIn(uint8_t pin) {
    return pinmap[pin].in;
}

uint8_t getPin(uint8_t pin) {
    return pinmap[pin].pin;
}