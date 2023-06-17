#if defined (__AVR_ATmega328P__)
#include "registers.hpp"

void unsetBit(uint8_t reg, uint8_t bit) {
    if (reg == _PORTD) {
        PORTD &= ~(1 << bit);
    }
    if (reg == _PORTB) {
        PORTB &= ~(1 << bit);
    }
    if (reg == _PORTC) {
        PORTC &= ~(1 << bit);
    }
    if (reg == _DDRD) {
        DDRD &= ~(1 << bit);
    }
    if (reg == _DDRB) {
        DDRB &= ~(1 << bit);
    }
    if (reg == _DDRC) {
        DDRC &= ~(1 << bit);
    }
    if (reg == _PIND) {
        PIND &= ~(1 << bit);
    }
    if (reg == _PINB) {
        PINB &= ~(1 << bit);
    }
    if (reg == _PINC) {
        PINC &= ~(1 << bit);
    }
}

void setBit(uint8_t reg, uint8_t bit) {
    if (reg == _PORTD) {
        PORTD |= (1 << bit);
    }
    if (reg == _PORTB) {
        PORTB |= (1 << bit);
    }
    if (reg == _PORTC) {
        PORTC |= (1 << bit);
    }
    if (reg == _DDRD) {
        DDRD |= (1 << bit);
    }
    if (reg == _DDRB) {
        DDRB |= (1 << bit);
    }
    if (reg == _DDRC) {
        DDRC |= (1 << bit);
    }
    if (reg == _PIND) {
        PIND |= (1 << bit);
    }
    if (reg == _PINB) {
        PINB |= (1 << bit);
    }
    if (reg == _PINC) {
        PINC |= (1 << bit);
    }
}

uint8_t readBit(uint8_t reg, uint8_t bit) {
    if (reg == _PORTD) {
        return (PORTD & (1 << bit)) ? 1 : 0;
    }
    if (reg == _PORTB) {
        return (PORTB & (1 << bit)) ? 1 : 0;
    }
    if (reg == _PORTC) {
        return (PORTC & (1 << bit)) ? 1 : 0;
    }
    if (reg == _DDRD) {
        return (DDRD & (1 << bit)) ? 1 : 0;
    }
    if (reg == _DDRB) {
        return (DDRB & (1 << bit)) ? 1 : 0;
    }
    if (reg == _DDRC) {
        return (DDRC & (1 << bit)) ? 1 : 0;
    }
    if (reg == _PIND) {
        return (PIND & (1 << bit)) ? 1 : 0;
    }
    if (reg == _PINB) {
        return (PINB & (1 << bit)) ? 1 : 0;
    }
    if (reg == _PINC) {
        return (PINC & (1 << bit)) ? 1 : 0;
    }
}

#endif