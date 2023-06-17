/**
 * @file serial.cpp
 * @author Lukáš Baštýř (l.bastyr@seznam.cz)
 * @brief 
 * @version 0.1
 * @date 16-06-2023
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include "serial.hpp"
#include "digitalIO.hpp"
#include "pinmap.hpp"

#if defined (__AVR_ATmega328P__)
void Serial::init(unsigned long baud_rate) {
    //set baud rate
    unsigned int ubrr = F_CPU / (16 * baud_rate) - 1;
    UBRR0H = (uint8_t)(ubrr >> 8);
    UBRR0L = (uint8_t) ubrr;

    //enable tx and rx
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);

    //set 8bit frame (1 stop bit)
    UCSR0C = (3 << UCSZ00);
}

void Serial::print(const char *str) {
    while (*str) {
        while (!(UCSR0A & (1 << UDRE0)));
        UDR0 = *str++;
    }
}

#elif defined (__AVR_ATtiny1624__)

void Serial::init(unsigned long baud_rate) {
    pinMode(TX0, OUTPUT);    //TX ouptut
    pinMode(RX0, INPUT);     //RX input
    
    //set baud rate
    USART0.BAUD = (uint16_t)((float)F_CPU * 64 / (16 * (float)baud_rate) + 0.5);

    //enable tx and rx
    USART0.CTRLB = USART_TXEN_bm | USART_RXEN_bm;

    //set 8bit frame, 1 stop bit, no parity
    USART0.CTRLC = USART_CHSIZE_8BIT_gc | USART_PMODE_DISABLED_gc | USART_SBMODE_1BIT_gc;

}

void Serial::print(const char *str) {
    while (*str) {
        while (!(USART0.STATUS & USART_DREIF_bm));
        USART0.TXDATAL = *str++;
    }
}

#endif

void Serial::println(const char *str) {
    print(str);
    print("\r\n");
}


void Serial::print(int32_t num) {
    char buffer[20];
    ltoa(num, buffer, 10);
    print(buffer);
}

void Serial::println(int32_t num) {
    print(num);
    print("\r\n");
}

void Serial::printU(uint32_t num) {
    char buffer[20];
    ultoa(num, buffer, 10);
    print(buffer);
}

void Serial::printlnU(uint32_t num) {
    printU(num);
    print("\r\n");
}


void Serial::printF(float num) {
    char buffer[32];
    dtostrf(num, 4, 2, buffer); // Float to string
    print(buffer);
}

void Serial::printlnF(float num) {
    print(num);
    print("\r\n");
}

void Serial::printHex(uint32_t num) {
    char buffer[12];
    ultoa(num, buffer, 16);
    print(buffer);
}

void Serial::printlnHex(uint32_t num) {
    printHex(num);
    print("\r\n");
}
