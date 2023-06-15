#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include "serial.hpp"


void Serial::init(unsigned long baud_rate) {
    //set baud rate
    unsigned int ubrr = 16000000 / (16 * baud_rate) - 1;
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
