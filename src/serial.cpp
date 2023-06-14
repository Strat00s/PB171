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

void Serial::print(int num) {
    char buffer[12];
    itoa(num, buffer, 10);
    print(buffer);
}

void Serial::println(int num) {
    print(num);
    print("\r\n");
}

void Serial::print(float num) {
    char buffer[32];
    dtostrf(num, 4, 2, buffer); // Float to string
    print(buffer);
}

void Serial::println(float num) {
    print(num);
    print("\r\n");
}

void Serial::printHex(int num) {
    char buffer[12];
    itoa(num, buffer, 16);
    print(buffer);
}

void Serial::printlnHex(int num) {
    printHex(num);
    print("\r\n");
}