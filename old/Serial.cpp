#include <stdlib.h>
#include "Serial.hpp"

#define F_CPU 16000000UL


Serial::Serial(/* args */) {
}

Serial::~Serial() {
}

void Serial::init(unsigned long baud)
{
    // Calculate baud rate register value
    uint16_t ubrr = F_CPU / 16 / baud - 1;

    // Set baud rate
    UBRR0H = (uint8_t)(ubrr >> 8);
    UBRR0L = (uint8_t)ubrr;

    // Enable receiver and transmitter
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);

    // Set frame format: 8 data bits, 1 stop bit, no parity
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void Serial::transmit(char data)
{
    // Wait for empty transmit buffer
    while (!(UCSR0A & (1 << UDRE0)));

    // Put data into buffer, sends the data
    UDR0 = data;
}

void Serial::print(const char* str)
{
    while (*str) {
        transmit(*str++);
    }
}

void Serial::print(int num)
{
    char buffer[10];
    itoa(num, buffer, 10);  // Convert integer to string (base 10)
    print(buffer);
}

void Serial::print(float num)
{
    char buffer[32];
    dtostrf(num, 6, 2, buffer);  // Convert float to string
    print(buffer);
}

void Serial::printHex(int num) {
    char buffer[50];
    itoa(num, buffer, 16); // str will contain "ff"
    print(buffer);
}

void Serial::println(const char* str) {
    print(str);
    print("\n");
}
void Serial::println(int num) {
    print(num);
    print("\n");
}
void Serial::println(float num) {
    print(num);
    print("\n");
}

void Serial::printHexln(int num) {
    printHex(num);
    print("\n");
}