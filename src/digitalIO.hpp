#pragma once
#include "pinmap.hpp"

#include <avr/io.h>


#define LOW 0
#define HIGH 1

#define INPUT 0
#define OUTPUT 1


void pinMode(uint8_t pin, uint8_t mode);
uint8_t digitalRead(uint8_t pin);
void digitalWrite(uint8_t pin, uint8_t value);
