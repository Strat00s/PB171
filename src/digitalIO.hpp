/**
 * @file digitalIO.hpp
 * @author Lukáš Baštýř (l.bastyr@seznam.cz)
 * @brief Digital IO header file
 * @version 0.1
 * @date 24-06-2023
 * 
 * @copyright Copyright (c) 2023
 * 
 */


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
