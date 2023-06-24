/**
 * @file serial.hpp
 * @author Lukáš Baštýř (l.bastyr@seznam.cz)
 * @brief Serial (UART) header file
 * @version 0.1
 * @date 16-06-2023
 * 
 * @copyright Copyright (c) 2023
 * 
 */


#pragma once
#include <avr/io.h>


class Serial {
public:
    Serial() {};
    ~Serial() {};

    void begin(unsigned long baud_rate);
    void end();

    void print(const char *str);
    void println(const char *str);

    void print(int32_t num);
    void println(int32_t num);
    void printU(uint32_t num);
    void printlnU(uint32_t num);

    void printF(float num);
    void printlnF(float num);

    void printHex(uint32_t num);
    void printlnHex(uint32_t num);
};