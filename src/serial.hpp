/**
 * @file serial.hpp
 * @author Lukáš Baštýř (l.bastyr@seznam.cz)
 * @brief 
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

    void init(unsigned long baud_rate);

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