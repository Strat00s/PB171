#pragma once
#include <avr/io.h>


class Serial {
public:
    Serial() {};
    ~Serial() {};

    void init(unsigned long baud_rate);

    void print(const char *str);
    void println(const char *str);

    void print(int num);
    void println(int num);

    void print(float num);
    void println(float num);

    void printHex(int num);
    void printlnHex(int num);
};