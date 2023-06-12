#include "avr/io.h"
class Serial {
private:
    void transmit(char data);

public:
    Serial(/* args */);
    ~Serial();

    void init(unsigned long baud);


    void print(const char* str);
    void print(int num);
    void print(float num);
    void printHex(int nume);

    void println(const char* str);
    void println(int num);
    void println(float num);
    void printHexln(int num);

};


