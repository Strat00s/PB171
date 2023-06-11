#include <avr/io.h>
#include <util/delay.h>

//#include "SPIClass.hpp"
#include "digitalIO.hpp"

#define LED_PIN PB5
#define LED_PORT PORTB
#define LED_DDR DDRB


int main(void) {
    pinMode(LED_DDR, LED_PIN, OUTPUT);

    while (1) {
        digitalWrite(LED_PORT, LED_PIN, HIGH);
        _delay_ms(1000);
        digitalWrite(LED_PORT, LED_PIN, LOW);
        _delay_ms(1000);

    }
    return 0;
}