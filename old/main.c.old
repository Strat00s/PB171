#include <avr/io.h>
#include <util/delay.h>

#define LED_PIN PB5

int main(void) {
    DD0;
    // Set LED_PIN as output
    DDRB |= (1 << LED_PIN);

    while (1) {
        // Turn LED on
        PORTB |= (1 << LED_PIN);
        _delay_ms(1000);

        // Turn LED off
        PORTB &= ~(1 << LED_PIN);
        _delay_ms(1000);
    }

    return 0;
}
