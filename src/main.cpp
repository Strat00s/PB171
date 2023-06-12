#include <avr/io.h>
#include <util/delay.h>
#include "SPIClass.hpp"
#include "digitalIO.hpp"
#include "SX1278.hpp"
#include "pinmap.hpp"
#include "registers.hpp"


#define RST_PIN  D9
#define CS_PIN   D10
#define IRQ_PIN  D7
#define GPIO_PIN D6
#define LED_PIN  D8


SPIClass spi = SPIClass();
SX1278 lora = SX1278(CS_PIN, RST_PIN, IRQ_PIN, GPIO_PIN);


void blink(uint8_t cnt) {
    for (int i = 0; i < cnt; i++){
        digitalWrite(LED_PIN, HIGH);
        _delay_ms(50);
        digitalWrite(LED_PIN, LOW);
        _delay_ms(50);
    }
}


int main() {

    pinMode(IRQ_PIN, INPUT);
    pinMode(GPIO_PIN, INPUT);

    spi.init();
    lora.init(&spi, 18U, 8U, 10);

    if (!lora.init(&spi, 18U, 8U, 10)){
        while (true) {
            blink(2);
            _delay_ms(100);
        }
        
    }

    pinMode(LED_PIN, OUTPUT);
    blink(10);

    while(true) {
        blink(5);
        _delay_ms(2000);
        lora.setMode(STANDBY);

        char message[] = "test";
        lora.transmit((uint8_t*)message, 5, 0, 0);

        lora.setMode(SLEEP);
    }

    return 0;
}
