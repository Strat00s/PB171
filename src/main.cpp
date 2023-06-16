/**
 * @file main.cpp
 * @author Lukáš Baštýř (l.bastyr@seznam.cz)
 * @brief 
 * @version 0.1
 * @date 16-06-2023
 * 
 * @copyright Copyright (c) 2023
 * 
 */


#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>

#include "SPIClass.hpp"
#include "digitalIO.hpp"
#include "SX1278.hpp"
#include "pinmap.hpp"
#include "registers.hpp"
#include "serial.hpp"
#include "bmx280.hpp"


#define DELAY(x) _delay_ms(x)


#define LORA_RST  D9
#define LORA_CS   D10
#define LORA_IRQ  D7
#define LORA_GPIO D6

#define LED_PIN D8

#define BMP_CS D3


SPIClass spi = SPIClass();
SX1278 lora = SX1278(LORA_CS, LORA_RST, LORA_IRQ, LORA_GPIO);
Serial serial = Serial();
BMX280 bmp = BMX280(BMP_CS);

void blink(uint8_t cnt) {
    for (int i = 0; i < cnt; i++){
        digitalWrite(LED_PIN, HIGH);
        _delay_ms(50);
        digitalWrite(LED_PIN, LOW);
        _delay_ms(50);
    }
}


int main() {
    digitalWrite(LORA_CS, HIGH);

    pinMode(LORA_IRQ, INPUT);
    pinMode(LORA_GPIO, INPUT);


    serial.init(9600);
    spi.init();
    serial.println("SPI init");

    serial.print("LoRa init: ");
    serial.println(lora.init(&spi, 18U, 8U, 10));
    serial.println("Chip found");

    serial.print("BMP init: ");
    serial.println(bmp.init(&spi));
    serial.print("BMP ID: 0x");
    serial.printlnHex(bmp.getId());
    //bmp.setTemperatureOversampling(BMX280_TEMP_OVERx1);


    pinMode(LED_PIN, OUTPUT);
    blink(10);

    while(true) {
        blink(5);
        _delay_ms(2000);
        //lora.setMode(STANDBY);

        int16_t temperature;
        uint32_t pressure;
        bmp.getAll(&temperature, &pressure);

        serial.println("Get all: ");
        serial.print("  Temperature: ");
        serial.println(temperature);
        serial.print("  Pressure: ");
        serial.println(pressure);


        char message[32];
        sprintf(message, "T%dP%ld", temperature, pressure);
        
        lora.setMode(STANDBY);
        lora.transmit((uint8_t*)message, 32, 0, 0);
        serial.println("Message sent");
        lora.setMode(SLEEP);
    }

    return 0;
}
