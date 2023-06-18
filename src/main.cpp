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


//TODO sleep with timer only

#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>

#include "SPIClass.hpp"
#include "digitalIO.hpp"
#include "SX1278.hpp"
#include "pinmap.hpp"
//#include "registers.hpp"
#include "serial.hpp"
#include "bmx280.hpp"


#define DELAY(x) _delay_ms(x)

#if defined (__AVR_ATmega328P__)
#  define LORA_RST  D9
#  define LORA_CS   D10
#  define LORA_IRQ  D7
#  define LED_PIN   D8
#  define BMP_CS    D3

#elif defined (__AVR_ATtiny1624__)
#  define LORA_RST  PA5
#  define LORA_CS   PA4
#  define LORA_IRQ  PA6 //DIO0
#  define LED_PIN   PB0
#  define BMP_CS    PA7

#endif


SPIClass spi = SPIClass();
SX1278 lora = SX1278(LORA_CS, LORA_RST, LORA_IRQ);
Serial serial = Serial();
BMX280 bmp = BMX280(BMP_CS);


//inspired by http://www.technoblogy.com/show?3K82
//use ADC and internal refference to calculate VDD
uint16_t getVDD() {

    //adc ref = vdd
    //adc muxpos =  1.024v - AC0

    VREF.CTRLA   = VREF_AC0REFSEL_1V024_gc;         //set AC0 reference to 1.024V
    AC0.DACREF   = 0xFF;                            //set AC.DACREF to to 1.024V on output
    AC0.CTRLA    = AC_LPMODE_bm | AC_ENABLE_bm;     //enable AC in low power mode
    
    ADC0.CTRLA   = ADC_ENABLE_bm;                                       //enable ADC
    ADC0.CTRLC   = ADC_REFSEL_VDD_gc | (10 << ADC_TIMEBASE_gp);        //set ADC reference to VDD and number of CLK_PER per 1us (10MHz * 0.000001 = 10)
    ADC0.MUXPOS  = ADC_MUXPOS_DACREF0_gc;                               //set ADC input as AC DACREF MUXPOS = ACV
    ADC0.CTRLB   = ADC_PRESC_DIV10_gc;                                  //1MHz ADC clock
    ADC0.COMMAND = ADC_MODE_SINGLE_12BIT_gc | ADC_START_IMMEDIATE_gc;    //start single 8bit conversion

    //wait for conversion to end
    while (ADC0.STATUS & ADC_ADCBUSY_bm);

    //AC0.DACREF = VREF * (ADC0.RESULT / 2^12)
    //VREF = (AC0.DACREF * 2^12) / ADC0.RESULT
    //VREF = (1.024 * 2^12) / ADC0.RESULT
    //VREF = 4194.304 / ADC0.RESULT
    //VREF/100 = 419430.4 / ADC0.RESULT

    return 419430 / ADC0.RESULT;
}




int main() {
    #if defined (__AVR_ATtiny1624__)
    //Run at 10MHz
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, CLKCTRL_PEN_bm);    //enable prescaler
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLA, CLKCTRL_CLKSEL_OSC20M_gc);  //set oscilator to 20MHz
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, CLKCTRL_PDIV_2X_gc | CLKCTRL_PEN_bm);   //set prescaler to 2
    #endif

    digitalWrite(LORA_CS, HIGH);

    pinMode(LORA_IRQ, INPUT);


    serial.init(9600);
    spi.init();
    serial.println("SPI init");

    serial.print("LoRa init: ");
    uint8_t init = lora.init(&spi, 434.0F, 18U, 8U);
    serial.println(init);
    if (init != 1) {
        serial.println(("SX1278 not found"));
        while (true);
    }
    lora.setMode(SX1278_SLEEP);

    serial.print("BMP init: ");
    init = bmp.init(&spi);
    serial.println(init);
    if (init != 0) {
        serial.print(("BMP280 not found: "));
        serial.println(init);
        while (true);
    }
    bmp.setMode(BMX280_SLEEP);
    serial.print("BMP ID: 0x");
    serial.printlnHex(bmp.getId());

    while(true) {
        uint8_t data[8] = {0};
        uint16_t vdd = getVDD();
        data[0] = (uint8_t)(vdd);
        data[1] = (uint8_t)(vdd >> 8);
        bmp.getAll((int16_t*)(data + 2), (uint32_t*)(data + 4));

        int16_t temp = (int16_t)data[3] << 8 | (int16_t)data[2];
        uint32_t press = (uint32_t)data[7] << 24 | (uint32_t)data[6] << 16 | (uint32_t)data[5] << 8 | data[4];

        serial.println("Get all: ");
        serial.print("  Temperature: ");
        serial.println(temp);
        serial.print("  Pressure: ");
        serial.println(press);
        serial.print("  Voltage:");
        serial.println(vdd);

        lora.setMode(SX1278_STANDBY);
        lora.transmit((uint8_t*)data, 8, 0);
        lora.setMode(SX1278_SLEEP);
        serial.println("data probably sent");

        _delay_ms(5000);
    }

    return 0;
}
