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
#include <avr/interrupt.h>
#include <avr/sleep.h>
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
//Serial serial = Serial();
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


void blink() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
    _delay_ms(200);
    digitalWrite(LED_PIN, LOW);
}

ISR(RTC_CNT_vect) {
    //cleat PIT flag
    //while (RTC.PITSTATUS);
    //RTC.PITINTFLAGS = 0xFF;
    RTC.INTFLAGS = RTC_OVF_bm;
}

volatile uint8_t wtf = 0;

/** @brief sleep MCU for specified number of seconds
 * 
 *
 * @param time how long to sleep the MCU for in seconds
 */
void sleep() {
    //save pin state
    uint8_t porta_dir   = PORTA.DIR;
    uint8_t porta_state = PORTA.OUT;
    uint8_t portb_dir   = PORTB.DIR;
    uint8_t portb_state = PORTB.OUT;

    //TODO set the pins using single line

    //set all pins as outputs and pull them low (lowest power consumption)
    //PORTA.DIR = 0xFF;
    //PORTA.OUT = 0;
    //PORTB.DIR = 0xFF;
    //PORTB.OUT = 0;

    //LORA_CS cannot be input
    pinMode(LORA_CS, OUTPUT);
    digitalWrite(LORA_CS, HIGH);
    //IRQ must be pulled LOW
    pinMode(LORA_IRQ, OUTPUT);
    digitalWrite(LORA_IRQ, LOW);

    //SCK and MOSI are pulled high by BMP
    pinMode(SCK, OUTPUT);
    digitalWrite(SCK, HIGH);
    pinMode(MOSI, OUTPUT);
    digitalWrite(MOSI, HIGH);
    //MISO is pulled to ground by BMP
    pinMode(MISO, OUTPUT);
    digitalWrite(MISO, LOW);

    //CS is pulled high by BMP
    pinMode(BMP_CS, OUTPUT);
    digitalWrite(BMP_CS, HIGH);

    //all other pins must be outputs pulled LOW for lowest power consumption
    pinMode(PA0, OUTPUT);
    digitalWrite(PA0, LOW);
    pinMode(PB0, OUTPUT);
    digitalWrite(PB0, LOW);
    pinMode(PB1, OUTPUT);
    digitalWrite(PB1, LOW);
    pinMode(PB2, OUTPUT);
    digitalWrite(PB2, LOW);
    pinMode(PB3, OUTPUT);
    digitalWrite(PB3, LOW);


    //disable ADC. Pretty much everything should be already disabled (BOD, WDT and so on)
    ADC0.CTRLA = 0;

    SREG |= 0b10000000;
    sleep_cpu();
    SREG &= 0b01111111;

    //disable RTC
    //while(RTC.STATUS & RTC_CTRLABUSY_bm)
    //RTC.CTRLA = 0;

    //restore all pins
    PORTA.DIR = porta_dir;
    PORTA.OUT = porta_state;
    PORTB.DIR = portb_dir;
    PORTB.OUT = portb_state;
}

int main() {
    #if defined (__AVR_ATtiny1624__)
    //Run at 5MHz
    //_PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, CLKCTRL_PEN_bm);    //enable prescaler
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, CLKCTRL_PDIV_4X_gc | CLKCTRL_PEN_bm);   //set prescaler to 4 -> 5MHz
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLA, CLKCTRL_CLKSEL_OSC20M_gc);  //set oscilator to 20MHz
    #endif


    //enable RTC and start counting
    RTC.CLKSEL = RTC_CLKSEL_INT1K_gc;   //set 1kHz clock
    while (RTC.STATUS & RTC_PERBUSY_bm);
    RTC.PER = 5 - 1;  //5 second overflow
    //set RTC to overflow mode
    while (RTC.STATUS & RTC_CNTBUSY_bm);
    RTC.CNT = 0;
    RTC.INTCTRL = RTC_OVF_bm;
    //enable RTC, enable it in standby, set prescaler to 1024 clock -> 1s per cnt
    while (RTC.STATUS & RTC_CTRLABUSY_bm);
    RTC.CTRLA = RTC_RUNSTDBY_bm | RTC_PRESCALER_DIV1024_gc | RTC_RTCEN_bm;

    //enable standby mode
    SLPCTRL.CTRLA = SLEEP_MODE_STANDBY | SLEEP_ENABLED_gc;


    //initialize everything
    spi.begin();
    uint8_t init = lora.begin(&spi, 434.0F, 18U, 8U);
    if (init != 0) {
        while (true);
    }
    lora.setMode(SX1278_SLEEP);

    init = bmp.begin(&spi);
    if (init != 0) {
        while (true);
    }


    //main loop
    while(true) {
        //start spi
        spi.begin();

        //calcualte all data
        uint8_t data[8] = {0};
        uint16_t vdd = getVDD();
        data[0] = (uint8_t)(vdd);
        data[1] = (uint8_t)(vdd >> 8);
        bmp.getAll((int16_t*)(data + 2), (uint32_t*)(data + 4));
        int16_t temp = (int16_t)data[3] << 8 | (int16_t)data[2];
        uint32_t press = (uint32_t)data[7] << 24 | (uint32_t)data[6] << 16 | (uint32_t)data[5] << 8 | data[4];

        //send the data
        lora.transmit((uint8_t*)data, 8);

        //disable everything
        lora.setMode(SX1278_SLEEP);
        spi.end();

        //sleep for 5 seconds
        sleep();
    }

    return 0;
}
