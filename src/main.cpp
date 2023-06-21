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

ISR(RTC_PIT_vect) {
    //cleat PIT flag
    //while (RTC.PITSTATUS);
    RTC.PITINTFLAGS = 0xFF;
}

volatile uint8_t wtf = 0;

/** @brief sleep MCU for specified number of seconds
 * 
 *
 * @param time how long to sleep the MCU for in seconds
 */
void sleep(volatile uint16_t time) {
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
    

    //configure PIT with 1kHz clock
    while (RTC.STATUS & RTC_CTRLABUSY_bm);
    RTC.CTRLA = RTC_PRESCALER_DIV1_gc;                      //disable RTC (it does not work otherwise for some reason)
    RTC.CLKSEL = RTC_CLKSEL_INT1K_gc;   //set 1kHz clock
    //while (RTC.PITSTATUS);
    RTC.PITINTCTRL = RTC_PI_bm;                 //enable PIT interrupt
    //RTC.PITDBGCTRL = 1;
    //RTC.PITCTRLA = RTC_PERIOD_CYC8192_gc | RTC_PITEN_bm;

    //RTC.PITINTCTRL = RTC_PI_bm;                 //enable PIT interrupt

    //enable interrupts, enable sleep, switch clock source, go to sleep
    SLPCTRL.CTRLA = SLEEP_MODE_PWR_DOWN | SLEEP_ENABLED_gc;
    while ( (CLKCTRL.MCLKSTATUS & CLKCTRL_SOSC_bm) > 0)
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLA, CLKCTRL_CLKSEL_OSCULP32K_gc);  //select 32khz clock
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, 0); //disable prescaler


    while (time > 0) {
        blink();
        if (time >= 32) {
            time -= 32;
            while (RTC.PITSTATUS);
            RTC.PITCTRLA = RTC_PERIOD_CYC32768_gc | RTC_PITEN_bm;
        }
        else if (time >= 16){
            time -= 16;
            while (RTC.PITSTATUS);
            RTC.PITCTRLA = RTC_PERIOD_CYC16384_gc | RTC_PITEN_bm;
        }
        else if (time >= 8){
            time -= 8;
            while (RTC.PITSTATUS);
            RTC.PITCTRLA = RTC_PERIOD_CYC8192_gc | RTC_PITEN_bm;
        }
        else if (time >= 4){
            time -= 4;
            while (RTC.PITSTATUS);
            RTC.PITCTRLA = RTC_PERIOD_CYC4096_gc | RTC_PITEN_bm;
        }
        else if (time >= 2){
            time -= 2;
            while (RTC.PITSTATUS);
            RTC.PITCTRLA = RTC_PERIOD_CYC2048_gc | RTC_PITEN_bm;
        }
        else {
            wtf = ~wtf;
            if (wtf)
                break;
            time -= 1;
            while (RTC.PITSTATUS);
            RTC.PITCTRLA = RTC_PERIOD_CYC1024_gc | RTC_PITEN_bm;
        }

        SREG |= 0b10000000;
        sleep_cpu();
        SREG &= 0b01111111;

        //disable PIT
        while(RTC.PITSTATUS)
        RTC.PITCTRLA = 0;
    }

    RTC.PITINTCTRL = 0;


    //change back to main clock
    while ( (CLKCTRL.MCLKSTATUS & CLKCTRL_SOSC_bm) > 0)
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLA, CLKCTRL_CLKSEL_OSC20M_gc);
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, CLKCTRL_PDIV_4X_gc | CLKCTRL_PEN_bm);
    //disable interrupts
    //SREG &= 0b01111111;
    //while (RTC.PITSTATUS);
    //RTC.PITINTCTRL = 0;
    //RTC.PITCTRLA &= ~(RTC_PITEN_bm); 

    //DONE enable what is needed
    PORTA.DIR = porta_dir;
    PORTA.OUT = porta_state;
    PORTB.DIR = portb_dir;
    PORTB.OUT = portb_state;
}

/*
int main() {
    //5mhz
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, CLKCTRL_PEN_bm);    //enable prescaler
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLA, CLKCTRL_CLKSEL_OSC20M_gc);  //set oscilator to 20MHz
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, CLKCTRL_PDIV_2X_gc | CLKCTRL_PEN_bm);   //set prescaler to 4

    RTC.PITCTRLA = RTC_PITEN_bm;

    spi.begin();
    lora.begin(&spi, 434.0F, 18U, 8U);
    lora.setMode(SX1278_SLEEP);
    bmp.begin(&spi);

    while (true) {
        //start spi
        //spi.begin();

        //calculate everything
        uint8_t data[8] = {0};
        uint16_t vdd = getVDD();
        data[0] = (uint8_t)(vdd);
        data[1] = (uint8_t)(vdd >> 8);
        bmp.getAll((int16_t*)(data + 2), (uint32_t*)(data + 4));
        int16_t temp = (int16_t)data[3] << 8 | (int16_t)data[2];
        uint32_t press = (uint32_t)data[7] << 24 | (uint32_t)data[6] << 16 | (uint32_t)data[5] << 8 | data[4];

        //transmit it
        lora.transmit((uint8_t*)data, 8);

        //DELAY(4000);

        //go to sleep
        //lora.setMode(SX1278_SLEEP);
        //spi.end();

        //sleep(4);
    }
    
}
*/

int main() {
    #if defined (__AVR_ATtiny1624__)
    //Run at 5MHz
    //_PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, CLKCTRL_PEN_bm);    //enable prescaler
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, CLKCTRL_PDIV_4X_gc | CLKCTRL_PEN_bm);   //set prescaler to 4 -> 5MHz
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLA, CLKCTRL_CLKSEL_OSC20M_gc);  //set oscilator to 20MHz
    #endif

    //enable pit timer to start the prescale counter immediately
    //while (RTC.PITSTATUS)
    //RTC.PITCTRLA = 1;
    //while (RTC.STATUS & RTC_CTRLABUSY_bm);
    //RTC.CTRLA = 0;

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
        sleep(6);
    }

    return 0;
}
