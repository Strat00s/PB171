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


#include "digitalIO.hpp"

#include "pinmap.hpp"
#include "serial.hpp"
//#include "registers.hpp"



#define DELAY(x) _delay_ms(x)

#define LED_PIN   PB0


Serial serial = Serial();



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
    //configure PIT with 1kHz clock
    while (RTC.STATUS & RTC_CTRLABUSY_bm);
    RTC.CTRLA = RTC_PRESCALER_DIV1_gc;  //disable RTC (it does not work otherwise for some reason)
    RTC.CLKSEL = RTC_CLKSEL_INT1K_gc;   //set 1kHz clock
    RTC.PITINTCTRL = RTC_PI_bm;                 //enable PIT interrupt

    SLPCTRL.CTRLA = SLEEP_MODE_PWR_DOWN | SLEEP_ENABLED_gc;


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
            RTC.PITCTRLA = RTC_PERIOD_CYC2048_gc | RTC_PITEN_bm;
        }
        else if (time >= 2){
            time -= 2;
            while (RTC.PITSTATUS);
            RTC.PITCTRLA = RTC_PERIOD_CYC8192_gc | RTC_PITEN_bm;
        }
        else {
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
}


int main() {
    #if defined (__AVR_ATtiny1624__)
    //Run at 5MHz
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, CLKCTRL_PDIV_4X_gc | CLKCTRL_PEN_bm);   //set prescaler to 4 -> 5MHz
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLA, CLKCTRL_CLKSEL_OSC20M_gc);  //set oscilator to 20MHz
    #endif

    serial.begin(9600);

    SLPCTRL.CTRLA = SLEEP_MODE_PWR_DOWN | SLEEP_ENABLED_gc;
    
    while(true) {

        //while (RTC.STATUS & RTC_CTRLABUSY_bm);
        //RTC.CTRLA = 1;//RTC_PRESCALER_DIV1_gc;                      //disable RTC (it does not work otherwise for some reason)
        //RTC.CLKSEL = RTC_CLKSEL_INT1K_gc;   //set 1kHz clock
        //RTC.PITINTCTRL = RTC_PI_bm;                 //enable PIT interrupt

        //sleep for 4 seconds
        //blink();
        //while (RTC.PITSTATUS);
        //RTC.PITCTRLA = RTC_PERIOD_CYC4096_gc | RTC_PITEN_bm;

        //while(!(RTC.PITINTFLAGS & 0b00000001));
        //serial.print("ITINTFLAGS: 0x"); serial.printlnHex(RTC.PITINTFLAGS);
        //RTC.PITINTFLAGS = 0xFF;
        //SREG |= 0b10000000;
        //sleep_cpu();
        //SREG &= 0b01111111;
        //while(RTC.PITSTATUS)
        //RTC.PITCTRLA = 0;

        //digitalWrite(LED_PIN, HIGH);
        //_delay_ms(400);
        //digitalWrite(LED_PIN, LOW);
        //_delay_ms(100);
        //RTC.PITINTCTRL = 0;
        sleep(6);
    }

    return 0;
}
