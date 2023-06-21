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




ISR(RTC_PIT_vect) {
    //cleat PIT flag
    RTC.PITINTFLAGS = 1;
}


/** @brief sleep MCU for specified time
 * 
 *
 * @param time how long to sleep the MCU for in seconds
 */
void sleep(uint16_t time) {
    //save pin state
    uint8_t porta_dir   = PORTA.DIR;
    uint8_t porta_state = PORTA.OUT;
    uint8_t portb_dir   = PORTB.DIR;
    uint8_t portb_state = PORTB.OUT;

    //TODO properly test all pin combinations

    //set all pins as outputs and pull them low (lowest power consumption)
    //PORTA.DIR = 0xFF;
    //PORTA.OUT = 0;
    //PORTB.DIR = 0xFF;
    //PORTB.OUT = 0;

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

    //good for lora
    //LORA_CS cannot be input
    pinMode(LORA_CS, OUTPUT);
    digitalWrite(LORA_CS, HIGH);
    //IRQ must be pulled LOW
    pinMode(LORA_IRQ, OUTPUT);
    digitalWrite(LORA_IRQ, LOW);

    //pinMode(LORA_RST, OUTPUT);
    //digitalWrite(LORA_RST, LOW);

    //good for lora and bmp
    //SCK and MOSI are pulled high on BMP sensor
    //MISO is pulled to ground
    //CS for BMP is pulled high
    pinMode(SCK, OUTPUT);
    digitalWrite(SCK, HIGH);
    pinMode(MOSI, OUTPUT);
    digitalWrite(MOSI, HIGH);
    pinMode(MISO, OUTPUT);
    digitalWrite(MISO, LOW);

    //good for bmp
    pinMode(BMP_CS, OUTPUT);
    digitalWrite(BMP_CS, HIGH);

    //disable ADC. Pretty much everything is already disabled (BOD, WDT and so on)
    ADC0.CTRLA = 0;

    //configure PIT with 1kHz clock
    while (RTC.STATUS);
    RTC.CTRLA = 0;                      //disable RTC (it does not work otherwise for some reason)
    RTC.CLKSEL = RTC_CLKSEL_INT1K_gc;   //set 1kHz clock
    while (RTC.PITSTATUS);
    RTC.PITINTCTRL = 1;                 //enable PIT interrupt
    RTC.PITCTRLA = RTC_PERIOD_CYC8192_gc | RTC_PITEN_bm;

    //enable interrupts, enable sleep, switch clock source, go to sleep
    SREG |= 0b10000000;
    SLPCTRL.CTRLA = SLEEP_MODE_STANDBY | SLEEP_ENABLED_gc;
    
    //while (time > 0) {
    //    if (time >= 32) {
    //        time -= 32;
    //        RTC.PITCTRLA = RTC_PERIOD_CYC32768_gc | RTC_PITEN_bm;
    //    }
    //    else if (time >= 16){
    //        time -= 16;
    //        RTC.PITCTRLA = RTC_PERIOD_CYC16384_gc | RTC_PITEN_bm;
    //    }
    //    else if (time >= 8){
    //        time -= 8;
    //        RTC.PITCTRLA = RTC_PERIOD_CYC8192_gc | RTC_PITEN_bm;
    //    }
    //    else if (time >= 4){
    //        time -= 4;
    //        RTC.PITCTRLA = RTC_PERIOD_CYC4096_gc | RTC_PITEN_bm;
    //    }
    //    else if (time >= 2){
    //        time -= 2;
    //        RTC.PITCTRLA = RTC_PERIOD_CYC2048_gc | RTC_PITEN_bm;
    //    }
    //    else if (time >= 1){
    //        time -= 1;
    //        RTC.PITCTRLA = RTC_PERIOD_CYC1024_gc | RTC_PITEN_bm;
    //    }
    //    else {
    //        break;
    //    }
    //    sleep_cpu();
    //}

    //switch to faster clock again
    sleep_cpu();

    //disable interrupts
    SREG &= 0b01111111;
    while (RTC.PITSTATUS);
    RTC.PITINTCTRL = 0;
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
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, CLKCTRL_PEN_bm);    //enable prescaler
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLA, CLKCTRL_CLKSEL_OSC20M_gc);  //set oscilator to 20MHz
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, CLKCTRL_PDIV_4X_gc | CLKCTRL_PEN_bm);   //set prescaler to 2
    #endif

    digitalWrite(LORA_CS, HIGH);

    pinMode(LORA_IRQ, INPUT);


    serial.begin(9600);
    spi.begin();
    serial.println("SPI init");

    serial.print("LoRa init: ");
    uint8_t init = lora.begin(&spi, 434.0F, 18U, 8U);
    serial.println(init);
    if (init != 0) {
        serial.println(("SX1278 not found"));
        while (true);
    }
    lora.setMode(SX1278_SLEEP);

    serial.print("BMP init: ");
    init = bmp.begin(&spi);
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
        serial.begin(9600);
        spi.begin();

        
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

        
        lora.transmit((uint8_t*)data, 8);
        lora.setMode(SX1278_SLEEP);

        serial.println("data probably sent");

        serial.end();
        spi.end();

        sleep(4);
    }

    return 0;
}
