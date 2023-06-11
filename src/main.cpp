/*
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
*/


#include <avr/io.h>
#include <util/delay.h>

#include "SPIClass.hpp"
#include "digitalIO.hpp"

//SX1278 registers
#define REG_FIFO                                0x00
#define REG_OP_MODE                             0x01
#define REG_FRF_MSB                             0x06
#define REG_FRF_MID                             0x07
#define REG_FRF_LSB                             0x08
#define REG_PA_CONFIG                           0x09
#define REG_OCP                                 0x0B
#define REG_FIFO_ADDR_PTR                       0x0D
#define REG_FIFO_TX_BASE_ADDR                   0x0E
#define REG_IRQ_FLAGS                           0x12
#define REG_MODEM_CONFIG_1                      0x1D
#define REG_MODEM_CONFIG_2                      0x1E
#define REG_PREAMBLE_MSB                        0x20
#define REG_PREAMBLE_LSB                        0x21
#define REG_PAYLOAD_LENGTH                      0x22
#define REG_HOP_PERIOD                          0x24
#define REG_MODEM_CONFIG_3                      0x26
#define REG_DETECT_OPTIMIZE                     0x31
#define REG_DETECTION_THRESHOLD                 0x37
#define REG_SYNC_WORD                           0x39
#define REG_DIO_MAPPING_1                       0x40
#define REG_VERSION                             0x42
#define REG_PA_DAC                              0x4D

//SX1278 chip version
#define CHIP_VERSION                            0x12

//modem settings/modes
#define SLEEP                                   0b00000000
#define STANDBY                                 0b00000001
#define TX                                      0b00000011
#define LORA                                    0b10000000

//power config
#define PA_SELECT_BOOST                         0b10000000  //  7     7   PA_BOOST pin output, power limited to +20 dBm
#define MAX_POWER                               0b01110000  //  6     4   max power: P_max = 10.8 + 0.6*MAX_POWER [dBm]; P_max(MAX_POWER = 0b111) = 15 dBm
#define PA_BOOST_OFF                            0b00000100  //  2     0   PA_BOOST disabled
//overcurrent protection
#define OCP_OFF                                 0b00000000  //  5     5   PA overload current protection disabled
#define OCP_ON                                  0b00100000  //  5     5   PA overload current protection enabled
#define OCP_TRIM                                0b00001011  //  4     0   OCP current: I_max(OCP_TRIM = 0b1011) = 100 mA

//hopping config
#define HOP_PERIOD_OFF                          0b00000000  //  7     0   number of periods between frequency hops; 0 = disabled

//config 1
#define BW_7_80_KHZ                             0b00000000  //  7     4   bandwidth:  7.80 kHz
#define BW_10_40_KHZ                            0b00010000  //  7     4               10.40 kHz
#define BW_15_60_KHZ                            0b00100000  //  7     4               15.60 kHz
#define BW_20_80_KHZ                            0b00110000  //  7     4               20.80 kHz
#define BW_31_25_KHZ                            0b01000000  //  7     4               31.25 kHz
#define BW_41_70_KHZ                            0b01010000  //  7     4               41.70 kHz
#define BW_62_50_KHZ                            0b01100000  //  7     4               62.50 kHz
#define BW_125_00_KHZ                           0b01110000  //  7     4               125.00 kHz
#define BW_250_00_KHZ                           0b10000000  //  7     4               250.00 kHz
#define BW_500_00_KHZ                           0b10010000  //  7     4               500.00 kHz
#define CR_4_5                                  0b00000010  //  3     1   error coding rate:  4/5
#define CR_4_6                                  0b00000100  //  3     1                       4/6
#define CR_4_7                                  0b00000110  //  3     1                       4/7
#define CR_4_8                                  0b00001000  //  3     1                       4/8
#define HEADER_EXPL_MODE                        0b00000000  //  0     0   explicit header mode
#define HEADER_IMPL_MODE                        0b00000001  //  0     0   implicit header mode

//config 2
#define SF_6                                    0b01100000  //  7     4   spreading factor:   64 chips/bit
#define SF_7                                    0b01110000  //  7     4                       128 chips/bit
#define SF_8                                    0b10000000  //  7     4                       256 chips/bit
#define SF_9                                    0b10010000  //  7     4                       512 chips/bit
#define SF_10                                   0b10100000  //  7     4                       1024 chips/bit
#define SF_11                                   0b10110000  //  7     4                       2048 chips/bit
#define SF_12                                   0b11000000  //  7     4                       4096 chips/bit
#define TX_MODE_SINGLE                          0b00000000  //  3     3   single TX
#define TX_MODE_CONT                            0b00001000  //  3     3   continuous TX
#define RX_TIMEOUT_MSB                          0b00000000  //  1     0
#define RX_CRC_MODE_OFF                         0b00000000  //  2     2   CRC disabled
#define RX_CRC_MODE_ON                          0b00000100  //  2     2   CRC enabled

//config 3
#define LOW_DATA_RATE_OPT_OFF                   0b00000000  //  3     3   low data rate optimization disabled
#define LOW_DATA_RATE_OPT_ON                    0b00001000  //  3     3   low data rate optimization enabled
#define AGC_AUTO_OFF                            0b00000000  //  2     2   LNA gain set by REG_LNA
#define AGC_AUTO_ON                             0b00000100  //  2     2   LNA gain set by internal AGC loop

//Detection optimize
#define DETECT_OPTIMIZE_SF_7_12                 0b00000011  //  2     0   SF7 to SF12 detection optimization
//Detection treshold
#define DETECTION_THRESHOLD_SF_7_12             0b00001010  //  7     0   SF7 to SF12 detection threshold

//pinmaps
#define DIO0_LORA_TX_DONE                       0b01000000  //  7     6

#define FIFO_TX_BASE_ADDR_MAX                   0b00000000  //  7     0   allocate the entire FIFO buffer for TX only


//Pin defines
#define RST_PIN   PB1
#define RST_PORT  PORTB
#define RST_DDR   DDRB
#define RST_IN    PINB
#define CS_PIN    PB2
#define CS_PORT   PORTB
#define CS_DDR    DDRB
#define CS_IN     PINB
#define IRQ_PIN   PD7
#define IRQ_PORT  PORTD
#define IRQ_DDR   DDRD
#define IRQ_IN    PIND
#define GPIO_PIN  PD6
#define GPIO_PORT PORTD
#define GPIO_DDR  DDRD
#define GPIO_IN   PIND

#define LED_PIN  PB0
#define LED_PORT PORTB
#define LED_DDR  DDRB
#define LED_IN   PINB

//#define FREQUENCY 434.0F

float freq = 434.0F;
float bw = 125.0F;
uint8_t sf = 9U;
uint8_t cr = 7U;
uint8_t syncWord = 18U;
int8_t power = 10;
uint16_t preambleLength = 8U;
uint8_t gain = 0U;
//uint8_t cs  = 10;
//uint8_t LORA_IRQ  = 2;
//uint8_t LORA_GPIO = 3;
//uint8_t LORA_RST = 9;

//SPISettings SPI_settings(2000000, MSBFIRST, SPI_MODE0);
SPIClass spi = SPIClass();


void blink(uint8_t cnt) {
    for (int i = 0; i < cnt; i++){
        digitalWrite(LED_PORT, LED_PIN, HIGH);
        _delay_ms(50);
        digitalWrite(LED_PORT, LED_PIN, LOW);
        _delay_ms(50);
    }
}


uint8_t LoRaRead(uint8_t addr) {
    digitalWrite(CS_PORT, CS_PIN, LOW);
    uint8_t reg = spi.readRegister(addr);
    
    digitalWrite(CS_PORT, CS_PIN, HIGH);
    return reg;
}

void LoRaWrite(uint8_t addr, uint8_t data) {
    digitalWrite(CS_PORT, CS_PIN, LOW);
    spi.writeRegister(addr, data);
    digitalWrite(CS_PORT, CS_PIN, HIGH);
}

void LoRaSet(uint8_t addr, uint8_t data, uint8_t mask_lsb = 0, uint8_t mask_msb = 7) {
    digitalWrite(CS_PORT, CS_PIN, LOW);
    uint8_t reg = spi.readRegister(addr);
    digitalWrite(CS_PORT, CS_PIN, HIGH);

    uint8_t mask = ~(0xFF >> (8 - mask_lsb) | 0xFF << (mask_msb + 1));
    data = (reg & ~mask) | (data & mask);

    digitalWrite(CS_PORT, CS_PIN, LOW);
    spi.writeRegister(addr, data);
    digitalWrite(CS_PORT, CS_PIN, HIGH);
}


void reset() {
    pinMode(RST_DDR, RST_PIN, OUTPUT);
    digitalWrite(RST_PORT, RST_PIN, LOW);
    _delay_ms(1);
    digitalWrite(RST_PORT, RST_PIN, HIGH);
    _delay_ms(5);
}


void setMode(uint8_t mode) {
    LoRaSet(REG_OP_MODE, mode, 0, 2);
}


void setModemMode(uint8_t modem) {
    setMode(SLEEP);

    LoRaSet(REG_OP_MODE, modem, 7, 7);

    setMode(STANDBY);
}

void setCurrentLimit(uint8_t current_limit) {
    setMode(STANDBY);

    uint8_t raw;
    uint8_t reg = LoRaRead(REG_OCP);

    if(current_limit == 0) {
        // limit set to 0, disable OCP
        reg = (reg & 0b11011111) | OCP_OFF;
    }
    else if(current_limit <= 120) {
        raw = (current_limit - 45) / 5;
        reg = (reg & 0b11000000) | OCP_ON | raw;
    }
    else if(current_limit <= 240) {
        raw = (current_limit + 30) / 10;
        reg = (reg & 0b11000000) | OCP_ON | raw;
    }

    LoRaWrite(REG_OCP, reg);
}


void startTransmission(uint8_t* data, uint8_t length, uint8_t addr) {
    setMode(STANDBY);

    // set DIO mapping
    LoRaSet(REG_DIO_MAPPING_1, DIO0_LORA_TX_DONE, 6, 7);

    //clear interrupt flags
    LoRaWrite(REG_IRQ_FLAGS, 0xFF);

    //set packet length
    LoRaSet(REG_PAYLOAD_LENGTH, length);

    // set FIFO pointers
    LoRaSet(REG_FIFO_TX_BASE_ADDR, FIFO_TX_BASE_ADDR_MAX);
    LoRaSet(REG_FIFO_ADDR_PTR, FIFO_TX_BASE_ADDR_MAX);

  // write packet to FIFO
    digitalWrite(CS_PORT, CS_PIN, LOW);
    spi.writeRegisterBurst(REG_FIFO, data, length);
    digitalWrite(CS_PORT, CS_PIN, HIGH);

    // start transmission
    setMode(TX);
}

void finishTransmit() {
    blink(3);
    ////Serial.println(SPIReadRegister(LORA_CS, REG_IRQ_FLAGS), 2);
    // clear interrupt flags
    
    LoRaWrite(REG_IRQ_FLAGS, 0xFF);
    
    setMode(STANDBY);

    blink(4);
}

void transmit(uint8_t* data, uint8_t length, uint8_t addr) {
    blink(2);

    setMode(STANDBY);

    // calculate timeout (150 % of expected time-on-air)
    //double symbolLength = (double)(uint32_t(1) << sf) / (double)bw;
    //double de = 0;
    //double ih = 0;
    //double crc = 3;
    //double n_pre = (double)preambleLength;
    //uint8_t pl = length;
    //uint8_t crc_en = 1;

    //double top = 8 * pl - 4 * sf + 28 + 16 * crc_en - 20 * ih;
    //double bottom = 4 * (sf - 2 * de);
    
    //double payload_sim = 8.0 + max(ceil(top / bottom) * (crc + 4), 0.0);
    //uint32_t time_on_air = ceil(symbolLength * (n_pre + payload_sim + 4.25)) * 1000;
    //uint32_t timeout = time_on_air * 1.5;

    startTransmission(data, length, addr);

    _delay_ms(500);

    blink(1);
    while(!digitalRead(IRQ_IN, IRQ_PIN));
    blink(5);

    _delay_ms(500);

    blink(2);
    //Serial.println(LoRaRead(REG_IRQ_FLAGS), 2);

    finishTransmit();
}


int main() {
    spi.init();
    //SPI.begin();
    //Serial.begin(115200);

    // initialize SX1278 with default settings
    //Serial.print("[SX1278] Initializing ... ");

    // set module properties
    pinMode(CS_DDR, CS_PIN, OUTPUT);
    digitalWrite(CS_PORT, CS_PIN, HIGH);
    
    pinMode(IRQ_DDR, IRQ_PIN, INPUT);
    pinMode(GPIO_DDR, GPIO_PIN, INPUT);
    pinMode(LED_DDR, LED_PIN, OUTPUT);

    blink(10);
    _delay_ms(1000);

    reset();

    _delay_ms(10);
    // check version register
    int16_t version = LoRaRead(REG_VERSION);
    if(version != CHIP_VERSION) {
        //Serial.println("No SX127x found!");
        while(true);
    }
    //Serial.println("SX1278 chip found!");
    //Serial.println(LoRaRead(REG_VERSION), HEX);

    //set mode to standby
    setMode(STANDBY);

    //Serial.print("OpModeReg: 0x");
    //Serial.println(LoRaRead(REG_OP_MODE), HEX);

    //turn off frequency hopping
    LoRaWrite(REG_HOP_PERIOD, HOP_PERIOD_OFF);

    //set lora mode
    setModemMode(LORA);
    //Serial.print("OpModeReg: 0x");
    //Serial.println(LoRaRead(REG_OP_MODE), HEX);

    //set LoRa sync word
    LoRaWrite(REG_SYNC_WORD, syncWord);

    // set over current protection
    //Serial.print("RegOcp: 0x");
    //Serial.println(LoRaRead(REG_OCP), HEX);
    setCurrentLimit(60);
    //Serial.print("RegOcp: 0x");
    //Serial.println(LoRaRead(REG_OCP), HEX);

    // set preamble length
    //Serial.print("RegPreambleMsb: 0x");
    //Serial.println(LoRaRead(REG_PREAMBLE_MSB), HEX);
    //Serial.print("RegPreambleLsb: 0x");
    //Serial.println(LoRaRead(REG_PREAMBLE_LSB), HEX);
    
    LoRaWrite(REG_PREAMBLE_MSB, (uint8_t)((preambleLength >> 8) & 0xFF));
    LoRaWrite(REG_PREAMBLE_LSB, (uint8_t)(preambleLength & 0xFF));

    //Serial.print("RegPreambleMsb: 0x");
    //Serial.println(LoRaRead(REG_PREAMBLE_MSB), HEX);
    //Serial.print("RegPreambleLsb: 0x");
    //Serial.println(LoRaRead(REG_PREAMBLE_LSB), HEX);

    //125khz bandwidth
    //is already default
    LoRaSet(REG_MODEM_CONFIG_1, 0b01110000, 4, 7);
    //Serial.print("Config1: 0x");
    //Serial.println(LoRaRead(REG_MODEM_CONFIG_1), HEX);


    //434 frequency
    //default already is 434
    uint32_t FRF = (freq * (uint32_t(1) << 19)) / 32.0;
    LoRaSet(REG_FRF_MSB, (FRF & 0xFF0000) >> 16);
    LoRaSet(REG_FRF_MID, (FRF & 0x00FF00) >> 8);
    LoRaSet(REG_FRF_LSB, FRF & 0x0000FF);

    //Serial.print("FRF_MSB: 0x");
    //Serial.println(LoRaRead(REG_FRF_MSB), HEX);
    //Serial.print("FRF_MID: 0x");
    //Serial.println(LoRaRead(REG_FRF_MID), HEX);
    //Serial.print("FRF_LSB: 0x");
    //Serial.println(LoRaRead(REG_FRF_LSB), HEX);


    //set spreading factor;
    //is also default already
    LoRaSet(REG_MODEM_CONFIG_1, HEADER_EXPL_MODE, 0, 0);
    LoRaSet(REG_MODEM_CONFIG_2, SF_9 | TX_MODE_SINGLE, 4, 7);
    LoRaSet(REG_DETECT_OPTIMIZE, DETECT_OPTIMIZE_SF_7_12, 0, 2);
    LoRaSet(REG_DETECTION_THRESHOLD, DETECTION_THRESHOLD_SF_7_12);

    //set error coding rate to 4/7
    LoRaSet(REG_MODEM_CONFIG_1, CR_4_7, 1, 3);

    //state = setOutputPower(power);
    LoRaSet(REG_PA_CONFIG, PA_SELECT_BOOST, 7, 7);
    LoRaSet(REG_PA_CONFIG, MAX_POWER | (power - 2), 0, 6);
    LoRaSet(REG_PA_DAC, PA_BOOST_OFF, 0, 2);

    //enable automatic gain control
    LoRaSet(REG_MODEM_CONFIG_3, AGC_AUTO_ON, 2, 2);

    //enable crc
    LoRaSet(REG_MODEM_CONFIG_2, RX_CRC_MODE_ON, 2, 2);

    //configure low data rate
    float symbolLength = (float)(uint32_t(1) << sf) / (float)bw;
    //Serial.println(symbolLength);
    if(symbolLength >= 16.0) {
        LoRaSet(REG_MODEM_CONFIG_3, LOW_DATA_RATE_OPT_ON, 3, 3);
    }
    else {
        LoRaSet(REG_MODEM_CONFIG_3, LOW_DATA_RATE_OPT_OFF, 3, 3);
    }

    while(true) {
        //Serial.print(F("[SX1278] Transmitting packet ... "));

        // you can transmit C-string or Arduino string up to
        // 256 characters long
        // NOTE: transmit() is a blocking method!
        //       See example SX127x_Transmit_Interrupt for details
        //       on non-blocking transmission method.
        //byte data[] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF};
        char message[] = "test";
        transmit((uint8_t*)message, 5, 0);
        
        //digitalWrite(LED_PORT, LED_PIN, digitalRead(IRQ_PORT, IRQ_PIN));

        // wait for a second before transmitting again
        _delay_ms(100);
    }
    return 0;
}
