
/*
   RadioLib SX127x Receive Example

   This example listens for LoRa transmissions using SX127x Lora modules.
   To successfully receive data, the following settings have to be the same
   on both transmitter and receiver:
    - carrier frequency
    - bandwidth
    - spreading factor
    - coding rate
    - sync word
    - preamble length

   Other modules from SX127x/RFM9x family can also be used.

   For default module settings, see the wiki page
   https://github.com/jgromes/RadioLib/wiki/Default-configuration#sx127xrfm9x---lora-modem

   For full API reference, see the GitHub Pages
   https://jgromes.github.io/RadioLib/
*/

//#define RECV

// include the library
//#include <RadioLib.h>

// SX1278 has the following connections:
// NSS pin:   10
// DIO0 pin:  2
// RESET pin: 9
// DIO1 pin:  3
//SX1278 radio = new Module(10, 2, 9, 3);

// or using RadioShield
// https://github.com/jgromes/RadioShield
//SX1278 radio = RadioShield.ModuleA;
#include <math.h>

#ifdef RECV
#include <RadioLib.h>
SX1278 radio = new Module(10, 2, 9, 3);
void setup() {
  Serial.begin(115200);

  // initialize SX1278 with default settings
  Serial.print(F("[SX1278] Initializing ... "));
  int state = radio.begin();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }
}

void loop() {
  Serial.print(F("[SX1278] Waiting for incoming transmission ... "));
  // you can receive data as an Arduino String
  // NOTE: receive() is a blocking method!
  //       See example ReceiveInterrupt for details
  //       on non-blocking reception method.
  String str;
  int state = radio.receive(str);

  // you can also receive data as byte array
  /*
    byte byteArr[8];
    int state = radio.receive(byteArr, 8);
  */

  if (state == RADIOLIB_ERR_NONE) {
    // packet was successfully received
    Serial.println(F("success!"));

    // print the data of the packet
    Serial.print(F("[SX1278] Data:\t\t\t"));
    Serial.println(str);

    // print the RSSI (Received Signal Strength Indicator)
    // of the last received packet
    Serial.print(F("[SX1278] RSSI:\t\t\t"));
    Serial.print(radio.getRSSI());
    Serial.println(F(" dBm"));

    // print the SNR (Signal-to-Noise Ratio)
    // of the last received packet
    Serial.print(F("[SX1278] SNR:\t\t\t"));
    Serial.print(radio.getSNR());
    Serial.println(F(" dB"));

    // print frequency error
    // of the last received packet
    Serial.print(F("[SX1278] Frequency error:\t"));
    Serial.print(radio.getFrequencyError());
    Serial.println(F(" Hz"));

  } else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
    // timeout occurred while waiting for a packet
    Serial.println(F("timeout!"));

  } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
    // packet was received, but is malformed
    Serial.println(F("CRC error!"));

  } else {
    // some other error occurred
    Serial.print(F("failed, code "));
    Serial.println(state);

  }
}

#else
#include <Arduino.h>
#include <SPI.h>

#define RADIOLIB_SX127X_REG_FIFO                                0x00
#define RADIOLIB_SX1278_CHIP_VERSION                            0x12
#define RADIOLIB_SX127X_REG_VERSION                             0x42
#define RADIOLIB_SX127X_SLEEP                                   0b00000000  //  2     0   sleep
#define RADIOLIB_SX127X_STANDBY                                 0b00000001  //  2     0   standby
#define RADIOLIB_SX127X_REG_OP_MODE                             0x01
#define RADIOLIB_SX127X_REG_HOP_PERIOD                          0x24
#define RADIOLIB_SX127X_HOP_PERIOD_OFF                          0b00000000  //  7     0   number of periods between frequency hops; 0 = disabled
#define RADIOLIB_SX127X_LORA                                    0b10000000  //  7     7   LoRa mode
#define RADIOLIB_SX127X_REG_SYNC_WORD                           0x39
#define RADIOLIB_SX127X_REG_OCP                                 0x0B
#define RADIOLIB_SX127X_OCP_OFF                                 0b00000000  //  5     5   PA overload current protection disabled
#define RADIOLIB_SX127X_OCP_ON                                  0b00100000  //  5     5   PA overload current protection enabled
#define RADIOLIB_SX127X_OCP_TRIM                                0b00001011  //  4     0   OCP current: I_max(OCP_TRIM = 0b1011) = 100 mA
#define RADIOLIB_SX127X_REG_PREAMBLE_MSB                        0x20
#define RADIOLIB_SX127X_REG_PREAMBLE_LSB                        0x21
#define RADIOLIB_SX127X_REG_MODEM_CONFIG_1                      0x1D
#define RADIOLIB_SX127X_REG_MODEM_CONFIG_2                      0x1E
#define RADIOLIB_SX1278_REG_MODEM_CONFIG_3                      0x26
#define RADIOLIB_SX1278_BW_125_00_KHZ                           0b01110000  //  7     4               125.00 kHz
#define RADIOLIB_SX127X_REG_FRF_MSB                             0x06
#define RADIOLIB_SX127X_REG_FRF_MID                             0x07
#define RADIOLIB_SX127X_REG_FRF_LSB                             0x08
#define RADIOLIB_SX127X_SF_9                                    0b10010000
#define RADIOLIB_SX1278_HEADER_EXPL_MODE                        0b00000000  //  0     0   explicit header mode
#define RADIOLIB_SX127X_TX_MODE_SINGLE                          0b00000000  //  3     3   single TX
#define RADIOLIB_SX127X_REG_DETECT_OPTIMIZE                     0x31
#define RADIOLIB_SX127X_REG_DETECTION_THRESHOLD                 0x37
#define RADIOLIB_SX127X_DETECT_OPTIMIZE_SF_7_12                 0b00000011  //  2     0   SF7 to SF12 detection optimization
#define RADIOLIB_SX127X_DETECTION_THRESHOLD_SF_7_12             0b00001010  //  7     0   SF7 to SF12 detection threshold
#define RADIOLIB_SX1278_CR_4_5                                  0b00000010  //  3     1   error coding rate:  4/5
#define RADIOLIB_SX1278_CR_4_6                                  0b00000100  //  3     1                       4/6
#define RADIOLIB_SX1278_CR_4_7                                  0b00000110  //  3     1                       4/7
#define RADIOLIB_SX1278_CR_4_8                                  0b00001000  //  3     1                       4/8
#define RADIOLIB_SX127X_REG_PA_CONFIG                           0x09
#define RADIOLIB_SX1278_REG_PA_DAC                              0x4D
#define RADIOLIB_SX127X_PA_SELECT_BOOST                         0b10000000  //  7     7   PA_BOOST pin output, power limited to +20 dBm
#define RADIOLIB_SX1278_MAX_POWER                               0b01110000  //  6     4   max power: P_max = 10.8 + 0.6*MAX_POWER [dBm]; P_max(MAX_POWER = 0b111) = 15 dBm
#define RADIOLIB_SX127X_PA_BOOST_OFF                            0b00000100  //  2     0   PA_BOOST disabled
#define RADIOLIB_SX1278_AGC_AUTO_ON                             0b00000100  //  2     2   LNA gain set by internal AGC loop
#define RADIOLIB_SX1278_RX_CRC_MODE_ON                          0b00000100  //  2     2   CRC enabled
#define RADIOLIB_SX1278_LOW_DATA_RATE_OPT_OFF                   0b00000000  //  3     3   low data rate optimization disabled
#define RADIOLIB_SX1278_LOW_DATA_RATE_OPT_ON                    0b00001000  //  3     3   low data rate optimization enabled
#define RADIOLIB_SX127X_REG_DIO_MAPPING_1                       0x40
#define RADIOLIB_SX127X_DIO0_LORA_TX_DONE                       0b01000000  //  7     6
#define RADIOLIB_SX127X_REG_IRQ_FLAGS                           0x12
#define RADIOLIB_SX127X_REG_PAYLOAD_LENGTH                      0x22
#define RADIOLIB_SX127X_REG_FIFO_TX_BASE_ADDR                   0x0E
#define RADIOLIB_SX127X_FIFO_TX_BASE_ADDR_MAX                   0b00000000  //  7     0   allocate the entire FIFO buffer for TX only
#define RADIOLIB_SX127X_REG_FIFO_ADDR_PTR                       0x0D
#define RADIOLIB_SX127X_TX                                      0b00000011  //  2     0   transmit


float freq = 434.0F;
float bw = 125.0F;
uint8_t sf = 9U;
uint8_t cr = 7U;
uint8_t syncWord = 18U;
int8_t power = 10;
uint16_t preambleLength = 8U;
uint8_t gain = 0U;
uint8_t cs  = 10;
uint8_t irq  = 2;
uint8_t gpio = 3;
uint8_t rst = 9;

SPISettings SPI_settings(2000000, MSBFIRST, SPI_MODE0);


byte SPIReadRegister(uint8_t cs, byte reg) {
    SPI.beginTransaction(SPI_settings);
    digitalWrite(cs, LOW); // Pull the NSS (Slave Select) pin low to start the transaction
    
    delay(1);
    
    SPI.transfer(reg & 0x7F); // Send the register address with the MSB set to 0 for read command
    byte regValue = SPI.transfer(0x00); // Send a dummy byte (0x00) to receive the response

    digitalWrite(cs, HIGH); // Pull the NSS pin high to end the transaction
    SPI.endTransaction();

    return regValue;
}

void SPIWriteRegister(uint8_t cs, uint8_t reg, uint8_t value) {
    SPI.beginTransaction(SPI_settings);
    // take the SS pin low to select the chip:
    digitalWrite(cs, LOW);

    delay(1);

    // send the register address, with write command (most significant bit should be 1 for write)
    SPI.transfer(reg | 0x80); 

    // send value
    SPI.transfer(value);

    // take the SS pin high to de-select the chip:
    digitalWrite(cs, HIGH);
    SPI.endTransaction();
}

//write to register and wait for it to actually update
void SPIWriteRegisterWait(uint8_t cs, uint8_t reg, uint8_t value, uint8_t check_mask, uint16_t check_interval = 5) {
    SPI.beginTransaction(SPI_settings);
    SPIWriteRegister(cs, reg, value);

    uint32_t start = micros();
    uint8_t readValue = 0x00;

    while(micros() - start < (check_interval * 1000)) {
        readValue = SPIReadRegister(cs, reg);
        if((readValue & check_mask) == (value & check_mask)) {
            // check passed, we can stop the loop
            return;
        }
    }

    Serial.println("Reg check failed!");
    SPI.endTransaction();
}

void SPIWriteRegisterMulti(uint8_t cs, uint8_t reg, uint8_t* data, size_t length) {
    SPI.beginTransaction(SPI_settings);
    // take the SS pin low to select the chip:
    digitalWrite(cs, LOW);

    delay(1);

    //send starting register address
    SPI.transfer(reg | 0x80);

    // send values
    for(size_t i = 0; i < length; i++) {
        SPI.transfer(data[i]);
    }


    digitalWrite(cs, HIGH);
    SPI.endTransaction();
}

uint8_t SPISetRegister(uint8_t reg, uint8_t value, uint8_t range_start = 0, uint8_t range_end = 7) {
    uint8_t reg_val = SPIReadRegister(cs, reg);
    
    Serial.print("SetRegister 0x");
    Serial.print(reg, HEX);
    Serial.println(":");
    Serial.print("Old:  0x");
    Serial.println(reg_val, HEX);
    
    uint8_t mask = ~(0xFF >> (8 - range_start) | 0xFF << (range_end + 1));
    reg_val = (reg_val & ~mask) | (value & mask);
    
    Serial.print("Want: 0x");
    Serial.println(reg_val, HEX);
    
    SPIWriteRegister(cs, reg, reg_val);
    delay(5);
    reg_val =  SPIReadRegister(cs, reg);
    
    Serial.print("Got:  0x");
    Serial.println(reg_val, HEX);

    return reg_val;
}


void reset() {
    pinMode(rst, OUTPUT);
    digitalWrite(rst, LOW);
    delay(1);
    digitalWrite(rst, HIGH);
    delay(5);
}

int16_t getChipVersion() {
    return SPIReadRegister(10, RADIOLIB_SX127X_REG_VERSION);
}

uint8_t setMode(uint8_t mode) {
    uint8_t reg = SPIReadRegister(cs, RADIOLIB_SX127X_REG_OP_MODE);

    reg = (reg & 0b11111000) | mode;
    SPIWriteRegister(cs, RADIOLIB_SX127X_REG_OP_MODE, reg);
    delay(5);
    reg = SPIReadRegister(cs, RADIOLIB_SX127X_REG_OP_MODE);

    return reg;
}

//try to find the SX127x chip
bool findChip(uint8_t ver) {
    uint8_t i = 0;
    bool flagFound = false;
    
    while((i < 10) && !flagFound) {
        //reset the module
        reset();
        delay(10);

        // check version register
        int16_t version = getChipVersion();
        if(version == ver) {
            flagFound = true;
        } else {
            Serial.println("SX127x not found!");
            Serial.print("  Got: ");
            Serial.println(version);
            Serial.print("  Expected: ");
            Serial.println(ver);
            delay(10);
            i++;
        }
    }

    return flagFound;
}


void setModemMode(uint8_t modem) {
    setMode(RADIOLIB_SX127X_SLEEP);

    uint8_t reg = SPIReadRegister(cs, RADIOLIB_SX127X_REG_OP_MODE);
    reg = (reg & 0b01111111) | modem;
    SPIWriteRegister(cs, RADIOLIB_SX127X_REG_OP_MODE, reg);

    delay(5);

    setMode(RADIOLIB_SX127X_STANDBY);
}

void setCurrentLimit(uint8_t current_limit) {
    setMode(RADIOLIB_SX127X_STANDBY);

    uint8_t raw;
    uint8_t reg = SPIReadRegister(cs, RADIOLIB_SX127X_REG_OCP);
    if(current_limit == 0) {
        // limit set to 0, disable OCP
        reg = (reg & 0b11011111) | RADIOLIB_SX127X_OCP_OFF;
        SPIWriteRegister(cs, RADIOLIB_SX127X_REG_OCP, reg);
    } else if(current_limit <= 120) {
        raw = (current_limit - 45) / 5;
        reg = (reg & 0b11000000) | RADIOLIB_SX127X_OCP_ON | raw;
        SPIWriteRegister(cs, RADIOLIB_SX127X_REG_OCP, reg);
    } else if(current_limit <= 240) {
        raw = (current_limit + 30) / 10;
        reg = (reg & 0b11000000) | RADIOLIB_SX127X_OCP_ON | raw;
        SPIWriteRegister(cs, RADIOLIB_SX127X_REG_OCP, reg);
    }
}


void startTransmission(uint8_t* data, size_t length, uint8_t addr) {
    setMode(RADIOLIB_SX127X_STANDBY);
    
    // set DIO mapping
    SPISetRegister(RADIOLIB_SX127X_REG_DIO_MAPPING_1, RADIOLIB_SX127X_DIO0_LORA_TX_DONE, 6, 7);


    // clear interrupt flags
    SPIWriteRegister(cs, RADIOLIB_SX127X_REG_IRQ_FLAGS, 0b11111111);

    // set packet length
    SPISetRegister(RADIOLIB_SX127X_REG_PAYLOAD_LENGTH, length);

    // set FIFO pointers
    SPISetRegister(RADIOLIB_SX127X_REG_FIFO_TX_BASE_ADDR, RADIOLIB_SX127X_FIFO_TX_BASE_ADDR_MAX);
    SPISetRegister(RADIOLIB_SX127X_REG_FIFO_ADDR_PTR, RADIOLIB_SX127X_FIFO_TX_BASE_ADDR_MAX);

  // write packet to FIFO
    SPIWriteRegisterMulti(cs, RADIOLIB_SX127X_REG_FIFO, data, length);

    // start transmission
    setMode(RADIOLIB_SX127X_TX);
}

void finishTransmit() {
    // clear interrupt flags
    SPIWriteRegister(cs, RADIOLIB_SX127X_REG_IRQ_FLAGS, 0b11111111);

    setMode(RADIOLIB_SX127X_STANDBY);
}

void transmit(uint8_t* data, size_t length, uint8_t addr) {
    setMode(RADIOLIB_SX127X_STANDBY);
    
    // calculate timeout (150 % of expected time-on-air)
    double symbolLength = (double)(uint32_t(1) << sf) / (double)bw;
    double de = 0;
    double ih = 0;
    double crc = 3;
    double n_pre = (double)preambleLength;
    size_t pl = length;
    uint8_t crc_en = 1;

    double top = 8 * pl - 4 * sf + 28 + 16 * crc_en - 20 * ih;
    double bottom = 4 * (sf - 2 * de);
    
    double payload_sim = 8.0 + max(ceil(top / bottom) * (crc + 4), 0.0);
    uint32_t time_on_air = ceil(symbolLength * (n_pre + payload_sim + 4.25)) * 1000;
    uint32_t timeout = time_on_air * 1.5;

    startTransmission(data, length, addr);

    while(!digitalRead(irq));

    finishTransmit();
}


void setup() {
    SPI.begin();
    Serial.begin(115200);

    // initialize SX1278 with default settings
    Serial.print(F("[SX1278] Initializing ... "));

    // set module properties
    pinMode(cs, OUTPUT);
    digitalWrite(cs, HIGH);
    pinMode(irq, INPUT);
    pinMode(gpio, INPUT);


    if(!findChip(RADIOLIB_SX1278_CHIP_VERSION)) {
        Serial.println("No SX127x found!");
        while(true);
    }
    Serial.println("SX1278 chip found!");
    Serial.println(SPIReadRegister(cs, RADIOLIB_SX1278_CHIP_VERSION), HEX);


    // set mode to standby
    setMode(0x01);

    Serial.print("OpModeReg: 0x");
    Serial.println(SPIReadRegister(cs, RADIOLIB_SX127X_REG_OP_MODE), HEX);

    //turn off frequency hopping
    SPIWriteRegister(cs, RADIOLIB_SX127X_REG_HOP_PERIOD, RADIOLIB_SX127X_HOP_PERIOD_OFF);

    //set lora mode
    setModemMode(RADIOLIB_SX127X_LORA);
    Serial.print("OpModeReg: 0x");
    Serial.println(SPIReadRegister(cs, RADIOLIB_SX127X_REG_OP_MODE), HEX);

    // set LoRa sync word
    SPIWriteRegister(cs, RADIOLIB_SX127X_REG_SYNC_WORD, syncWord);

    // set over current protection
    Serial.print("RegOcp: 0x");
    Serial.println(SPIReadRegister(cs, RADIOLIB_SX127X_REG_OCP), HEX);
    setCurrentLimit(60);
    Serial.print("RegOcp: 0x");
    Serial.println(SPIReadRegister(cs, RADIOLIB_SX127X_REG_OCP), HEX);

    // set preamble length
    Serial.print("RegPreambleMsb: 0x");
    Serial.println(SPIReadRegister(cs, RADIOLIB_SX127X_REG_PREAMBLE_MSB), HEX);
    Serial.print("RegPreambleLsb: 0x");
    Serial.println(SPIReadRegister(cs, RADIOLIB_SX127X_REG_PREAMBLE_LSB), HEX);
    
    SPIWriteRegister(cs, RADIOLIB_SX127X_REG_PREAMBLE_MSB, (uint8_t)((preambleLength >> 8) & 0xFF));
    SPIWriteRegister(cs, RADIOLIB_SX127X_REG_PREAMBLE_LSB, (uint8_t)(preambleLength & 0xFF));

    Serial.print("RegPreambleMsb: 0x");
    Serial.println(SPIReadRegister(cs, RADIOLIB_SX127X_REG_PREAMBLE_MSB), HEX);
    Serial.print("RegPreambleLsb: 0x");
    Serial.println(SPIReadRegister(cs, RADIOLIB_SX127X_REG_PREAMBLE_LSB), HEX);

    //125khz bandwidth
    //is already default
    SPISetRegister(RADIOLIB_SX127X_REG_MODEM_CONFIG_1, 0b01110000, 4, 7);


    //434 frequency
    //default already is 434
    uint32_t FRF = (freq * (uint32_t(1) << 19)) / 32.0;
    SPISetRegister(RADIOLIB_SX127X_REG_FRF_MSB, (FRF & 0xFF0000) >> 16);
    SPISetRegister(RADIOLIB_SX127X_REG_FRF_MID, (FRF & 0x00FF00) >> 8);
    SPISetRegister(RADIOLIB_SX127X_REG_FRF_LSB, FRF & 0x0000FF);


    //set spreading factor;
    //is also default already
    SPISetRegister(RADIOLIB_SX127X_REG_MODEM_CONFIG_1, RADIOLIB_SX1278_HEADER_EXPL_MODE, 0, 0);
    SPISetRegister(RADIOLIB_SX127X_REG_MODEM_CONFIG_2, RADIOLIB_SX127X_SF_9 | RADIOLIB_SX127X_TX_MODE_SINGLE, 4, 7);
    SPISetRegister(RADIOLIB_SX127X_REG_DETECT_OPTIMIZE, RADIOLIB_SX127X_DETECT_OPTIMIZE_SF_7_12, 0, 2);
    SPISetRegister(RADIOLIB_SX127X_REG_DETECTION_THRESHOLD, RADIOLIB_SX127X_DETECTION_THRESHOLD_SF_7_12);

    //set error coding rate to 4/7
    SPISetRegister(RADIOLIB_SX127X_REG_MODEM_CONFIG_1, RADIOLIB_SX1278_CR_4_7, 1, 3);

    //state = setOutputPower(power);
    SPISetRegister(RADIOLIB_SX127X_REG_PA_CONFIG, RADIOLIB_SX127X_PA_SELECT_BOOST, 7, 7);
    SPISetRegister(RADIOLIB_SX127X_REG_PA_CONFIG, RADIOLIB_SX1278_MAX_POWER | (power - 2), 0, 6);
    SPISetRegister(RADIOLIB_SX1278_REG_PA_DAC, RADIOLIB_SX127X_PA_BOOST_OFF, 0, 2);

    //enable automatic gain control
    SPISetRegister(RADIOLIB_SX1278_REG_MODEM_CONFIG_3, RADIOLIB_SX1278_AGC_AUTO_ON, 2, 2);

    //enable crc
    SPISetRegister(RADIOLIB_SX127X_REG_MODEM_CONFIG_2, RADIOLIB_SX1278_RX_CRC_MODE_ON, 2, 2);

    //configure low data rate
    float symbolLength = (float)(uint32_t(1) << sf) / (float)bw;
    Serial.println(symbolLength);
    if(symbolLength >= 16.0) {
        SPISetRegister(RADIOLIB_SX1278_REG_MODEM_CONFIG_3, RADIOLIB_SX1278_LOW_DATA_RATE_OPT_ON, 3, 3);
    }
    else {
        SPISetRegister(RADIOLIB_SX1278_REG_MODEM_CONFIG_3, RADIOLIB_SX1278_LOW_DATA_RATE_OPT_OFF, 3, 3);
    }
}


void loop() {
    Serial.print(F("[SX1278] Transmitting packet ... "));

    // you can transmit C-string or Arduino string up to
    // 256 characters long
    // NOTE: transmit() is a blocking method!
    //       See example SX127x_Transmit_Interrupt for details
    //       on non-blocking transmission method.
    //byte data[] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF};
    char message[] = "test";
    transmit((uint8_t*)message, strlen(message), 0);

  // wait for a second before transmitting again
  delay(1000);
}
#endif