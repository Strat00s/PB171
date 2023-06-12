#pragma once
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


class SX1278 {
private:
    /* data */
    SPIClass *spi;
    uint8_t irq;
    uint8_t gpio;
    uint8_t cs;
    uint8_t rst;

    void startTransmission(uint8_t *data, uint8_t length, uint8_t addr);
    uint8_t finishTransmission();

public:
    SX1278(uint8_t cs, uint8_t rst, uint8_t irq, uint8_t gpio);
    ~SX1278();

    void reset();

    uint8_t getVersion();

    void setMode(uint8_t mode);

    void setModemMode(uint8_t modem);

    void setCurrentLimit(uint8_t current);



    void init(SPIClass *spi, uint8_t sync_word, uint16_t preamble_len, int8_t power);
    uint8_t readRegister(uint8_t addr);
    void writeRegister(uint8_t addr, uint8_t data);
    void setRegister(uint8_t addr, uint8_t data, uint8_t mask_lsb = 0, uint8_t mask_msb = 7);

    uint8_t transmit(uint8_t *data, uint8_t length, uint8_t addr, uint8_t timeout);
};

