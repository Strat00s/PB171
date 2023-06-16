#pragma once
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
#define PA_SELECT_BOOST                         0b10000000
#define MAX_POWER                               0b01110000
#define PA_BOOST_OFF                            0b00000100
//overcurrent protection
#define OCP_OFF                                 0b00000000
#define OCP_ON                                  0b00100000
#define OCP_TRIM                                0b00001011

//hopping config
#define HOP_PERIOD_OFF                          0b00000000

//config 1
#define BW_7_80_KHZ                             0b00000000
#define BW_10_40_KHZ                            0b00010000
#define BW_15_60_KHZ                            0b00100000
#define BW_20_80_KHZ                            0b00110000
#define BW_31_25_KHZ                            0b01000000
#define BW_41_70_KHZ                            0b01010000
#define BW_62_50_KHZ                            0b01100000
#define BW_125_00_KHZ                           0b01110000
#define BW_250_00_KHZ                           0b10000000
#define BW_500_00_KHZ                           0b10010000
#define CR_4_5                                  0b00000010
#define CR_4_6                                  0b00000100
#define CR_4_7                                  0b00000110
#define CR_4_8                                  0b00001000
#define HEADER_EXPL_MODE                        0b00000000
#define HEADER_IMPL_MODE                        0b00000001

//config 2
#define SF_6                                    0b01100000
#define SF_7                                    0b01110000
#define SF_8                                    0b10000000
#define SF_9                                    0b10010000
#define SF_10                                   0b10100000
#define SF_11                                   0b10110000
#define SF_12                                   0b11000000
#define TX_MODE_SINGLE                          0b00000000
#define TX_MODE_CONT                            0b00001000
#define RX_TIMEOUT_MSB                          0b00000000
#define RX_CRC_MODE_OFF                         0b00000000
#define RX_CRC_MODE_ON                          0b00000100

//config 3
#define LOW_DATA_RATE_OPT_OFF                   0b00000000
#define LOW_DATA_RATE_OPT_ON                    0b00001000
#define AGC_AUTO_OFF                            0b00000000
#define AGC_AUTO_ON                             0b00000100

//Detection optimize
#define DETECT_OPTIMIZE_SF_7_12                 0b00000011
//Detection treshold
#define DETECTION_THRESHOLD_SF_7_12             0b00001010

//pinmaps
#define DIO0_LORA_TX_DONE                       0b01000000

#define FIFO_TX_BASE_ADDR_MAX                   0b00000000


#define SX1278_READ  0
#define SX1278_WRITE 1


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



    uint8_t init(SPIClass *spi, uint8_t sync_word, uint16_t preamble_len, int8_t power);
    uint8_t readRegister(uint8_t addr);
    void writeRegister(uint8_t addr, uint8_t data);
    void setRegister(uint8_t addr, uint8_t data, uint8_t mask_lsb = 0, uint8_t mask_msb = 7);

    uint8_t transmit(uint8_t *data, uint8_t length, uint8_t addr, uint8_t timeout);
};

