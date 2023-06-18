#pragma once
#include "SPIClass.hpp"
#include "digitalIO.hpp"

/*----(register fields)----*/
//SX1278 chip version
#define SX1278_CHIP_VERSION                            0x12

//config 1
#define LORA_BANDWIDTH_7_8kHz    0b00000000
#define LORA_BANDWIDTH_10_4kHz   0b00010000
#define LORA_BANDWIDTH_15_6kHz   0b00100000
#define LORA_BANDWIDTH_20_8kHz   0b00110000
#define LORA_BANDWIDTH_31_25kHz  0b01000000
#define LORA_BANDWIDTH_41_7kHz   0b01010000
#define LORA_BANDWIDTH_62_5kHz   0b01100000
#define LORA_BANDWIDTH_125kHz    0b01110000
#define LORA_BANDWIDTH_250kHz    0b10000000
#define LORA_BANDWIDTH_500kHz    0b10010000

#define LORA_CODING_RATE_4_5              0b00000010
#define LORA_CODING_RATE_4_6              0b00000100
#define LORA_CODING_RATE_4_7              0b00000110
#define LORA_CODING_RATE_4_8              0b00001000

#define LORA_IMPLICIT_HEADER     0b00000001
#define LORA_EXPLICIT_HEADER     0b00000000

//config 2
#define LORA_SPREADING_FACTOR_6         0b01100000
#define LORA_SPREADING_FACTOR_7         0b01110000
#define LORA_SPREADING_FACTOR_8         0b10000000
#define LORA_SPREADING_FACTOR_9         0b10010000
#define LORA_SPREADING_FACTOR_10        0b10100000
#define LORA_SPREADING_FACTOR_11        0b10110000
#define LORA_SPREADING_FACTOR_12        0b11000000
#define LORA_TX_CONTINUOUS_MODE_ON      0b00001000
#define LORA_TX_CONTINUOUS_MODE_OFFT    0b00000000
//#define LORA_SYMB_TIMEOUT               0b00000000
#define LORA_RX_PAYLOAD_CRC_ON          0b00000100
#define LORA_RX_PAYLOAD_CRC_OFF         0b00000000

//config 3
#define LORA_LOW_DATA_RATE_OPT_OFF   0b00000000
#define LORA_LOW_DATA_RATE_OPT_ON    0b00001000
#define LORA_AGC_AUTO_OFF            0b00000000
#define LORA_AGC_AUTO_ON             0b00000100

//modes
#define SX1278_SLEEP                0b00000000
#define SX1278_STANDBY              0b00000001
#define SX1278_TX                   0b00000011
#define SX1278_LORA                 0b10000000

//power config
#define SX1278_PA_SELECT_BOOST   0b10000000
#define SX1278_PA_SELECT_RFO     0b00000000
//overcurrent protection
#define OCP_OFF                                 0b00000000
#define OCP_ON                                  0b00100000
#define OCP_TRIM                                0b00001011

//hopping config
#define HOP_PERIOD_OFF                          0b00000000

//Detection optimize
#define DETECT_OPTIMIZE_SF_7_12                 0b00000011
//Detection treshold
#define DETECTION_THRESHOLD_SF_7_12             0b00001010

//pinmaps
#define DIO0_LORA_TX_DONE                       0b01000000

#define FIFO_TX_BASE_ADDR_MAX                   0b00000000


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


//hopping config
#define HOP_PERIOD_OFF                          0b00000000

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
    uint8_t dio0;
    uint8_t cs;
    uint8_t rst;

    void startTransmission(uint8_t *data, uint8_t length);
    uint8_t finishTransmission();

public:
    SX1278(uint8_t cs, uint8_t rst, uint8_t dio0);
    ~SX1278();

    void reset();

    uint8_t getVersion();

    void setMode(uint8_t mode);

    void setModemMode(uint8_t modem);

    void setCurrentLimit(uint8_t current);



    uint8_t init(SPIClass *spi, float frequency, uint8_t sync_word, uint16_t preamble_len);
    uint8_t readRegister(uint8_t addr);
    void writeRegister(uint8_t addr, uint8_t data);
    void setRegister(uint8_t addr, uint8_t data, uint8_t mask_lsb = 0, uint8_t mask_msb = 7);

    uint8_t transmit(uint8_t *data, uint8_t length, uint8_t timeout);
};

