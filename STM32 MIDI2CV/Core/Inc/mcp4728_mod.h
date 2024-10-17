/*
 * mcp4728.h
 *
 *  Created on: Jul 9, 2024
 *      Author: Arda
 */

#ifndef INC_MCP4728_MOD_H_
#define INC_MCP4728_MOD_H_

#include "stm32f3xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* MCP4728 Channel Configuration Structure */
typedef struct {
    uint8_t vref;      /* 4-bit reference voltage info: 1=2.048V, 0=VDD */
    uint8_t gain;      /* 4-bit gain info: 1=x2, 0=x1 */
    uint16_t val[4];   /* 12-bit numbers specifying outputs for channels A, B, C, D */
//    uint8_t address;
} ChannelConfig;

typedef struct {
    uint8_t vref;      /* 4-bit reference voltage info: 1=2.048V, 0=VDD */
    uint8_t gain;      /* 4-bit gain info: 1=x2, 0=x1 */
    uint16_t val[4];   /* 12-bit numbers specifying outputs for channels A, B, C, D */
//    uint8_t address;
} ChannelConfig_2;

void dac_init(ChannelConfig *config, ChannelConfig_2 *config2);

/* Function Prototypes */
HAL_StatusTypeDef mcp4728_vrefSelect(I2C_HandleTypeDef *i2cHandler, ChannelConfig config);
HAL_StatusTypeDef mcp4728_gainSelect(I2C_HandleTypeDef *i2cHandler, ChannelConfig config);
HAL_StatusTypeDef mcp4728_writePwrDownSelect(I2C_HandleTypeDef *i2cHandler, uint8_t power_modes);
HAL_StatusTypeDef mcp4728_generalCall(I2C_HandleTypeDef *i2cHandler, uint8_t command);
HAL_StatusTypeDef DACx60FW(I2C_HandleTypeDef *i2cHandler, ChannelConfig config);
HAL_StatusTypeDef DACx61FW(I2C_HandleTypeDef *i2cHandler, ChannelConfig_2 config_0x61);
HAL_StatusTypeDef mcp4728_multiWrite(I2C_HandleTypeDef *i2cHandler, ChannelConfig config, uint8_t channel);
HAL_StatusTypeDef mcp4728_sequentialWrite(I2C_HandleTypeDef *i2cHandler, ChannelConfig config, uint8_t channel);
HAL_StatusTypeDef DACx60SW(I2C_HandleTypeDef *i2cHandler, ChannelConfig config, uint8_t channel);
HAL_StatusTypeDef DACx61SW(I2C_HandleTypeDef *i2cHandler, ChannelConfig_2 config_0x61, uint8_t channel);
HAL_StatusTypeDef mcp4728_newI2CAddress(I2C_HandleTypeDef *i2cHandler, uint8_t address);
HAL_StatusTypeDef mcp4728_configure(I2C_HandleTypeDef *i2cHandler, ChannelConfig config);

/* Commands and Modes */
#define MCP4728_GENERAL_RESET           0x06
#define MCP4728_GENERAL_WKUP            0x09
#define MCP4728_GENERAL_SOFTWARE_UPDATE 0x08
#define MCP4728_GENERAL_READ_ADDR       0x0C

#define MCP4728_FAST_WRITE              0x00
#define MCP4728_MULTI_WRITE             0x40
#define MCP4728_SEQ_WRITE               0x50
#define MCP4728_SINGLE_WRITE            0x58
#define MCP4728_ADDR_WRITE              0x60
#define MCP4728_VREF_WRITE              0x80
#define MCP4728_GAIN_WRITE              0xC0
#define MCP4728_PWRDOWN_WRITE           0xA0

#define dac1               (0x60 << 1) // 7-bit address shifted left
#define dac2               (0x61 << 1) // 7-bit address shifted left

#define MCP4728_GAIN_1                  0x0
#define MCP4728_GAIN_2                  0x1

#define ch_A               0x0
#define ch_B               0x1
#define ch_C               0x2
#define ch_D               0x3

#define MCP4728_PWRDWN_NORMAL           0x0
#define MCP4728_PWRDWN_1                0x1
#define MCP4728_PWRDWN_2                0x2
#define MCP4728_PWRDWN_3                0x3

#define MCP4728_UDAC_UPLOAD             0x1
#define MCP4728_UDAC_NOLOAD             0x0

#ifdef __cplusplus
}
#endif

#endif /* INC_MCP4728_MOD_H_ */
