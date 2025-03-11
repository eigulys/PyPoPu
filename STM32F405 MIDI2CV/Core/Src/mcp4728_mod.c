/*
 * mcp4728.c
 *
 *  Created on: Jul 10, 2024
 *      Author: Arda
 */

#include <mcp4728_mod.h>


void dac_init(ChannelConfig *config ,ChannelConfig_2 *config2) {
    config->vref = 0x1;  // Use VDD as reference voltage - 0, internal - 1
    config->gain = 0x1;  // Gain of 1x
    config->val[0] = 0;  // 12-bit DAC value for channel A
    config->val[1] = 0;  // 12-bit DAC value for channel B
    config->val[2] = 0;  // 12-bit DAC value for channel C
    config->val[3] = 0;  // 12-bit DAC value for channel D


    config2->vref = 0x0;  // Use VDD as reference voltage
    config2->gain = 0x1;  // Gain of 1x
    config2->val[0] = 0;  // 12-bit DAC value for channel A
    config2->val[1] = 0;  // 12-bit DAC value for channel B
    config2->val[2] = 0;  // 12-bit DAC value for channel C
    config2->val[3] = 0;  // 12-bit DAC value for channel D

}



/**
 * @brief Selects the voltage reference for the MCP4728.
 *
 * This function selects the voltage reference source (either internal or external)
 * for the specified channel of the MCP4728 device.
 *
 * @param i2cHandler Pointer to the I2C handler.
 * @param config Configuration structure containing the Vref selection.
 * @return HAL_StatusTypeDef HAL status indicating success or failure.
 */
HAL_StatusTypeDef mcp4728_vrefSelect(I2C_HandleTypeDef *i2cHandler, ChannelConfig config) {
    uint8_t data = MCP4728_VREF_WRITE | config.vref;
    // Transmit the single byte containing the Vref selection
    return HAL_I2C_Master_Transmit(i2cHandler, dac1, &data, 0, HAL_MAX_DELAY);
}

/**
 * @brief Selects the gain for the MCP4728.
 *
 * This function selects the gain (either 1 or 2) for the specified channel of the
 * MCP4728 device.
 *
 * @param i2cHandler Pointer to the I2C handler.
 * @param config Configuration structure containing the gain selection.
 * @return HAL_StatusTypeDef HAL status indicating success or failure.
 */
HAL_StatusTypeDef mcp4728_gainSelect(I2C_HandleTypeDef *i2cHandler, ChannelConfig config) {
    uint8_t data = MCP4728_GAIN_WRITE | config.gain;
    // Transmit the single byte containing the gain selection
    return HAL_I2C_Master_Transmit(i2cHandler, dac1, &data, 1, HAL_MAX_DELAY);
}

/**
 * @brief Selects the power down mode for the MCP4728.
 *
 * This function sets the power down mode for the specified channels of the MCP4728
 * device. The power down mode determines the output impedance of the channels.
 *
 * @param i2cHandler Pointer to the I2C handler.
 * @param power_modes Byte including power down modes for each channel.
 * @return HAL_StatusTypeDef HAL status indicating success or failure.
 */
HAL_StatusTypeDef mcp4728_writePwrDownSelect(I2C_HandleTypeDef *i2cHandler, uint8_t power_modes) {
    uint8_t buf[2];
    buf[0] = MCP4728_PWRDOWN_WRITE | ((power_modes & 0xF0) >> 4);
    buf[1] = (power_modes & 0x0F) << 4;
    // Transmit the two bytes containing the power down mode selection
    return HAL_I2C_Master_Transmit(i2cHandler, dac1, buf, sizeof(buf), HAL_MAX_DELAY);
}

/**
 * @brief Sends a general call command to the MCP4728.
 *
 * This function sends a general call command to the MCP4728 device. General call commands
 * can be used to perform actions such as reset or update all DAC channels.
 *
 * @param i2cHandler Pointer to the I2C handler.
 * @param command General call command to send.
 * @return HAL_StatusTypeDef HAL status indicating success or failure.
 */
//HAL_StatusTypeDef mcp4728_generalCall(I2C_HandleTypeDef *i2cHandler, uint8_t command) {
//    // Transmit the general call command
//    return HAL_I2C_Master_Transmit(i2cHandler, 0x00, &command, 1, HAL_MAX_DELAY);
//}

HAL_StatusTypeDef mcp4728_generalCall(I2C_HandleTypeDef *i2cHandler, uint8_t command) {
    // Transmit the general call command
    HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit_DMA(i2cHandler, 0x00, &command, 1);
    if (ret != HAL_OK) {
        return ret;
    }

    // Wait for the DMA transfer to complete
    while (HAL_I2C_GetState(i2cHandler) != HAL_I2C_STATE_READY) {}

    return HAL_OK;
}
/**
 * @brief Performs a fast write operation on the MCP4728.
 *
 * This function performs a fast write operation on the MCP4728 device, updating the DAC
 * output values for all four channels in a single I2C transaction. It then sends a general
 * call command to update the outputs. Gain, voltage reference, power mode options are not
 * configurable. They are not changed in the transaction. Prior values are used from EEPROM.
 *
 * @param i2cHandler Pointer to the I2C handler.
 * @param config Configuration structure containing the DAC values for each channel.
 * @return HAL_StatusTypeDef HAL status indicating success or failure.
 */
HAL_StatusTypeDef DACx60FW(I2C_HandleTypeDef *i2cHandler, ChannelConfig config) {
    uint8_t buf[8]; // Buffer to hold the data for 4 channels, 2 bytes each

    for (uint8_t i = 0; i < 4; i++) {
        buf[2 * i] = (config.val[i] >> 8); // Upper 8 bits of DAC value
        buf[2 * i + 1] = config.val[i] & 0xFF; // Lower 8 bits of DAC value
    }

    HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit_DMA(i2cHandler, dac1, buf, sizeof(buf));
    if (ret != HAL_OK) {
        return ret;
    }

    while (HAL_I2C_GetState(i2cHandler) != HAL_I2C_STATE_READY) {}

    return mcp4728_generalCall(i2cHandler, MCP4728_GENERAL_SOFTWARE_UPDATE);
}

HAL_StatusTypeDef DACx60FW_b(I2C_HandleTypeDef *i2cHandler, ChannelConfig config) {
    uint8_t buf[8]; // Buffer to hold the data for 4 channels, 2 bytes each

    for (uint8_t i = 0; i < 4; i++) {
        buf[2 * i] = (config.val[i] >> 8); // Upper 8 bits of DAC value
        buf[2 * i + 1] = config.val[i] & 0xFF; // Lower 8 bits of DAC value
    }

    HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(i2cHandler, dac1, buf, sizeof(buf), HAL_MAX_DELAY);
    if (ret != HAL_OK) {
        return ret;
    }

    return mcp4728_generalCall(i2cHandler, MCP4728_GENERAL_SOFTWARE_UPDATE);
}



HAL_StatusTypeDef DACx61FW(I2C_HandleTypeDef *i2cHandler, ChannelConfig_2 config_0x61) {
    uint8_t buf[8]; // Buffer to hold the data for 4 channels, 2 bytes each

    for (uint8_t i = 0; i < 4; i++) {
        buf[2 * i] = (config_0x61.val[i] >> 8); // Upper 8 bits of DAC value
        buf[2 * i + 1] = config_0x61.val[i] & 0xFF; // Lower 8 bits of DAC value
    }

    HAL_StatusTypeDef ret =  HAL_I2C_Master_Transmit_DMA(i2cHandler, dac2, buf, sizeof(buf));
    if (ret != HAL_OK) {
        return ret;
    }

    while (HAL_I2C_GetState(i2cHandler) != HAL_I2C_STATE_READY) {}

    return mcp4728_generalCall(i2cHandler, MCP4728_GENERAL_SOFTWARE_UPDATE);
}

/**
 * @brief Performs a single write operation on a specified channel of the MCP4728.
 *
 * This function performs a single write operation on a specified channel of the MCP4728 device,
 * updating the DAC output value for that channel. It then sends a general call command to update
 * the output.
 *
 * @param i2cHandler Pointer to the I2C handler.
 * @param config Configuration structure containing the DAC value and configuration for the channel.
 * @param channel The channel to be updated (0 for A, 1 for B, 2 for C, 3 for D).
 * @return HAL_StatusTypeDef HAL status indicating success or failure.
 */
HAL_StatusTypeDef DACx60SW(I2C_HandleTypeDef *i2cHandler, ChannelConfig config, uint8_t channel) {
    uint8_t buf[3];
    buf[0] = MCP4728_SINGLE_WRITE | (channel << 1); // Command and channel
    buf[1] = ((config.vref & (1 << channel)) ? 0x80 : 0) | // VREF bit (7th bit)
             ((config.gain & (1 << channel)) ? 0x10 : 0) | // Gain bit (4th bit)
             (config.val[channel] >> 8); // Upper 8 bits of the 12-bit DAC value
    buf[2] = config.val[channel] & 0xFF; // Lower 8 bits of the 12-bit DAC value

    HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit_DMA(i2cHandler, dac1, buf, sizeof(buf));
    if (ret != HAL_OK) {
        return ret;
    }

    // Wait for the DMA transfer to complete and then send the general call command
    while (HAL_I2C_GetState(i2cHandler) != HAL_I2C_STATE_READY) {}

    return mcp4728_generalCall(i2cHandler, MCP4728_GENERAL_SOFTWARE_UPDATE);
}



HAL_StatusTypeDef DACx61SW(I2C_HandleTypeDef *i2cHandler, ChannelConfig_2 config_0x61, uint8_t channel) {
    uint8_t buf[3];
    buf[0] = MCP4728_SINGLE_WRITE | (channel << 1); // Command and channel
    buf[1] = ((config_0x61.vref & (1 << channel)) ? 0x80 : 0) | // VREF bit (7th bit)
             ((config_0x61.gain & (1 << channel)) ? 0x10 : 0) | // Gain bit (4th bit)
             (config_0x61.val[channel] >> 8); // Upper 8 bits of the 12-bit DAC value
    buf[2] = config_0x61.val[channel] & 0xFF; // Lower 8 bits of the 12-bit DAC value

    HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit_DMA(i2cHandler, dac2, buf, sizeof(buf));
    if (ret != HAL_OK) {
        return ret;
    }

    // Wait for the DMA transfer to complete and then send the general call command
    while (HAL_I2C_GetState(i2cHandler) != HAL_I2C_STATE_READY) {}

    return mcp4728_generalCall(i2cHandler, MCP4728_GENERAL_SOFTWARE_UPDATE);
}

//HAL_StatusTypeDef DACx61SW(I2C_HandleTypeDef *i2cHandler, ChannelConfig_2 config_0x61, uint8_t channel) {
//    uint8_t buf[3];
//    buf[0] = MCP4728_SINGLE_WRITE | (channel << 1); // Command and channel
//    buf[1] = ((config_0x61.vref & (1 << channel)) ? 0x80 : 0) | // VREF bit (7th bit)
//             ((config_0x61.gain & (1 << channel)) ? 0x10 : 0) | // Gain bit (4th bit)
//             (config_0x61.val[channel] >> 8); // Upper 8 bits of the 12-bit DAC value
//    buf[2] = config_0x61.val[channel] & 0xFF; // Lower 8 bits of the 12-bit DAC value
//
//    HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(i2cHandler, dac2, buf, sizeof(buf), HAL_MAX_DELAY);
//    if (ret != HAL_OK) {
//        return ret;
//    }
//
//    return mcp4728_generalCall(i2cHandler, MCP4728_GENERAL_SOFTWARE_UPDATE);
//}

/**
 * @brief Performs a multi-write operation on the MCP4728.
 *
 * This function performs a multi-write operation on the MCP4728 device, updating the DAC
 * output values for all specified channels in a single I2C transaction. It then sends a general
 * call command to update the outputs.
 *
 * @param i2cHandler Pointer to the I2C handler.
 * @param config Configuration structure containing the DAC values and configuration for each channel.
 * @param channel The starting channel to be updated (0 for A, 1 for B, 2 for C, 3 for D).
 * @return HAL_StatusTypeDef HAL status indicating success or failure.
 */
HAL_StatusTypeDef mcp4728_multiWrite(I2C_HandleTypeDef *i2cHandler, ChannelConfig config, uint8_t channel) {
    uint8_t buf[12];
    uint8_t buf_index = 0;

    for (uint8_t i = channel; i < 4; i++) {
        buf[buf_index++] = MCP4728_MULTI_WRITE | (i << 1); // Command and channel
        buf[buf_index++] = ((config.vref & (1 << i)) ? 0x80 : 0) | // VREF bit (7th bit)
                           ((config.gain & (1 << i)) ? 0x10 : 0) | // Gain bit (4th bit)
                           (config.val[i] >> 8); // Upper 8 bits of the 12-bit DAC value
        buf[buf_index++] = config.val[i] & 0xFF; // Lower 8 bits of the 12-bit DAC value
    }

    HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit_DMA(i2cHandler, dac1, buf, buf_index);
    if (ret != HAL_OK) {
        return ret;
    }

    while (HAL_I2C_GetState(i2cHandler) != HAL_I2C_STATE_READY) {}

    return mcp4728_generalCall(i2cHandler, MCP4728_GENERAL_SOFTWARE_UPDATE);
}

/**
 * @brief Performs a sequential write operation on the MCP4728.
 *
 * This function performs a sequential write operation on the MCP4728 device, updating the DAC
 * output values for all specified channels in a single I2C transaction. It then sends a general
 * call command to update the outputs.
 *
 * @param i2cHandler Pointer to the I2C handler.
 * @param config Configuration structure containing the DAC values and configuration for each channel.
 * @param channel The starting channel to be updated (0 for A, 1 for B, 2 for C, 3 for D).
 * @return HAL_StatusTypeDef HAL status indicating success or failure.
 */
HAL_StatusTypeDef mcp4728_sequentialWrite(I2C_HandleTypeDef *i2cHandler, ChannelConfig config, uint8_t channel) {
    uint8_t buf[9 - 2 * channel];
    buf[0] = MCP4728_SEQ_WRITE | (channel << 1);
    uint8_t buf_index = 1;

    for (uint8_t i = 0; i < 4 - channel; i++) {
        buf[buf_index++] = ((config.vref & (0 << (i + channel))) ? 0x80 : 0) | // VREF bit (7th bit)
                           ((config.gain & (0 << (i + channel))) ? 0x10 : 0) | // Gain bit (4th bit)
                           (config.val[i + channel] >> 8); // Upper 8 bits of the 12-bit DAC value
        buf[buf_index++] = config.val[i + channel] & 0xFF; // Lower 8 bits of the 12-bit DAC value
    }

    HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit_DMA(i2cHandler, dac1, buf, buf_index);
    if (ret != HAL_OK) {
        return ret;
    }

    while (HAL_I2C_GetState(i2cHandler) != HAL_I2C_STATE_READY) {}

    return mcp4728_generalCall(i2cHandler, MCP4728_GENERAL_SOFTWARE_UPDATE);
}



HAL_StatusTypeDef mcp4728_configure(I2C_HandleTypeDef *i2cHandler, uint8_t dac_address, ChannelConfig config) {
    uint8_t buf[9];

    // Build multi-write command for all 4 channels
    buf[0] = MCP4728_MULTI_WRITE;
    for(uint8_t i = 0; i < 4; i++) {
        buf[1 + (i*3)] = (i << 1); // Channel select
        buf[2 + (i*3)] = ((config.vref & (1 << i)) ? 0x80 : 0) |
                         ((config.gain & (1 << i)) ? 0x10 : 0) |
                         (config.val[i] >> 8);
        buf[3 + (i*3)] = config.val[i] & 0xFF;
    }

    HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(i2cHandler, dac_address, buf, 13, HAL_MAX_DELAY);
    if(ret != HAL_OK) return ret;

    return mcp4728_generalCall(i2cHandler, MCP4728_GENERAL_SOFTWARE_UPDATE);
}
