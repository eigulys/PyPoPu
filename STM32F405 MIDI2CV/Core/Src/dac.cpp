#include "dac.h"

extern SPI_HandleTypeDef hspi2;
static uint16_t data_[2];
static uint8_t active_channel_ = 0;
int16_t count_1 = 0;
int16_t count_2 = 0;

void DAC_Init() {
    // Initialization code for DAC
    // Ensure GPIO and SPI are initialized properly
//    __HAL_RCC_GPIOA_CLK_ENABLE();
//    __HAL_RCC_SPI2_CLK_ENABLE();
}

void DAC_Write(uint16_t channel_1, uint16_t channel_2) {
    data_[0] = channel_1;
    data_[1] = channel_2;
}

bool DAC_Ready() {
    return active_channel_ == 0;
}

void DAC_Update() {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
    uint8_t txData[2];
    HAL_StatusTypeDef status;
    if (active_channel_ == 0) {
        txData[0] = 0x10 | (data_[0] >> 8);
        txData[1] = data_[0] & 0xFF;
        status = HAL_SPI_Transmit(&hspi2, txData, 2, HAL_MAX_DELAY);
        if (status != HAL_OK) {
        	count_1++;
        }
        active_channel_ = 1;
    } else {
        txData[0] = 0x24 | (data_[1] >> 8);
        txData[1] = data_[1] & 0xFF;
        status = HAL_SPI_Transmit(&hspi2, txData, 2, HAL_MAX_DELAY);
           if (status != HAL_OK) {
           	count_2++;
           }
        active_channel_ = 0;
    }

}

//void DACup() {
////    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
//
//    GPIOA->BSRR = GPIO_PIN_8;
//    GPIOA->BRR = GPIO_PIN_8;
//    if (active_channel_ == 0) {
//      SPI2->DR = 0x1000 | (data_[0] >> 8);
//      while (!(SPI2->SR & SPI_FLAG_TXE)) {
//          // Debugging: Print the status register
//    	  count_1++;
//      }
////      while( (SPI1->SR & SPI_SR_BSY) );  			//jei visdar uzimta
////      volatile uint16_t dummy = SPI2->DR;			//pratrina
//      SPI2->DR = (data_[0] << 8) & 0xffff;
//      active_channel_ = 1;
//    } else {
//      SPI2->DR = 0x2000 | (data_[1] >> 8);
//      while (!(SPI2->SR & SPI_FLAG_TXE )) {
//    	  count_2++;
//      }
//      SPI2->DR = (data_[1] << 8) & 0xffff;
//      active_channel_ = 0;
//    }
////        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
//}

void DAC_Update_Channel(uint8_t channel) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
    if (channel == 0) {
        SPI2->DR = 0x1000 | (data_[0] >> 8);
        while (!(SPI2->SR & SPI_FLAG_RXNE));
        SPI2->DR = (data_[0] << 8) & 0xffff;
    } else {
        SPI2->DR = 0x2400 | (data_[1] >> 8);
        while (!(SPI2->SR & SPI_FLAG_RXNE));
        SPI2->DR = (data_[1] << 8) & 0xffff;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
    }
}
