#ifndef DAC_H_
#define DAC_H_

#include <stm32f4xx_hal.h>

//const uint16_t kPinSS = GPIO_PIN_8;
void DACup();
void DAC_Init();
void DAC_Write(uint16_t channel_1, uint16_t channel_2);
bool DAC_Ready();
void DAC_Update();
void DAC_Update_Channel(uint8_t channel);

#endif // DAC_H_
