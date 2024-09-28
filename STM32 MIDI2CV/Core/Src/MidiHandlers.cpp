/*
 * MidiHandlers.cpp
 *
 *  Created on: Mar 20, 2020
 *      Author: Spiros Makris
 *
 */

#include "MidiHandlers.h"
#include "stm32f3xx_hal.h"
int i;
int u;
void Handle_Clock() {
	i++;
	if (i / 12 == 1) {
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		i = 0;
	}
}

void Handle_Start() {
	i = 0;
//	HAL_GPIO_WritePin(GPIOB, gate1_Pin, GPIO_PinState::GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOB, gate2_Pin, GPIO_PinState::GPIO_PIN_SET);
}
void Handle_Stop() {
//	HAL_GPIO_WritePin(GPIOB, gate1_Pin, GPIO_PinState::GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(GPIOB, gate2_Pin, GPIO_PinState::GPIO_PIN_RESET);
}

// Callback function for when a Note On is received


// Callback function for when a Note Off is received
