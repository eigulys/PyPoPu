/*
 * MidiHandlers.h
 *
 *  Created on: Mar 20, 2020
 *      Author: Spiros
 */

#ifndef INC_MIDIHANDLERS_H_
#define INC_MIDIHANDLERS_H_




#endif /* INC_MIDIHANDLERS_H_ */
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "main.h"
#include "MIDI.h"
void Handle_Clock();
void Handle_Start();
void Handle_Stop();
void Handle_NoteOn(uint8_t status, uint8_t data1, uint8_t data2);
void Handle_NoteOff(uint8_t channel, uint8_t note, uint8_t velocity);
