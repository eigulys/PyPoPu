

#include "MidiHandlers.h"
#include "stm32f4xx_hal.h"
#include "MIDI.h"
#include "mcp4728_mod.h"
#include "adsr.h"
#include "main.h"
#include "lookup_t.h"

#define X 0.25 // santykis
extern I2C_HandleTypeDef hi2c1;
extern ADSR_t envelopes[NUM_ENVELOPES];

int i;
int u;
int tt;
int yy;
bool first_note_active = false;
bool second_note_active = false;
bool third_note_active = false;
bool fourth_note_active = false;
uint32_t pitch1_CV;
uint32_t pitch2_CV;
uint32_t pitch3_CV;
uint32_t pitch4_CV;
uint8_t first_note, range_idx;
uint8_t second_note;
uint8_t third_note;
uint8_t fourth_note;
uint16_t coarse_DAC, coarse_start, fine_index, fine_DAC;
void Handle_Clock() {
	i++;
	if (i / 12 == 1) {
		HAL_GPIO_TogglePin(LED_D1_GPIO_Port, LED_D1_Pin);
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
void Handle_NoteOn(uint8_t channel, uint8_t note, uint8_t velocity) {
//    uint32_t pitch_CV = (uint32_t)((note * 0.0833333333 * X) / (3.3 / 4095));  // Calculate pitch CV from MIDI note
    uint32_t velo_CV = (uint32_t)((velocity / 127.0) * 4095);

/*  Natos aukščio skaičiavimai  */
    range_idx = note / 24;         // Integer division
    range_idx = (range_idx > 4) ? 4 : range_idx; // Clamp to 4
    coarse_DAC = coarse_cv[range_idx];

    fine_DAC = fine_cv[note];

    ChannelConfig config;
    ChannelConfig_2 config2;

    if (!first_note_active) {
//        pitch1_CV = pitch_CV;
        tt=0;
        yy=0;
        ADSR_SetGateSignal(&envelopes[0], 1);
        config.val[0] = fine_DAC; // 12-bit DAC value for channel A
        config.val[1] = coarse_DAC;  // 12-bit DAC value for channel B
        config.val[2] = 4094; // 12-bit DAC value for channel C
        config.val[3] = 4094;  // 12-bit DAC value for channel D

          DACx60FW(&hi2c1, config);

          config2.val[0] = 4095; // 12-bit DAC value for channel A
          config2.val[1] = 4095;  // 12-bit DAC value for channel B
          config2.val[2] = 4095; // 12-bit DAC value for channel C
          config2.val[3] = 4095;  // 12-bit DAC value for channel D
          DACx61FW(&hi2c1, config2);


//        mcp4728_multiWrite(&hi2c1, config, 1);
//        DACx60SW(&hi2c1, config, 0);
//        DACx60SW(&hi2c1, config, 1);

//        HAL_GPIO_WritePin(GPIOB, gate1_Pin, GPIO_PIN_SET);  // Indicate first note is on via gate3_Pin
        first_note_active = true;  // First note is now active
        first_note = note;  // Store the note value
    }
//    	else if (!second_note_active) {
//        pitch2_CV = pitch_CV;
//        yy=0;
//        ADSR_SetGateSignal(&envelopes[1], 1);
//        config.val[2] = pitch2_CV; // 12-bit DAC value for channel C
//        config.val[3] = yy;  // 12-bit DAC value for channel D
////        DACx60FW(&hi2c1, config);
//          DACx60FW(&hi2c1, config);
////        DACx60SW(&hi2c1, config, 2);
////        DACx60SW(&hi2c1, config, 3);
//
////        HAL_GPIO_WritePin(GPIOB, gate2_Pin, GPIO_PIN_SET);  // Indicate second note is on via gate2_Pin
//        second_note_active = true;  // Second note is now active
//        second_note = note;  // Store the note value
//     }
//      else if (!third_note_active) {
//        pitch3_CV = pitch_CV;
//
//        ADSR_SetGateSignal(&envelopes[2], 1);
//        config.val[2] = pitch3_CV; // 12-bit DAC value for channel A
//        config2.val[2] = velo_CV;  // 12-bit DAC value for channel B
//        DACx60SW(&hi2c1, config, 2);
//        DACx61SW(&hi2c1, config2, 2);
//
////        HAL_GPIO_WritePin(GPIOB, gate3_Pin, GPIO_PIN_SET);  // Indicate third note is on via gate1_Pin
//        third_note_active = true;  // Third note is now active
//        third_note = note;  // Store the note value
//    }
//    	  else if (!fourth_note_active) {
//        pitch4_CV = pitch_CV;
//
//        ADSR_SetGateSignal(&envelopes[3], 1);
//        config.val[3] = pitch4_CV; // 12-bit DAC value for channel A
//        config2.val[3] = velo_CV;  // 12-bit DAC value for channel B
//        DACx60SW(&hi2c1, config, 3);
//        DACx61SW(&hi2c1, config2, 3);
//
////        HAL_GPIO_WritePin(GPIOE, gate4_Pin, GPIO_PIN_SET);  // Indicate fourth note is on via gate4_Pin
//        fourth_note_active = true;  // Fourth note is now active
//        fourth_note = note;  // Store the note value
//    }
}

void Handle_NoteOff(uint8_t channel, uint8_t note, uint8_t velocity) {
    if (first_note_active && note == first_note) {
        ADSR_SetGateSignal(&envelopes[0], 0);

//        HAL_GPIO_WritePin(GPIOB, gate1_Pin, GPIO_PIN_RESET);  // PB2 Turn off gate for first note
        first_note_active = false;  // First note is no longer active
    }
    	else if (second_note_active && note == second_note) {
        ADSR_SetGateSignal(&envelopes[1], 0);
//       HAL_GPIO_WritePin(GPIOB, gate2_Pin, GPIO_PIN_RESET);  // Turn off gate for second note
        second_note_active = false;  // Second note is no longer active
    }
//      else if (third_note_active && note == third_note) {
//        ADSR_SetGateSignal(&envelopes[2], 0);
////        HAL_GPIO_WritePin(GPIOB, gate3_Pin, GPIO_PIN_RESET);  // Turn off gate for third note
//        third_note_active = false;  // Third note is no longer active
//    }
//    else if (fourth_note_active && note == fourth_note) {
//        ADSR_SetGateSignal(&envelopes[3], 0);
////        HAL_GPIO_WritePin(GPIOE, gate4_Pin, GPIO_PIN_RESET);  // Turn off gate for fourth note
//        fourth_note_active = false;  // Fourth note is no longer active
//    }
}
