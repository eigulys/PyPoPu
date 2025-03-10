/*
 * mygtukai.c
 *
 *  Created on: Nov 10, 2024
 *      Author: Master
 */
// adsr_encoders.c
#include "mygtukai.h"
#include "main.h" // Include your main header where ROT1 and ROT2 are defined
#include "adsr.h"

extern float attack_rate_lookup[128]; // Assuming this is defined elsewhere

extern ADSR_t envelopes[NUM_ENVELOPES];





void meniu_mygtukai(void) {
    switch (busena) {
            case M_ADSR:
                // Case for M1 button being active
                ADSR_SetAttackRate(&envelopes[0], ROT1);  // Map ROT1 to attack rate
                ADSR_SetDecayRate(&envelopes[0], ROT2);   // Map ROT2 to decay rate
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
                break;

            case M_DAC:
                // Case for M2 button being active
                ADSR_SetSustainLevel(&envelopes[0], ROT1);  // Map ROT1 to attack rate
                ADSR_SetReleaseRate(&envelopes[0], ROT2);   // Map ROT2 to decay rate
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
                break;

            default:
                // No button is pressed; do nothing
                break;
        }
    }

void UI_Update(void) {
	meniu_mygtukai();
}


