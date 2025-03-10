/*
 * mygtukai.h
 *
 *  Created on: Nov 10, 2024
 *      Author: Master
 */

#ifndef INC_MYGTUKAI_H_
#define INC_MYGTUKAI_H_


#include "adsr.h" // Include the header where ADSR_t is defined

extern int16_t ROT1;
extern int16_t ROT2;
extern int busena;
//volatile uint8_t busena = 0; // 0: None, 1: M1, 2: M2

//extern int busena; // Declare busena as extern

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    M_ADSR,
    M_DAC
} ControlState;




void meniu_mygtukai(void);
void UI_Update(void);


#ifdef __cplusplus
}
#endif
#endif /* INC_MYGTUKAI_H_ */
