// adsr.h

#ifndef ADSR_H
#define ADSR_H

#include "stm32f3xx_hal.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif
// ADSR State Machine Enum
typedef enum {
    ATTACK,
    DECAY,
    SUSTAIN,
    RELEASE,
    IDLE
} ADSR_State;



// ADSR structure for holding parameters and state for each envelope
typedef struct {
    float attack_rate;
    float decay_rate;
    float sustain_level;
    float release_rate;
    float amplitude;                 // Overall amplitude control (0.0 to 1.0)
    volatile float envelope_value;   // Current envelope value (0.0 to 1.0)
    volatile ADSR_State state;
    volatile uint8_t gate_signal;
} ADSR_t;

// Function declarations for controlling individual ADSR envelopes
void ADSR_Init(ADSR_t *adsr, int num_envelopes);  // Initialize multiple ADSR instances
//void ADSR_Init(ADSR_t *adsr);                           // Initialize an ADSR instance
void ADSR_UpdateEnvelope(ADSR_t *adsr);
void ADSR_UpdateEnvelope_old(ADSR_t *adsr);  // Update a specific ADSR instance
void ADSR_SetGateSignal(ADSR_t *adsr, uint8_t gate);    // Set gate signal for a specific ADSR
//void ADSR_SetGateSignal(ADSR_t *adsr, bool gate);
//void ADSR_HandleCC(ADSR_t *adsr, uint8_t cc, uint8_t value); // Handle MIDI CC messages
void ADSR_SetAmplitude(ADSR_t *adsr, float amplitude);  // Set amplitude for a specific ADSR
float ADSR_GetEnvelopeValue(const ADSR_t *adsr);        // Get the envelope value for a specific ADSR

// Setters for individual parameters
void ADSR_SetAttackRate(ADSR_t *adsr, float attack_rate);
void ADSR_SetDecayRate(ADSR_t *adsr, float decay_rate);
void ADSR_SetSustainLevel(ADSR_t *adsr, float sustain_level);
void ADSR_SetReleaseRate(ADSR_t *adsr, float release_rate);
void ADSR_SetAmplitude(ADSR_t *adsr, float amplitude);



#ifdef __cplusplus
}
#endif

#endif // ADSR_H
