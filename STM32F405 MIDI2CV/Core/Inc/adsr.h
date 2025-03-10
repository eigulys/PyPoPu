// adsr.h

#ifndef ADSR_H
#define ADSR_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>

#define NUM_ENVELOPES 4

#ifdef __cplusplus
extern "C" {
#endif


#define ATTACK_RATE_MAX 127
#define DECAY_RATE_MAX 127
#define SUSTAIN_LEVEL_MAX 1.0f
#define SUSTAIN_LEVEL_MIN 0.0f
#define RELEASE_RATE_MAX 127
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
    int attack_index;
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
void ADSR_SetAttackRate(ADSR_t *adsr, int increment);
//void ADSR_SetAttackRate(ADSR_t *adsr, int index);
void ADSR_SetDecayRate(ADSR_t *adsr, int index);
void ADSR_SetSustainLevel(ADSR_t *adsr, float sustain_level);
void ADSR_SetReleaseRate(ADSR_t *adsr, int index);
void ADSR_SetAmplitude(ADSR_t *adsr, float amplitude);
void ADSR_UpdateParametersWithEncoders(ADSR_t *adsr);


#ifdef __cplusplus
}
#endif

#endif // ADSR_H
