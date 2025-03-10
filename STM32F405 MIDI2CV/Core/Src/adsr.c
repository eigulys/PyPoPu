// adsr.c

#include "adsr.h"
#include <stdbool.h>
#include <lookup_t.h>
#include <stdio.h>

extern int16_t ROT1;
extern int16_t ROT2;

// Initializes an individual ADSR instance
//void ADSR_Init(ADSR_t *adsr) {
//    adsr->state = IDLE;
//    adsr->envelope_value = attack_rate_lookup[64];
//    adsr->attack_rate = attack_rate_lookup[64];  // Default rates, can be modified by MIDI or control signals
//    adsr->decay_rate = attack_rate_lookup[64];
//    adsr->sustain_level = 0.6;
//    adsr->release_rate = attack_rate_lookup[64];
//    adsr->gate_signal = 0;
//    adsr->amplitude = 1;      // Default amplitude is 1.0 (full volume)
//}
//extern float lookup_t[128];

void ADSR_Init(ADSR_t *adsr, int num_envelopes) {
    for (int i = 0; i < num_envelopes; i++) {
    	adsr[i].attack_index = 64;  // Set default index
        adsr[i].attack_rate = attack_rate_lookup[64];
        adsr[i].decay_rate = attack_rate_lookup[64];
        adsr[i].sustain_level = 0.5f;
        adsr[i].release_rate = attack_rate_lookup[64];
        adsr[i].amplitude = 1.0f;
    }
}

void ADSR_UpdateParametersWithEncoders(ADSR_t *adsr) {
    int current_attack_index = 0;
    while (current_attack_index < ATTACK_RATE_MAX && attack_rate_lookup[current_attack_index] < adsr->attack_rate) {
        current_attack_index++;
    }
    current_attack_index += ROT1;
    if (current_attack_index < 0) current_attack_index = 0;
    if (current_attack_index > ATTACK_RATE_MAX) current_attack_index = ATTACK_RATE_MAX;
    adsr->attack_rate = attack_rate_lookup[current_attack_index];

}

// In adsr.c

//void ADSR_SetAttackRate(ADSR_t *adsr, int index) {
//    if (index < 0) {
//        index = 0;  // Minimum limit
//    } else if (index >= 128) {
//        index = 127;  // Maximum limit
//    }
//    adsr->attack_rate = attack_rate_lookup[index];
//}
void ADSR_SetAttackRate(ADSR_t *adsr, int increment) {
    adsr->attack_index += increment;  // Increment or decrement the index

    // Limit checking
    if (adsr->attack_index < 0) {
        adsr->attack_index = 0;
    } else if (adsr->attack_index >= 128) {
        adsr->attack_index = 127;
    }

    // Update attack rate from lookup table using current index
    adsr->attack_rate = attack_rate_lookup[adsr->attack_index];
}

void ADSR_SetDecayRate(ADSR_t *adsr, int index) {
    if (index < 0) {
        index = 0;  // Minimum limit
    } else if (index >= 128) {
        index = 127;  // Maximum limit
    }
    adsr->decay_rate = attack_rate_lookup[index];
}

void ADSR_SetSustainLevel(ADSR_t *adsr, float sustain_level) {
	sustain_level += sustain_level * 0.02f;
    if (sustain_level < 0.0f) {
        sustain_level = 0.0f;  // Minimum limit
    } else if (sustain_level > 1.0f) {
        sustain_level = 1.0f;  // Maximum limit
    }
    adsr->sustain_level = (sustain_level * 0.02f);
}

void ADSR_SetReleaseRate(ADSR_t *adsr, int index) {
    if (index < 0) {
        index = 0;  // Minimum limit
    } else if (index >= 128) {
        index = 127;  // Maximum limit
    }
    adsr->release_rate = attack_rate_lookup[index];
}
//void ADSR_UpdateEnvelope_old(ADSR_t *adsr) {
//
//    switch (adsr->state) {
//    for (int i = 0; i < NUM_ENVELOPES; i++) {
//    	switch (envelopes[i].state) {
//        case ATTACK:
//            adsr->envelope_value += adsr->attack_rate;
//            if (adsr->envelope_value >= 1.0f) {
//                adsr->envelope_value = 1.0f;
//                adsr->state = DECAY;
//            }
//            break;
//
//        case DECAY:
//            adsr->envelope_value -= adsr->decay_rate;
//            if (adsr->envelope_value <= adsr->sustain_level) {
//                adsr->envelope_value = adsr->sustain_level;
//                adsr->state = SUSTAIN;
//            }
//            break;
//
//        case SUSTAIN:
//            if (!adsr->gate_signal) {
//                adsr->state = RELEASE;
//            }
//            break;
//
//        case RELEASE:
//            adsr->envelope_value -= adsr->release_rate;
//            if (adsr->envelope_value <= 0.0f) {
//                adsr->envelope_value = 0.0f;
//                adsr->state = IDLE;
//            }
//            break;
//
//        case IDLE:
//            // Do nothing
//            break;
//    }
//}}}

// Updates the envelope for a specific ADSR instance
void ADSR_UpdateEnvelope(ADSR_t *adsr) {

    switch (adsr->state) {
        case ATTACK:
            adsr->envelope_value += adsr->attack_rate;
            if (adsr->envelope_value >= 1.0f) {
                adsr->envelope_value = 1.0f;
                adsr->state = DECAY;
            }
            break;

        case DECAY:
            adsr->envelope_value -= adsr->decay_rate;
            if (adsr->envelope_value <= adsr->sustain_level) {
                adsr->envelope_value = adsr->sustain_level;
                adsr->state = SUSTAIN;
            }
            break;

        case SUSTAIN:
            if (!adsr->gate_signal) {
                adsr->state = RELEASE;
            }
            break;

        case RELEASE:
            adsr->envelope_value -= adsr->release_rate;
            if (adsr->envelope_value <= 0.0f) {
                adsr->envelope_value = 0.0f;
                adsr->state = IDLE;
            }
            break;

        case IDLE:
            // Do nothing
            break;
    }
}

// Sets the gate signal for a specific ADSR instance
void ADSR_SetGateSignal(ADSR_t *adsr, uint8_t gate_signal_value) {
    adsr->gate_signal = gate_signal_value;
    if (adsr->gate_signal) {
        adsr->state = ATTACK;
    } else {
        adsr->state = RELEASE;
    }
}

//void ADSR_SetGateSignal(ADSR_t *adsr, bool gate) {
//    if (gate) {
//        adsr->state = ATTACK;
//    } else {
//        if (adsr->state != IDLE) {
//            adsr->state = RELEASE;
//        }
//    }
//}

// Handles MIDI CC messages for controlling individual ADSR instances


// Set attack rate for a specific ADSR instance
//void ADSR_SetAttackRate(ADSR_t *adsr, float attack_rate) {
//    if (attack_rate < 0.001f) {
//        attack_rate = 0.001f;  // Minimum limit
//    }
//    adsr->attack_rate = attack_rate;
//}

// Set decay rate for a specific ADSR instance
//void ADSR_SetDecayRate(ADSR_t *adsr, float decay_rate) {
//    if (decay_rate < 0.001f) {
//        decay_rate = 0.001f;  // Minimum limit
//    }
//    adsr->decay_rate = decay_rate;
//}
//
//// Set sustain level for a specific ADSR instance
//void ADSR_SetSustainLevel(ADSR_t *adsr, float sustain_level) {
//    if (sustain_level < 0.0f) {
//        sustain_level = 0.0f;  // Minimum limit
//    } else if (sustain_level > 1.0f) {
//        sustain_level = 1.0f;  // Maximum limit
//    }
//    adsr->sustain_level = sustain_level;
//}
//
//// Set release rate for a specific ADSR instance
//void ADSR_SetReleaseRate(ADSR_t *adsr, float release_rate) {
//    if (release_rate < 0.001f) {
//        release_rate = 0.001f;  // Minimum limit
//    }
//    adsr->release_rate = release_rate;
//}
//
//
//// Sets the amplitude for a specific ADSR instance
//void ADSR_SetAmplitude(ADSR_t *adsr, float amplitude) {
//    if (amplitude < 0.0f) {
//        amplitude = 0.0f;  // Ensure amplitude is non-negative
//    } else if (amplitude > 1.0f) {
//        amplitude = 1.0f;  // Cap amplitude to 1.0
//    }
//    adsr->amplitude = amplitude;
//}

// Returns the envelope value (scaled by the amplitude) for a specific ADSR instance
float ADSR_GetEnvelopeValue(const ADSR_t *adsr) {
    return adsr->envelope_value * adsr->amplitude;
}
