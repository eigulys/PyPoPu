// lookup_tables.h

#ifndef LOOKUP_T_H
#define LOOKUP_T_H

#include <stdint.h>

// Extern declarations of lookup tables for attack, decay, and release rates
extern const float attack_rate_lookup[128];
extern const float decay_rate_lookup[128];
extern const float release_rate_lookup[128];
extern const uint16_t fine_cv[127];
extern const uint16_t coarse_cv[5];

#endif // LOOKUP_TABLES_H
