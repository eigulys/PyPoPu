// lookup_tables.c

#include "lookup_t.h"

// Precomputed lookup table for exponential scaling (0.001 to 0.01 for attack rate)
const float attack_rate_lookup[128] = {
//		0		  1		 	2		  3		  	4		  5			6		  7			8		  9			10		  11		12		  13		14		  15
		1.00000f, 0.50000f, 0.20000f, 0.15000f, 0.10000f, 0.06600f, 0.05000f, 0.04000f, 0.03200f, 0.02750f, 0.02420f, 0.02170f, 0.01930f, 0.01640f, 0.01390f, 0.01300f,
		0.01200f, 0.01960f, 0.00950f, 0.00510f, 0.00500f, 0.00490f, 0.00480f, 0.00470f, 0.00460f, 0.00450f, 0.00440f, 0.00430f, 0.00420f, 0.00410f, 0.00400f, 0.00390f,
		0.00380f, 0.00370f, 0.00360f, 0.00350f, 0.00340f, 0.00330f, 0.00320f, 0.00310f, 0.00300f, 0.00290f, 0.00280f, 0.00270f, 0.00260f, 0.00250f, 0.00240f, 0.00230f,
		0.00220f, 0.00210f, 0.00200f, 0.00190f, 0.00180f, 0.00170f, 0.00160f, 0.00150f, 0.00140f, 0.00135f, 0.00130f, 0.00125f, 0.00120f, 0.00115f, 0.00110f, 0.00107f,
		0.00104f, 0.00101f, 0.00098f, 0.00095f, 0.00092f, 0.00089f, 0.00086f, 0.00084f, 0.00082f, 0.00080f, 0.00078f, 0.00076f, 0.00074f, 0.00072f, 0.00070f, 0.00068f,
		0.00066f, 0.00064f, 0.00062f, 0.00060f, 0.00058f, 0.00056f, 0.00054f, 0.00052f, 0.00050f, 0.00048f, 0.00047f, 0.00046f, 0.00045f, 0.00044f, 0.00043f, 0.00042f,
		0.00041f, 0.00040f, 0.00039f, 0.00038f, 0.00037f, 0.00036f, 0.00035f, 0.00034f, 0.00033f, 0.00032f, 0.00031f, 0.00030f, 0.00029f, 0.00028f, 0.00027f, 0.00026f,
		0.00025f, 0.00024f, 0.00023f, 0.00022f, 0.00021f, 0.00020f, 0.00019f, 0.00018f, 0.00017f, 0.00016f, 0.00015f, 0.00014f, 0.00013f, 0.00012f, 0.00011f, 0.00010f

    // Note: You should have all 128 values precomputed and placed here.
};

// Precomputed lookup table for decay rate
const float decay_rate_lookup[128] = {
    0.0010f, 0.0011f, 0.0012f, /* ... your 128 values ... */ 0.0100f
};

// Precomputed lookup table for release rate
const float release_rate_lookup[128] = {
    0.0010f, 0.0011f, 0.0012f, /* ... your 128 values ... */ 0.0100f
};