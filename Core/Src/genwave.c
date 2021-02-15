/*
 * genwave.c
 *
 *  Created on: Feb 14, 2021
 *      Author: nqodo
 */

#include <stdio.h>
#include <math.h>

#define N 2048

uint16_t wavetable[N];

void gen_sine(void) {
	int x;
	for(x=0; x<N; x++) {
		// 2^16 / 2 = 32768
		wavetable[x] = 32767 + 32767 * sin(2 * M_PI * x / N);
//		wavetable[x] = 32767 * sin(2 * M_PI * x / N);
		/* NOTICE those two definitions make different sounds! */
	}
}

void gen_square(void) {
	int x;
	for(x=0; x<N/2; x++)
		wavetable[x] = 2 * 32767;
	for(x=N/2; x<N; x++)
		wavetable[x] = 0;
}

void gen_sawtooth(void) {
	int x;
	for(x=0; x < N; x++)
		wavetable[x] = 32767 + 32767.0 * (x - N/2) / (1.0*N);
}
