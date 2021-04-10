/*
 * genwave.c
 *
 *  Created on: Feb 14, 2021
 *      Author: nqodo
 */

#include <stdio.h>
#include <math.h>
#include "stm32f4xx.h"

#define N 1746
//#define N 2048

volatile int16_t wavetable[N + 2];
const int TABLESIZE = N;

double compute_wave_RMS(void) {
	double rms = 0;
	for (int i = 0; i < N; i++)
		rms += wavetable[i] * wavetable[i];
	rms /= TABLESIZE;
	rms = sqrt(rms);
	return rms;
}

void scale_wavetable(void) {
	double divisor = sqrt(10) * compute_wave_RMS() / (23169.326506 * 0.3162278 / 2.1);
	int i;
	long max_squared = 0;
	for (i = 0; i < N; i++) {
		wavetable[i] /= divisor;
		if (wavetable[i] * wavetable[i] > max_squared)
			max_squared = wavetable[i] * wavetable[i];
	}
	if (sqrt(max_squared) > 3276) {
		// clipping possible: interference of 10 notes at peak > 0xffff
		for (i = 0; i < N; i++)
			wavetable[i] /= sqrt(max_squared) / 3276.7;
	}
}

/* helper functions for when wavetable can't be recvd from F4 */
void gen_sine(void) {
	int x;
	for(x=0; x<N; x++)
		wavetable[x] = 32767 * sin(2 * M_PI * x / N);
	scale_wavetable();
}
void gen_crest_sine(int exponent) {
	if (exponent % 2 == 0)
		exponent++;
	int x;
	for(x=0; x<N; x++)
		wavetable[x] = 32767 * pow(sin(2 * M_PI * x / N), exponent);
	scale_wavetable();
}
void gen_square(void) {
	int x;
	for(x=0; x<N/2; x++)
		wavetable[x] = 32767;
	for(x=N/2; x<N; x++)
		wavetable[x] = - 32767;
	scale_wavetable();
}
void gen_sawtooth(void) {
	int x;
	for(x=0; x < N; x++)
		wavetable[x] = 2 * 32767.0 * (x - N/2) / (1.0*N);
	scale_wavetable();
}
