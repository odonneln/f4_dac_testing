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

volatile int16_t wavetable[N];
const int TABLESIZE = N;

double wavetable_rms = 0xffff;

/*
void HAL_SPI_RxCpltCallback (SPI_HandleTypeDef * hspi) {
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
}
*/

double compute_wave_RMS(void) {
	double rms = 0;
	for (int i = 0; i < N; i++)
		rms += wavetable[i] * wavetable[i];
	rms /= TABLESIZE;
	rms = sqrt(rms);
	return rms;
}

/* helper functions for when wavetable can't be recvd from F4 */
void gen_sine(void) {
	int x;
	for(x=0; x<N; x++)
		wavetable[x] = 32767 * sin(2 * M_PI * x / N);
	wavetable_rms = compute_wave_RMS();
}
void gen_square(void) {
	int x;
	for(x=0; x<N/2; x++)
		wavetable[x] = 32767;
	for(x=N/2; x<N; x++)
		wavetable[x] = - 32767;
	wavetable_rms = compute_wave_RMS();
}
void gen_sawtooth(void) {
	int x;
	for(x=0; x < N; x++)
		wavetable[x] = 2 * 32767.0 * (x - N/2) / (1.0*N);
	wavetable_rms = compute_wave_RMS();
}
