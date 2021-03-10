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

/*
void HAL_SPI_RxCpltCallback (SPI_HandleTypeDef * hspi) {
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
}
*/

/* helper functions for when wavetable can't be recvd from F4 */
void gen_sine(void) {
	int x;
	for(x=0; x<N; x++)
		wavetable[x] = 32767 * sin(2 * M_PI * x / N);
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
