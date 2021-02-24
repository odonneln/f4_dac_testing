/*
 * sound_gen.c
 *
 *  Created on: Feb 12, 2021
 *      Author: nqodo
 */

#include <stdint.h>
#include <math.h>
#include "stm32f4xx.h"

#define BUFSIZE 2048
const int BUFFERSIZE = BUFSIZE;
extern int TABLESIZE;

extern uint16_t wavetable[];
uint16_t buffer[BUFSIZE];

char active_notes[10];
int active_count = 0;
int table_steps[88];
int table_indeces[88];


void fill_buffer(uint16_t * buffer, int num_samples) {
	int i, j, note, sample;
	for (i = 0; i < num_samples; i+=2) {
		sample = 0;
		for (j = 0; j < active_count; j++) {
			note = active_notes[j];
			sample += wavetable[table_indeces[note] >> 16];
			table_indeces[note] += table_steps[note];
			if ((table_indeces[note] >> 16) >= TABLESIZE)
				table_indeces[note] -= TABLESIZE << 16;
		}
//		sample /= 16;
	sample /= 32;
//		sample /= 64;
		buffer[i] = buffer[i+1] = sample;
	}
}

void init_note_steps(void) {
	double freq;
	for (int k = 0; k < 88; k++) {
		freq = pow(1.05946309436, k - 48) * 440;
		table_steps[k] = freq * TABLESIZE / 48000 * (1 << 16);
	}
}


void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
//	if (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0))
//		active_count = 0;
	fill_buffer(&buffer[0], BUFSIZE / 2);
}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s) {
	fill_buffer(&buffer[BUFSIZE / 2], BUFSIZE / 2);
}

 void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
//	 for (int i = 1; i < active_count; i++)
//		 active_notes[i-1] = active_notes[i];
	 active_count--;
 }
/* Percoset & Stripper Joint
 void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (active_notes[0] == 66)
		active_notes[0] = 64;
	else if (active_notes[0] == 64)
		active_notes[0] = 63;
	else
		active_notes[0] = 66;
}
*/
