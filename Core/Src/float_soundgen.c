/*
 * float_soundgen.c
 *
 *  Created on: Feb 15, 2021
 *      Author: nqodo
 */

#include <stdint.h>
#include <math.h>
#include "stm32f4xx.h"

#define BUFSIZE 4096
extern const int TABLESIZE;

extern uint16_t wavetable[];
uint16_t buffer[BUFSIZE];

extern char active_notes[10];
extern int active_count;
float table_steps[88];
float table_indeces[88];


void fl_fill_buffer(uint16_t * buffer, int num_samples) {
	int i, j, note, sample;
	for (i = 0; i < num_samples; i+=2) {
		sample = 0;
		for (j = 0; j < active_count; j++) {
			note = active_notes[j];
			sample += wavetable[(int)table_indeces[note]];
			table_indeces[note] += table_steps[note];
			if ((table_indeces[note]) >= TABLESIZE)
				table_indeces[note] -= TABLESIZE;
		}
		sample /= 16; //what is the magic number here?
		if (sample > 0xffff) sample = 0xffff;
		buffer[i] = buffer[i+1] = sample;
	}
}

void fl_init_note_steps(void) {
	double freq;
	for (int k = 0; k < 88; k++) {
		freq = pow(1.05946309436, k - 48) * 440;
		table_steps[k] = freq * TABLESIZE / 48000;
	}
}
