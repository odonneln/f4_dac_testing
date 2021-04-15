/*
 * sound_gen.c
 *
 *  Created on: Feb 12, 2021
 *      Author: nqodo
 */

#include <stdint.h>
#include <math.h>
#include "stm32f4xx.h"
#include "adsr.h"

#define BUFSIZE 256

const int BUFFERSIZE = BUFSIZE;
extern int TABLESIZE;

extern volatile int16_t wavetable[];
int16_t buffer[BUFSIZE];

char active_notes[10];
int active_count = 0;
int table_steps[88];
int table_indeces[88];

void fill_buffer(int16_t * buffer, int num_samples) {
	int i, j, note, sample;
	for (i = 0; i < num_samples; i+=2) {
		sample = 0;
		for (j = 0; j < active_count; j++) {
			note = active_notes[j];
			sample += get_multiplier(note) * wavetable[table_indeces[note] >> 16];
			table_indeces[note] += table_steps[note];
			if ((table_indeces[note] >> 16) >= TABLESIZE)
				table_indeces[note] -= TABLESIZE << 16;
		}
		sample_finished();
		uint8_t r_count = get_released_count();
		uint8_t *loc = get_released_array();
		for(int f = 0; f < r_count; f++) {
			sample += get_released_mult(loc[f]) * wavetable[table_indeces[loc[f]] >> 16];
		}
		if (sample > 0xffff / 2)
			sample = 0xffff / 2;
		else if (sample < - 0xffff / 2)
			sample = - 0xffff / 2;
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

void half_complete() {
	fill_buffer(&buffer[0], BUFSIZE / 2);
}

void full_complete() {
	fill_buffer(&buffer[BUFSIZE / 2], BUFSIZE / 2);
}

int note_to_int(char c) {
	if(c < 21 || c > 21 + 87) return -1; // note valid

	return (int)c - 21; //TODO lowest note const
}

void remove_note(int note) {
	int loc = -1;
	for(int i = 0; i < active_count; i++) {
		if(active_notes[i] == note) {
			loc = i;
			break;
		}
	}
	if (loc == -1)
		return;
	active_count--;
	// if the note is at the end, just decrement active notes
	if(loc == active_count) return;
	// swap what is at the end for what is now removed
	active_notes[loc] = active_notes[active_count];
	note_released(note);
}

void add_note(int note) {
	// don't exceed max
	if(active_count >= 10) return;
	for(int i = 0; i < active_count; i++) {
		if(active_notes[i] == note)
			return;
	}
	active_notes[active_count++] = note;
	note_pressed(note);
}
