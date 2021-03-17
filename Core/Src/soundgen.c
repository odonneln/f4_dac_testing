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

extern int16_t wavetable[];
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
			sample += wavetable[table_indeces[note] >> 16];
			table_indeces[note] += table_steps[note];
			if ((table_indeces[note] >> 16) >= TABLESIZE)
				table_indeces[note] -= TABLESIZE << 16;
		}

		sample /= 16; // for easier speaker testing
		if (active_count > 1 && active_count < 3) //this is not fine tuned yet
			sample /= active_count;

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
//	if (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0))
//		active_count = 0;
	fill_buffer(&buffer[0], BUFSIZE / 2);
}

void full_complete() {
	fill_buffer(&buffer[BUFSIZE / 2], BUFSIZE / 2);
}

int note_to_int(char c) {
	if(c < 'A' || c > 'z') return -1; // note valid

	return c - 'A';
}

void remove_note(int loc, int find) {
	active_count--;
	// if the note is at the end, just decrement active notes
	if(loc == active_count) return;
	// swap what is at the end for what is now removed
	active_notes[loc] = active_notes[active_count];
}

void add_note(int add) {
	// don't exceed max
	if(active_count == 10) return;

	active_notes[active_count++] = add;
}

void midi_note_received(char c) {
	int loc = -1;
	int find = note_to_int(c);
	if(find == -1) return; // invalid note
	for(int i = 0; i < active_count; i++) {
		if(active_notes[i] == find) {
			loc = i;
			break;
		}
	}
	if(loc == -1) add_note(find);
	else remove_note(loc, find);
}
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
//	 for (int i = 1; i < active_count; i++) active_notes[i-1] = active_notes[i];
//	 active_count--;
//	 if (active_count == 3)
//		 HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
// }

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
