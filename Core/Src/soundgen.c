/*
 * sound_gen.c
 *
 *  Created on: Feb 12, 2021
 *      Author: nqodo
 */

#include <stdint.h>
#include <string.h>
#include <math.h>
#include "stm32f4xx.h"
#include "usbh_MIDI.h"

#define BUFSIZE 2048
const int BUFFERSIZE = BUFSIZE;
extern int TABLESIZE;

extern int16_t wavetable[];
int16_t buffer[BUFSIZE];

volatile char active_notes[10];
char active_notes_rmv[10];
char active_notes_add[10];
volatile int active_count = 0;
int table_steps[88];
int table_indeces[88];
volatile int removing_note = 0;
volatile int adding_note = 0;

#include "usbh_MIDI.h"
extern USBH_HandleTypeDef hUSBHost; /* USB Host handle */

void fill_buffer(int16_t * buffer, int num_samples) {
	int i, j, note, sample;

	int divisor = 16; // for easier speaker testing
	if (active_count > 1 && active_count < 3) //this is not fine tuned yet
		divisor /= active_count;

	for (i = 0; i < num_samples; i+=2) {
		sample = 0;
		for (j = 0; j < active_count; j++) {
			note = active_notes[j];
			sample += wavetable[table_indeces[note] >> 16];
			table_indeces[note] += table_steps[note];
			if ((table_indeces[note] >> 16) >= TABLESIZE)
				table_indeces[note] -= TABLESIZE << 16;
		}
		sample /= divisor;
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
	fill_buffer(&buffer[0], BUFSIZE / 4);
	USBH_Process(&hUSBHost);
	fill_buffer(&buffer[BUFSIZE / 4], BUFSIZE / 4);
}

void full_complete() {
	fill_buffer(&buffer[BUFSIZE / 2], BUFSIZE / 4);
	USBH_Process(&hUSBHost);
	fill_buffer(&buffer[3 * BUFSIZE / 4], BUFSIZE / 4);
}

int note_to_int(char c) {
	if(c < 21 || c > 21 + 87) return -1; // note valid

	return (int)c - 21; //TODO lowest note const
}

int remove_note(int note) {
	if(adding_note) return 0;
	removing_note = 1;
	int cpy = active_count;
	int c = 0;
	for(int i = 0; i < cpy; i++) {
		if(active_notes[i] == note) {
			continue;
		}
		active_notes_rmv[c++] = active_notes[i];
	}
	if(cpy != active_count) remove_note(note);
	memcpy(active_notes, active_notes_rmv, 10);
	active_count = c;
	removing_note = 0;
	return 1;
}

int add_note(int note) {
	// don't exceed max
	if(removing_note) return 0;
	adding_note = 1;
	int cpy = active_count;
	if(cpy >= 10) return;
	memcpy(active_notes_add, active_notes, 10);
	for(int i = 0; i < cpy; i++) {
		if(active_notes_add[i] == note)
			return;
	}
	active_notes_add[cpy++] = note;
	memcpy(active_notes, active_notes_add, 10);
	active_count = cpy;
	adding_note = 0;
	return 1;
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
	else remove_note(loc);
}
