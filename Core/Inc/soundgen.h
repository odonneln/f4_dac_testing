/*
 * soundgen.h
 *
 *  Created on: Feb 12, 2021
 *      Author: nqodo
 */

#ifndef SRC_SOUNDGEN_H_
#define SRC_SOUNDGEN_H_



#endif /* SRC_SOUNDGEN_H_ */

void init_note_steps(void);
void fill_buffer(int16_t * buffer, int num_samples);
void half_complete();
void full_complete();
void button_pushed();

void midi_note_received(char c);
int add_note(int add);
int remove_note(int note);
