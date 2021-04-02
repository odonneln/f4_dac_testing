/*
 * MIDI_application.c
 *
 *  First created on: 6 dec. 2014
 *      Author: Xavier Halgand
 */

/* Includes ------------------------------------------------------------------*/

//#include "main.h"
#include "MIDI_application.h"
#include "adsr.h"
#include "soundgen.h"
//#include <string.h>

/* Private define ------------------------------------------------------------*/

#define RX_BUFF_SIZE 64 /* USB MIDI buffer : max received data 64 bytes */
#define LOWEST_NOTE			21  /* Lowest note : 21 is MIDI note number for A0 */

uint8_t MIDI_RX_Buffer[RX_BUFF_SIZE]; // MIDI reception buffer

//int8_t currentNote;
//int8_t velocity;
//uint8_t notes_On[128] = {0};
//int8_t notesCount = 0; // number of notes on (keys pressed)
//extern ADSR_t adsr;
extern bool sequencerIsOn;

// our DAC sound gen variables:
extern char active_notes[10];
extern int active_count;
char msg[20];

UART_HandleTypeDef huart4;

/* Private function prototypes -----------------------------------------------*/
void ProcessReceivedMidiDatas(void);

/*-----------------------------------------------------------------------------*/
/**
 * @brief  Main routine for MIDI application, looped in main.c
 * @param  None
 * @retval none
 */
void MIDI_Application(void)
{
	if (Appli_state == APPLICATION_READY)
	{
		USBH_MIDI_Receive(&hUSBHost, MIDI_RX_Buffer, RX_BUFF_SIZE); // just once at the beginning, start the first reception
		Appli_state = APPLICATION_RUNNING;
	}
	if (Appli_state == APPLICATION_RUNNING)
	{
		//....pffff......grrrrr......
	}
	if (Appli_state == APPLICATION_DISCONNECT)
	{
		Appli_state = APPLICATION_IDLE;
		USBH_MIDI_Stop(&hUSBHost);
	}
}

/*-----------------------------------------------------------------------------*/
/**
 * @brief  MIDI data receive callback.
 * @param  phost: Host handle
 * @retval None
 */
void USBH_MIDI_ReceiveCallback(USBH_HandleTypeDef *phost)
{
	ProcessReceivedMidiDatas();
	USBH_MIDI_Receive(&hUSBHost, MIDI_RX_Buffer, RX_BUFF_SIZE); // start a new reception
}

void ProcessReceivedMidiDatas(void)
{
	uint16_t numberOfPackets;
	uint8_t *ptr = MIDI_RX_Buffer;
	midi_package_t pack;

	numberOfPackets = USBH_MIDI_GetLastReceivedDataSize(&hUSBHost) / 4; //each USB midi package is 4 bytes long

	while (numberOfPackets--)
	{
		pack.cin_cable = *ptr;
		ptr++;
		pack.evnt0 = *ptr; // note ON or OFF -- already tracked by active_notes[]
		ptr++;
		pack.evnt1 = *ptr; // note number
		ptr++;
		pack.evnt2 = *ptr;
		ptr++;

//		if (pack.cin_cable != 0) // if incoming midi message...
			//start_LED_On(LED_Blue, 8);

		int note = note_to_int(pack.evnt1);

		if ((pack.evnt0 & 0xF0) == 0x80) {
			// note off
			remove_note(note);
		} else if ((pack.evnt0 & 0xF0) == 0x90) {
			//note ON
			if (pack.evnt2 == 0)
				remove_note(note);
			else
				add_note(note);
		}


		if ((pack.evnt0 & 0xF0) == 0xB0) /* If incoming midi message is a Control Change... */
		{
			uint8_t val = pack.evnt2;
		}
	}
}
