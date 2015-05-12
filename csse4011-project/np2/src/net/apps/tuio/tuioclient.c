/* -----------------------------------------------------------------------------
# Company name: CSSE3010 - Embedded Systems Design & Interfacing
# Module name : TUIO Client
# Functionality: Parse and extract TUIO /tuio/2dobj 
# See http://www.tuio.org/?specification TUIO 1.1 Specification.
#
# Author name: M. D'Souza
# Creation date: 220414
# Revision date (name): -
# Changes implemented (date): -
#(Comments):
------------------------------------------------------------------------------*/

#include "tuioclient.h"
#include <string.h>


#define	MSG_OFFSET	8
#define BREAK_MESSAGE_LOOP	3000

//#define DEBUG 1	//Uncomment to include debug messages

#ifdef DEBUG
#include "debug_printf.h"
#endif


/*---------------------------------------------------------------------------*/
/* Function called to parse TUIO message and extract orientation parameters */
void tuioclient_parser(unsigned char *buffer, tuiomessage_t *msg) {

	/* TUIO Attributes: s i x y a X Y A m r */ 
	/* See TUIO 1.1 Specification, Table 1. */
	//int session_id, class_id; /* TUIO session and class ID attributes */
	//float position_x, position_y, angle_a, velocity_x, velocity_y, rotation_velocity_a, motion_acceleration, rotation_acceleration;	

	int i, j, k, msg_index, msg_size;
	unsigned int tuio_attribute[10];

	memset(tuio_attribute, 0, sizeof(tuio_attribute)); 
 	
	/* Example Message = HeaderPayload:
		Header = /tuio/2Dobj,siiffffffffset	
		Payload = sixyaXYAmr */
	char *tuio_2dobj_set = "siiffffffff"; /* /tuio/2Dobj message field to search for (restricted to 11 characters). */

	msg_index = -1;
	i = 20;		/* Skip the first 20 characters of the message. */
	msg_size = strlen(tuio_2dobj_set);

	/* Search for /tuio/2Dobj set message. Note: message is part of OSC (Open Sound Control) 1.1 bundle message. */
	while (i < 100) {

		/* Check for first character of /tuio/2Dobj set message */
		if (buffer[i] == tuio_2dobj_set[0]) {

			k = 0;
			for (j=0; j < msg_size; j++) { 
			
				if (buffer[i+j] == tuio_2dobj_set[j]) {
					k++;
				}
			}

			/* If /tuio/2Dobj set message detected, breakout of loop */
			if (k == msg_size) {
				msg_index = i;
				i = BREAK_MESSAGE_LOOP;
			}
		}
		i++;
	}

	/* If TUIO /tuio/2Dobj set message detected, extract TUIO attributes. */
	if (msg_index >= 0) {

		/* Extract the TUIO attributes. See TUIO 1.1 Specification, Table 1. */
		for (i = 0; i < 10; i++) {
			tuio_attribute[i] = (unsigned int)((buffer[msg_index + msg_size + MSG_OFFSET + (4*i)] << 24) | (buffer[msg_index + msg_size + MSG_OFFSET + 1 + (4*i)] << 16) | (buffer[msg_index + msg_size + MSG_OFFSET + 2 + (4*i)] << 8) | buffer[msg_index + msg_size + MSG_OFFSET + 3 + (4*i)]);
		}

		/* Set TUIO attributes. See TUIO 1.1 Specification, Table 1. */
		msg->session_id				= (int) tuio_attribute[0];			 
		msg->class_id 				= (int) tuio_attribute[1];
		msg->position_x 			= *(float *)&tuio_attribute[2]; 	/* range 0...1 */
		msg->position_y 			= *(float *)&tuio_attribute[3]; 	/* range 0...1 */
		msg->angle_a 				= *(float *)&tuio_attribute[4]; 	/* range 0..2PI */
		msg->velocity_x				= *(float *)&tuio_attribute[5]; 
		msg->velocity_y 			= *(float *)&tuio_attribute[6]; 
		msg->rotation_velocity_a 	= *(float *)&tuio_attribute[7]; 
		msg->motion_acceleration 	= *(float *)&tuio_attribute[8]; 
		msg->rotation_acceleration 	= *(float *)&tuio_attribute[9];

#ifdef DEBUG
		debug_printf("s:%d i:%d x:%d y:%d a:%d ", msg->session_id, msg->class_id, (int)(msg->position_x*100.0f), (int)(msg->position_y*100.0f), (int)(msg->angle_a*100.0f)); 
		debug_printf("X:%d Y:%d A:%d m:%d r:%d \n", (int)(msg->velocity_x*100.0f), (int)(msg->velocity_y*100.0f), (int)(msg->rotation_velocity_a*100.0f), (int)(msg->motion_acceleration*100.0f), (int)(msg->rotation_acceleration*100.0f));	
#endif	
	}	

}

