/* -----------------------------------------------------------------------------
# Company name: CSSE3010 - Embedded Systems Design & Interfacing
# Project name: Practical 7 - FreeRTOS Networking
# Module name : TUIO Client
# Functionality: Extract TUIO /tuio/2dobj set messages on TCP Port 3000
# Hardware platform: NetduinoPlus (AT91SAM7)
#
# Author name: M. D'Souza
# Creation date: 060812
# Revision date (name): -
# Changes implemented (date): -
#(Comments): 
------------------------------------------------------------------------------*/

#ifndef __TUIOCLIENT_H__
#define __TUIOCLIENT_H__

 /*Next, we define the uip_tcp_appstate_t datatype. This is the state
   of our application, and the memory required for this state is
   allocated together with each TCP connection. One application state
   for each TCP connection. */
typedef struct tuiomessage {
	int session_id;;			 
	int class_id;
	float position_x; 	/* range 0...1 */
	float position_y; 	/* range 0...1 */
	float angle_a; 		/* range 0..2PI */
	float velocity_x; 
	float velocity_y; 
	float rotation_velocity_a; 
	float motion_acceleration; 
	float rotation_acceleration;
} tuiomessage_t;

/* Tuico client parser. */
void tuioclient_parser(unsigned char *buffer, tuiomessage_t *msg);

#endif /* __TUIOCLIENT_H__ */

