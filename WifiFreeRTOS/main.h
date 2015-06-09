#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
//#define MASTERNODE

#ifndef MASTERNODE
    #define NODE_ID 1
#else
    #define NODE_ID 0
#endif

#define BUFFER_SIZE 256
#define AUDIO_PRIORITY					( tskIDLE_PRIORITY + 4 )
#define AUDIO_STACK_SIZE		( configMINIMAL_STACK_SIZE * 2 )

#define TX_PRIORITY					( tskIDLE_PRIORITY + 2 )
#define TX_STACK_SIZE		( configMINIMAL_STACK_SIZE * 10 )

#define PACKETFORMAT "DA:[%d%d%d%s]"

void Hardware_init();
void timer_interupt_init( void );
void adc_hardware_init();
void adc_switch_channel_0( void );
void adc_switch_channel_1( void );

//FreeRTOS
void ApplicationIdleHook( void ); /* The idle hook is used to blink the Blue 'Alive LED' every second */
void Audio_Task( void );
void TX_Task( void );
void rateChecker( void );
void Software_timer( void );

void tim2_irqhandler( void );
void tim3_dma( void );

void DMACompleteISR1( void );
void DMACompleteISR2( void );
void Error_Handler( void );
void Delay(uint32_t cycles);