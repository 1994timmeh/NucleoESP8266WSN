/**
  ******************************************************************************
  * @file    ex16_fr_queue/main.c 
  * @author  MDS
  * @date    22-April-2014
  * @brief   FreeRTOS queue demonstration.Creates a queue to 
  *			 to send an item from one task to another. Open kermit to view output.
  *	
  *			 NOTE: THE IDLE TASK MUST BE DISABLED. 
  ******************************************************************************
  *  
  */ 

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "debug_printf.h"
#include <string.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"


/* Private typedef -----------------------------------------------------------*/
struct Message {	/* Message consists of sequence number and payload string */
	int Sequence_Number;
	char Payload[10];
};

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
QueueHandle_t MessageQueue;	/* Queue used */

/* Private function prototypes -----------------------------------------------*/
static void Hardware_init();
void ApplicationIdleHook( void );
void Sender_Task( void );
void Receiver_Task( void );

/* Task Priorities ------------------------------------------------------------*/
#define mainSENDERTASK_PRIORITY					( tskIDLE_PRIORITY + 2 )
#define mainRECEIVERTASK_PRIORITY				( tskIDLE_PRIORITY + 2 )

/* Task Stack Allocations -----------------------------------------------------*/
#define mainSENDERTASK_STACK_SIZE		( configMINIMAL_STACK_SIZE * 2 )
#define mainRECEIVERTASK_STACK_SIZE		( configMINIMAL_STACK_SIZE * 2 )


/**
  * @brief  Starts all the other tasks, then starts the scheduler.
  * @param  None
  * @retval None
  */
int main( void ) {

	BRD_init();
	Hardware_init();
	
	/* Start sender and receiver tasks */
    xTaskCreate( (void *) &Sender_Task, (const signed char *) "SEND", mainSENDERTASK_STACK_SIZE, NULL, mainSENDERTASK_PRIORITY, NULL );
	xTaskCreate( (void *) &Receiver_Task, (const signed char *) "RECV", mainRECEIVERTASK_STACK_SIZE, NULL, mainRECEIVERTASK_PRIORITY, NULL );

	/* Start the scheduler.

	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used here. */

	vTaskStartScheduler();
	
	/* We should never get here as control is now taken by the scheduler. */
  	return 0;
}

/**
  * @brief  Sender Task.Send a message to the queue, every second.
  * @param  None
  * @retval None
  */
void Sender_Task( void ) {
	
	struct Message SendMessage; 

	MessageQueue = xQueueCreate(10, sizeof(SendMessage));		/* Create queue of length 10 Message items */

	/*Initialise Message Item payload */
	strcpy(SendMessage.Payload, "Item");
	
	SendMessage.Sequence_Number = 0;

	for (;;) {

		
		if (MessageQueue != NULL) {	/* Check if queue exists */

			/*Send message to the front of the queue - wait atmost 10 ticks */
			if( xQueueSendToFront(MessageQueue, ( void * ) &SendMessage, ( portTickType ) 10 ) != pdPASS ) {
				debug_printf("Failed to post the message, after 10 ticks.\n\r");
			}
		}

		SendMessage.Sequence_Number++;		/* Increment Sequence Number */
		
		/* Wait for 1000ms */
		vTaskDelay(1000);

	}
}


/**
  * @brief  Receiver Task. Used to receive messages.
  * @param  None
  * @retval None
  */
void Receiver_Task( void ) {

	struct Message RecvMessage;
	
	BRD_LEDOff();	

	for (;;) {

		if (MessageQueue != NULL) {	/* Check if queue exists */
	

			/* Check for item received - block atmost for 10 ticks */
			if (xQueueReceive( MessageQueue, &RecvMessage, 10 )) {

				/* display received item */
				debug_printf("Received: %s - %d\n\r", RecvMessage.Payload, RecvMessage.Sequence_Number);
				
            	/* Toggle LED */
				BRD_LEDToggle();
        	}
		}	

		/* Delay for 10ms */
		vTaskDelay(10);
	}
}


/**
  * @brief  Hardware Initialisation.
  * @param  None
  * @retval None
  */
static void Hardware_init( void ) {

	portDISABLE_INTERRUPTS();	//Disable interrupts
	BRD_LEDInit();				//Initialise Blue LED
	BRD_LEDOff();				//Turn off Blue LED
	portENABLE_INTERRUPTS();	//Enable interrupts

}

/**
  * @brief  Application Tick Task.
  * @param  None
  * @retval None
  */
void vApplicationTickHook( void ) {

	BRD_LEDOff();
}

/**
  * @brief  Idle Application Task (Disabled)
  * @param  None
  * @retval None
  */
void vApplicationIdleHook( void ) {
	static TickType_t xLastTx = 0;

	BRD_LEDOff();

	for (;;) {
		/* The idle hook simply prints the idle tick count */
		if ((xTaskGetTickCount() - xLastTx ) > (1000 / portTICK_RATE_MS)) {
			xLastTx = xTaskGetTickCount();
			//debug_printf("IDLE Tick %d\n", xLastTx);
			//BRD_LEDToggle();		
		}
	}
}

/**
  * @brief  vApplicationStackOverflowHook
  * @param  Task Handler and Task Name
  * @retval None
  */
void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName ) {
	/* This function will get called if a task overflows its stack.   If the
	parameters are corrupt then inspect pxCurrentTCB to find which was the
	offending task. */

	BRD_LEDOff();
	( void ) pxTask;
	( void ) pcTaskName;

	for( ;; );
}

