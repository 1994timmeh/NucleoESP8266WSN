/**
  ******************************************************************************
  * @file    ex14_fr_ledflashing/main.c
  * @author  MDS
  * @date    04022015
  * @brief   FreeRTOS LED Flashing program.Creates a task to flash the onboard
  *			 Blue LED. Note the Idle task will also flash the Blue LED.
  ******************************************************************************
  *
  */

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "debug_printf.h"
#include "ESP8622.h"
#include "stddef.h"
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#define NODE_ID 1
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
void Hardware_init();
void ApplicationIdleHook( void ); /* The idle hook is used to blink the Blue 'Alive LED' every second */
void LED_Task( void );
void Testing_Task( void );

/* Task Priorities ------------------------------------------------------------*/
#define mainLED_PRIORITY					( tskIDLE_PRIORITY + 2 )

/* Task Stack Allocations -----------------------------------------------------*/
#define mainLED_TASK_STACK_SIZE		( configMINIMAL_STACK_SIZE * 2 )

extern int g_rssi;

/**
  * @brief  Starts all the other tasks, then starts the scheduler.
  * @param  None
  * @retval None
  */
int main( void ) {

	BRD_init();
	Hardware_init();
	ESP8622_init(); //This initiates another task

	/* Start the task to flash the LED. */
  xTaskCreate( (void *) &LED_Task, (const signed char *) "LED", mainLED_TASK_STACK_SIZE, NULL, mainLED_PRIORITY, NULL );
	xTaskCreate( (void *) &Testing_Task, (const signed char *) "TEST", mainLED_TASK_STACK_SIZE, NULL, mainLED_PRIORITY, NULL );

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
  * @brief  LED Flashing Task.
  * @param  None
  * @retval None
  */
void Testing_Task( void ) {
	char SSID[50];

  Wifi_reset();

	sprintf(&(SSID[0]), "NUCLEOWSN%d", NODE_ID);
	Wifi_setmode();

	Wifi_setAP(SSID,"password", 5, 0);
	//
	// //Wifi_join("Hadwen AirPort", "5Awr2juW");
	// //Wifi_join("Wi-Fi 4G-F1AC", "4132706873");
	//
	// Wifi_enserver();
	//
	// Wifi_getip();

	for (;;) {

		/* Toggle LED */
		Wifi_listAPs();

		/* Delay the task for 1000ms */
		vTaskDelay(1);
	}
}

void LED_Task( void ){
	for(;;){
		BRD_LEDToggle();
		vTaskDelay(100/(g_rssi) * 30);
	}
}

/**
  * @brief  Hardware Initialisation.
  * @param  None
  * @retval None
  */
void Hardware_init( void ) {

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
  * @brief  Idle Application Task
  * @param  None
  * @retval None
  */
void vApplicationIdleHook( void ) {
	static portTickType xLastTx = 0;

	BRD_LEDOff();

	for (;;) {

		/* The idle hook simply prints the idle tick count, every second */
		if ((xTaskGetTickCount() - xLastTx ) > (1000 / portTICK_RATE_MS)) {

			xLastTx = xTaskGetTickCount();

			//debug_printf("IDLE Tick %d\n", xLastTx);

			/* Blink Alive LED */
			BRD_LEDToggle();
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
