/**
  ******************************************************************************
  * @file    ex18_fr_cli/main.c 
  * @author  MDS
  * @date    9-May-2014
  * @brief   FreeRTOS CLI program.Creates a task to implement the CLI and flash
  *			 the onboard Blue LED.
  *
  *			 Implements the echo command  
  *			 See the FreeRTOSPlus CLI API for more information
  *			 http://www.freertos.org/FreeRTOS-Plus/FreeRTOS_Plus_CLI
  ******************************************************************************
  *  
  */ 

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "debug_printf.h"
#include <string.h>
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "FreeRTOS_CLI.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void Hardware_init();
void ApplicationIdleHook( void ); /* The idle hook is used to blink the Blue 'Alive LED' every second */
void LED_Task( void );
void CLI_Task(void);
static BaseType_t prvEchoCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );

/* Private variables ---------------------------------------------------------*/
CLI_Command_Definition_t xEcho = {	/* Structure that defines the "echo" command line command. */
	"echo",
	"echo: Echo the input.\r\n",
	prvEchoCommand,
	1
};

/* Task Priorities ------------------------------------------------------------*/
#define mainLED_PRIORITY					( tskIDLE_PRIORITY + 1 )
#define mainCLI_PRIORITY					( tskIDLE_PRIORITY + 2 )

/* Task Stack Allocations -----------------------------------------------------*/
#define mainLED_TASK_STACK_SIZE		( configMINIMAL_STACK_SIZE * 2 )
#define mainCLI_TASK_STACK_SIZE		( configMINIMAL_STACK_SIZE * 3 )

/**
  * @brief  Starts all the other tasks, then starts the scheduler.
  * @param  None
  * @retval None
  */
int main( void ) {

	BRD_init();
	Hardware_init();
	
	/* Start the tasks to flash the LED and start the CLI. */
    xTaskCreate( (void *) &LED_Task, (const signed char *) "LED", mainLED_TASK_STACK_SIZE, NULL, mainLED_PRIORITY, NULL );
	xTaskCreate( (void *) &CLI_Task, (const signed char *) "CLI", mainCLI_TASK_STACK_SIZE, NULL, mainCLI_PRIORITY, NULL );
	
	/* Register CLI commands */
	FreeRTOS_CLIRegisterCommand(&xEcho);

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
void LED_Task( void ) {

	BRD_LEDOff();

	for (;;) {

		/* Toggle LED */
		BRD_LEDToggle();

		/* Delay the task for 1000ms */
		vTaskDelay(1000);

	}
}


/**
  * @brief  CLI Receive Task.
  * @param  None
  * @retval None
  */
void CLI_Task(void) {

	char cRxedChar;
	char cInputString[100];
	int InputIndex = 0;
	char *pcOutputString;
	BaseType_t xReturned;

	/* Initialise pointer to CLI output buffer. */
	memset(cInputString, 0, sizeof(cInputString));
	pcOutputString = FreeRTOS_CLIGetOutputBuffer();

	for (;;) {

		/* Receive character */
		cRxedChar = debug_getc();

		/* Process if chacater if not Null */
		if (cRxedChar != '\0') {

			/* Put byte into USB buffer */
			debug_putc(cRxedChar);
		
			/* Process only if return is received. */
			if (cRxedChar == '\r') {

				//Put new line and transmit buffer
				debug_putc('\n');
				debug_flush();

				/* Put null character in command input string. */
				cInputString[InputIndex] = '\0';
				
				xReturned = pdTRUE;
				/* Process command input string. */
				while (xReturned != pdFALSE) {

					/* Returns pdFALSE, when all strings have been returned */
					xReturned = FreeRTOS_CLIProcessCommand( cInputString, pcOutputString, configCOMMAND_INT_MAX_OUTPUT_SIZE );

					/* Display CLI output string */
					debug_printf("%s\n\r",pcOutputString);
				    vTaskDelay(5);	//Must delay between debug_printfs.
				}

				memset(cInputString, 0, sizeof(cInputString));
				InputIndex = 0;

			} else {

				debug_flush();		//Transmit USB buffer

				if( cRxedChar == '\r' ) {

					/* Ignore the character. */
				} else if( cRxedChar == '\b' ) {

					/* Backspace was pressed.  Erase the last character in the
					 string - if any.*/
					if( InputIndex > 0 ) {
						InputIndex--;
						cInputString[ InputIndex ] = '\0';
					}

				} else {

					/* A character was entered.  Add it to the string
					   entered so far.  When a \n is entered the complete
					   string will be passed to the command interpreter. */
					if( InputIndex < 20 ) {
						cInputString[ InputIndex ] = cRxedChar;
						InputIndex++;
					}
				}
			}		
		}

		vTaskDelay(50);
	}
}

/**
  * @brief  Echo Command.
  * @param  writebuffer, writebuffer length and command strength
  * @retval None
  */
static BaseType_t prvEchoCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString ) {

	long lParam_len; 
	const char *cCmd_string;

	/* Get parameters from command string */
	cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);

	/* Write command echo output string to write buffer. */
	xWriteBufferLen = sprintf((char *) pcWriteBuffer, "\n\r%s\n\r", cCmd_string);

	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
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

