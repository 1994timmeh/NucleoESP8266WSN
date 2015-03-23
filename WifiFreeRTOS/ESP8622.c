#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "debug_printf.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "ESP8622.h"
#include <string.h>

UART_HandleTypeDef UART_Handler;
QueueHandle_t UARTQueue_TX;	/* Queue used */
QueueHandle_t UARTQueue_RX;	/* Queue used */


char test_message[30] = "NUCLEO-F401RE TEST MESSAGE\n";

/**
  * This module is for the ESP8622 'El Cheapo' Wifi Module.
  * It uses pins D10 (TX) and D2 (RX) and Serial 1 on the NucleoF401RE Dev Board
  *
  * @author Timmy Hadwen
  * @author Michael Thoreau
  */
void 	ESP8622_init( void ){
  GPIO_InitTypeDef GPIO_serial;

  __USART1_CLK_ENABLE();
  __BRD_D10_GPIO_CLK();
  __BRD_D2_GPIO_CLK();

  /* Configure settings for USART 6 */
  UART_Handler.Instance = (USART_TypeDef *)USART1_BASE;		//USART 1
  UART_Handler.Init.BaudRate   = 9600;	             			//Baudrate
  UART_Handler.Init.WordLength = UART_WORDLENGTH_8B;    	//8 bits data length
  UART_Handler.Init.StopBits   = UART_STOPBITS_1;	      	//1 stop bit
  UART_Handler.Init.Parity     = UART_PARITY_NONE;    		//No paraity
  UART_Handler.Init.Mode = UART_MODE_TX_RX;		           	//Set for Transmit and Receive mode
  UART_Handler.Init.HwFlowCtl = UART_HWCONTROL_NONE;	   	//Set HW Flow control to none.

  /* Configure the D2 as the RX pin for USARt1 */
  GPIO_serial.Pin = BRD_D2_PIN;
  GPIO_serial.Mode = GPIO_MODE_AF_PP;				             	//Enable alternate mode setting
  GPIO_serial.Pull = GPIO_PULLDOWN;
  GPIO_serial.Speed = GPIO_SPEED_HIGH;
  GPIO_serial.Alternate = GPIO_AF7_USART1;	           		//Set alternate setting to USART 6
  HAL_GPIO_Init(BRD_D2_GPIO_PORT, &GPIO_serial);

  /* Configure the D1 as the tX pin for USART6 */
  GPIO_serial.Pin = BRD_D10_PIN;
  GPIO_serial.Mode = GPIO_MODE_AF_PP;				             	//Enable alternate mode setting
  GPIO_serial.Pull = GPIO_PULLUP;
  GPIO_serial.Speed = GPIO_SPEED_HIGH;
  GPIO_serial.Alternate = GPIO_AF7_USART1;		          	//Set alternate setting to USART 6
  HAL_GPIO_Init(BRD_D10_GPIO_PORT, &GPIO_serial);

  /* Initialise USART */
  HAL_UART_Init(&UART_Handler);

  xTaskCreate( (void *) &UARTHandlerTX_task, (const signed char *) "UART_TX", mainLED_TASK_STACK_SIZE, NULL, mainLED_PRIORITY, NULL );
  xTaskCreate( (void *) &UARTHandlerRX_task, (const signed char *) "UART_RX", mainLED_TASK_STACK_SIZE, NULL, mainLED_PRIORITY, NULL );

	UARTQueue_TX = xQueueCreate(10, sizeof(UART_Message));
  UARTQueue_RX = xQueueCreate(100, sizeof(char));
}

void UARTHandlerTX_task( void ) {
  Wifi_reset();
  Wifi_setmode();
  Wifi_listAPs();

	task_loop{

		//debug_printf("UART Handler Task\n\r");

		/* Delay the task for 1000ms */
		vTaskDelay(1000);

	}
}

void UARTHandlerRX_task( void ) {

  char rx_char = 0;
	task_loop{

    while(HAL_UART_Receive(&UART_Handler, &rx_char, 1, 3000) == HAL_OK){
      if (UARTQueue_RX != NULL) {	/* Check if queue exists */
        xQueueSendToFront(UARTQueue_RX, ( void * ) &rx_char, ( portTickType ) 10 );
        debug_printf("Added charecter to RX Queue");
  		}
    }
		/* Delay the task for 1000ms */
		vTaskDelay(1000);
	}
}


//############################ HELPER FUNCTIONS ###############################

void waitFor( char x ){
  char rx_char = 0;
  while(rx_char != x){
    if(xQueueReceive( UARTQueue_RX, &rx_char, 10 )){
      debug_printf("Found char %c\n", rx_char);
    }

  }
}

void waitForOK(){
  waitFor('O');
  waitFor('K');
}

void waitForReady(){
  waitFor('r');
  waitFor('e');
  waitFor('a');
  waitFor('d');
  waitFor('y');
}

/* Resets the wifi module */
void Wifi_reset(){
  char command[20] = WIFI_CMD_RST;
  char rx_char = 0;
  int timeout = 0;

  debug_printf("Reseting module... Please wait\n");

  HAL_UART_Transmit(&UART_Handler, &(command[0]), WIFI_LEN_RST, 10);

  Delay(SEC);

  waitForReady();

  Delay(SEC);

  debug_printf("\nModule is ready\n");
}

/* Joins my home network */
void Wifi_join(){
  char command[50] = WIFI_CMD_JOIN_TIMMY_HOME;

  debug_printf("Joining network\n");

  HAL_UART_Transmit(&UART_Handler, &(command[0]), WIFI_LEN_JOIN_TIMMY_HOME, 10);

  Delay(SEC);

  waitForOK();

  Delay(SEC);
}

/* Currently sets mode to 3 -Both AP and ST) */
void Wifi_setmode(){
  char command[50] = WIFI_CMD_MODE_BOTH;

  debug_printf("Setting module mode\n");

  HAL_UART_Transmit(&UART_Handler, &(command[0]), WIFI_LEN_MODE_BOTH, 10);

  waitFor('n');
  waitFor('o');

  Delay(SEC);
}

/* Lists the AP names in return type
 * PROBLEMS
 * ===============
 * - For some reason the uart receive code wont work. Just gives 'A' once
 * @unfinsihed
 */
void Wifi_listAPs(){
  char command[50] = WIFI_CMD_LIST_APS;
  char rx_char;
  char ap_names[100];
  int i, j;

  char ap1[50]; int rssi1 = 0; int ap1c = 0; char rssi1c[10];
  char ap2[50]; int rssi2 = 0; int ap2c = 0; char rssi2c[10];

  debug_printf("Getting AP Names\n");

  HAL_UART_Transmit(&UART_Handler, &(command[0]), WIFI_LEN_LIST_APS, 10);

  waitForOK();
}

/* Sends the status command
 * @unfinished
 */
void Wifi_status(){
  char command[50] = WIFI_CMD_STATUS;
  HAL_UART_Transmit(&UART_Handler, &(command[0]), WIFI_LEN_STATUS, 10);
}

/* Sets the wifi ap
 * @unfinsihed
 */
void Wifi_setAP(){
  char command[50] = WIFI_CMD_SET_AP;
  HAL_UART_Transmit(&UART_Handler, &(command[0]), WIFI_LEN_SET_AP, 10);
}

/* Checks the IP address
 * @unfinished
 * @broken
 */
void Wifi_checkcon(){
  char command[50] = "AT+CWJAP\n\r";
  HAL_UART_Transmit(&UART_Handler, &(command[0]), 12, 10);
}

/*
 * Enables a TCP server on port 8888
 */
void Wifi_enserver(){
  char command[50] = WIFI_CMD_MUX_1;

  debug_printf("Enabling a server on 8888\n");

  HAL_UART_Transmit(&UART_Handler, &(command[0]), WIFI_LEN_MUX_1, 10);
  waitForOK();

  memcpy(&(command[0]), WIFI_CMD_SERVE, WIFI_LEN_SERVE);
  HAL_UART_Transmit(&UART_Handler, &(command[0]), WIFI_LEN_SERVE, 10);
  waitForOK();
}

void Delay(int x){
  while(x--);
}
