#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "debug_printf.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "ESP8622.h"
#include <string.h>

#define TRUE 1
#define FALSE 0

UART_HandleTypeDef UART_Handler;
QueueHandle_t Data_Queue;	/* Queue used */

volatile int lastTaskPassed = FALSE;


char test_message[100] = "+IPD=Hellakshdfijhalsjdhfa\n";
char ok[5] = "OK";
char line_buffer[100];
uint8_t line_buffer_index = 0;

char uart_buffer[100];
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


  /* Configure the D2 as the RX pin for USARt1 */
  GPIO_serial.Pin = BRD_D2_PIN;
  GPIO_serial.Mode = GPIO_MODE_AF_PP;				             	//Enable alternate mode setting
  GPIO_serial.Pull = GPIO_PULLDOWN;
  GPIO_serial.Speed = GPIO_SPEED_HIGH;
  GPIO_serial.Alternate = GPIO_AF7_USART1;	           		//Set alternate setting to USART1
  HAL_GPIO_Init(BRD_D2_GPIO_PORT, &GPIO_serial);

  /* Configure the D10 as the TX pin for USART1 */
  GPIO_serial.Pin = BRD_D10_PIN;
  GPIO_serial.Mode = GPIO_MODE_AF_PP;				             	//Enable al/Users/tim/Downloads/11064190_10205296920015695_250169967_o.jpgternate mode setting
  GPIO_serial.Pull = GPIO_PULLUP;
  GPIO_serial.Speed = GPIO_SPEED_HIGH;
  GPIO_serial.Alternate = GPIO_AF7_USART1;		          	//Set alternate setting to USART1
  HAL_GPIO_Init(BRD_D10_GPIO_PORT, &GPIO_serial);


  UART_Handler.Instance          = USART1;

  UART_Handler.Init.BaudRate     = 9600;
  UART_Handler.Init.WordLength   = UART_WORDLENGTH_8B;
  UART_Handler.Init.StopBits     = UART_STOPBITS_1;
  UART_Handler.Init.Parity       = UART_PARITY_NONE;
  UART_Handler.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  UART_Handler.Init.Mode         = UART_MODE_TX_RX;
  UART_Handler.Init.OverSampling = UART_OVERSAMPLING_16;


  HAL_NVIC_SetPriority(USART1_IRQn, 10, 0);
  NVIC_SetVector(USART1_IRQn, &UART1_IRQHandler);
  HAL_NVIC_EnableIRQ(USART1_IRQn);


  HAL_UART_Init(&UART_Handler);

  /*  Enable RXNE interrupt on USART_1 */

  // enable this to go to infinite loop, ref: esp8266.c; line 96  - 2nd time receive -> task_exit_critical
  //__HAL_UART_ENABLE_IT(&UART_Handler, USART_IT_RXNE);
  if (HAL_UART_Receive_IT((UART_HandleTypeDef*)&UART_Handler, (uint8_t *)uart_buffer, 100) != HAL_OK) {
	  debug_printf("UART Interrupt init FAIL");
  }



  xTaskCreate( (void *) &UART_Processor, (const signed char *) "DATA", mainLED_TASK_STACK_SIZE * 5, NULL, mainLED_PRIORITY + 1, NULL );

  Data_Queue = xQueueCreate(5, sizeof(char[100]));
}

/*
 * Task for processing UART data and adding new data to a data queue
 */
void UART_Processor( void ){
  char new_data[100];

  for(;;){

      if(xQueueReceive(Data_Queue, &new_data, 10) && new_data[0] != '\r'){
        //We have new data analyze it
        if(strncmp(&(new_data[0]), "+IPD", 4) == 0){
          debug_printf("Data: %s\n", &(new_data[5]));
        } else if(strncmp(&(new_data[0]), "OK", 2) == 0 || strncmp(&(new_data[0]), "ready", 5) == 0
    || strncmp(&(new_data[0]), "no change", 9) == 0){
          //Set the last task passed flag
          lastTaskPassed = TRUE;
        }
      }
      vTaskDelay(100);
  }
}

/*
 * Usart_1 interrupt
 */
void UART1_IRQHandler(void)
{
	uint8_t c;
	 //check data available
    if ((USART1->SR & USART_FLAG_RXNE) != (uint16_t)RESET) {
    	// clear the flag
    	__HAL_USART_CLEAR_FLAG(&UART_Handler, USART_IT_RXNE);
    	c = USART1->DR & 0xFF;		/* don't need no HAL */

    	//logic
    	// if not \n or \r add to line buffer
    	//if \n or \r & line buffer not empty -> put line buffer on queue
    	//
      //
  	  //add to queue
    	if (c != '\n' && c != '\r') {
    		line_buffer[line_buffer_index] = c;
    		line_buffer_index++;
    	} else if (index != 0) {
    			xQueueSendToBackFromISR(Data_Queue, line_buffer, ( portTickType ) 4 );
    			// clear line buffer
    			memset(line_buffer, 0, 100);
    			line_buffer_index = 0;
    	}
    } else {	// cleanup other flags
    	HAL_UART_IRQHandler((UART_HandleTypeDef *)&UART_Handler);
    }
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
debug_printf("rx\n\r");
}


//############################ HELPER FUNCTIONS ###############################

void waitForPassed(){
  char rx_char = 0;
  while(!lastTaskPassed){
    vTaskDelay(1000);
  }

  lastTaskPassed = FALSE;
}

/* Resets the wifi module */
void Wifi_reset(){
  char command[20] = WIFI_CMD_RST;
  char rx_char = 0;
  int timeout = 0;

  debug_printf("Reseting module... Please wait\n");

  HAL_UART_Transmit(&UART_Handler, &(command[0]), WIFI_LEN_RST, 10);

  waitForPassed();
  waitForPassed(); //We have to wait for OK and then ready

  debug_printf("Success.\n\n");
}

/* Joins my home network */
void Wifi_join(){
  char command[50] = WIFI_CMD_JOIN_TIMMY_HOME;

  debug_printf("Joining network\n");

  HAL_UART_Transmit(&UART_Handler, &(command[0]), WIFI_LEN_JOIN_TIMMY_HOME, 10);

  waitForPassed();

  debug_printf("Success.\n\n");
}

/* Currently sets mode to 3 -Both AP and ST) */
void Wifi_setmode(){
  char command[50] = WIFI_CMD_MODE_BOTH;

  debug_printf("Setting module mode\n");

  HAL_UART_Transmit(&UART_Handler, &(command[0]), WIFI_LEN_MODE_BOTH, 10);

  waitForPassed();

  debug_printf("Success.\n\n");
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

// void Wifi_getip(){
//   char command[50] = WIFI_CMD_GET_IP;
//   HAL_UART_Transmit(&UART_Handler, &(command[0]), WIFI_LEN_GET_IP, 10);
// }

/*
 * Enables a TCP server on port 8888
 */
void Wifi_enserver(){
  char command[50] = WIFI_CMD_MUX_1;

  debug_printf("Enabling a server on 8888\n");

  HAL_UART_Transmit(&UART_Handler, &(command[0]), WIFI_LEN_MUX_1, 10);

  waitForPassed();

  memcpy(&(command[0]), WIFI_CMD_SERVE, WIFI_LEN_SERVE);
  HAL_UART_Transmit(&UART_Handler, &(command[0]), WIFI_LEN_SERVE, 10);

  waitForPassed();

  debug_printf("Success\n\n");
}

// @deprectaed
void Delay(int x){
  while(x--);
  debug_printf("Deprecated Delay used.. Use vTaskDelay( ms )\n");
}
