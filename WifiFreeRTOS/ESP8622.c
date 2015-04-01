#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "debug_printf.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "ESP8622.h"
#include <string.h>
#include <stdio.h>

#define TRUE 1
#define FALSE 0

UART_HandleTypeDef UART_Handler;
QueueHandle_t Data_Queue;	/* Queue used */

volatile int lastTaskPassed = FALSE;
volatile int prompt = FALSE;


char test_message[100] = "+IPD=Hellakshdfijhalsjdhfa\n";
char ok[5] = "OK";
char line_buffer[100];
uint8_t line_buffer_index = 0;

char ip_addr_string[20];
char uart_buffer[100];

extern uint32_t time;

/*				wifi AP structs */


// wifi struct

// typedef struct Access_Point {
// 	int channel;
// 	char[20] ESSID;
// 	char[30] SSID;
// 	int RSSI;	// RSSI as negative
// 	Access_Point* next;
// 	Access_Point* prev;
// }   Access_Point;
//
// typedef struct Access_Points {
// 	Access_Point* AP;
// 	Access_Point* HEAD;
// 	Access_Point* TAIL;
// 	uint16_t size;
// } APs;
//
// APs Access_Points;
// Access_Points->size = 0;
//



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
  __BRD_D3_GPIO_CLK();
  __BRD_D4_GPIO_CLK();
  __BRD_D5_GPIO_CLK();
  __BRD_D6_GPIO_CLK();

  GPIO_serial.Pin = BRD_D3_PIN;
  GPIO_serial.Mode = GPIO_MODE_OUTPUT_PP;				             	//Enable alternate mode setting
  GPIO_serial.Pull = GPIO_PULLDOWN;
  GPIO_serial.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(BRD_D3_GPIO_PORT, &GPIO_serial);

  GPIO_serial.Pin = BRD_D4_PIN;
  HAL_GPIO_Init(BRD_D4_GPIO_PORT, &GPIO_serial);

  GPIO_serial.Pin = BRD_D5_PIN;
  HAL_GPIO_Init(BRD_D5_GPIO_PORT, &GPIO_serial);

  GPIO_serial.Pin = BRD_D6_PIN;
  HAL_GPIO_Init(BRD_D6_GPIO_PORT, &GPIO_serial);

  GPIO_serial.Pin = BRD_D7_PIN;
  HAL_GPIO_Init(BRD_D7_GPIO_PORT, &GPIO_serial);

  GPIO_serial.Pin = BRD_D8_PIN;
  HAL_GPIO_Init(BRD_D8_GPIO_PORT, &GPIO_serial);

  /* Configure the D2 as the RX pin for USART1 */
  GPIO_serial.Pin = BRD_D2_PIN;
  GPIO_serial.Mode = GPIO_MODE_AF_PP;				             	//Enable alternate mode setting
  GPIO_serial.Pull = GPIO_PULLDOWN;
  GPIO_serial.Speed = GPIO_SPEED_HIGH;
  GPIO_serial.Alternate = GPIO_AF7_USART1;	           		//Set alternate setting to USART1
  HAL_GPIO_Init(BRD_D2_GPIO_PORT, &GPIO_serial);

  /* Configure the D10 as the TX pin for USART1 */
  GPIO_serial.Pin = BRD_D10_PIN;
  GPIO_serial.Mode = GPIO_MODE_AF_PP;				             	//Enable alternate mode setting
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
  if (HAL_UART_Receive_IT((UART_HandleTypeDef*)&UART_Handler, (uint8_t *)uart_buffer, 100) != HAL_OK) {
	  debug_printf("UART Interrupt init FAIL");
  }

  xTaskCreate( (void *) &UART_Processor, (const signed char *) "DATA", mainLED_TASK_STACK_SIZE * 5, NULL, mainLED_PRIORITY + 1, NULL );
  Data_Queue = xQueueCreate(10, sizeof(char[100]));
}

/*
 * Task for processing UART data and adding new data to a data queue
 */
void UART_Processor( void ){
  char new_data[100];

  for(;;){
      if(xQueueReceive(Data_Queue, &new_data, 10) && new_data[0] != '\r'){
        debug_printf("LINE RX: %s\n", new_data);
        //We have new data analyze it
        if(strncmp(&(new_data[0]), "+IPD", 4) == 0){
          BRD_LEDToggle();
        	debug_printf("1: %s\n", new_data);
          debug_printf("Data: %s\n", &(new_data[5]));
          handle_data(new_data+5);

        } else if(strncmp(&(new_data[0]), "OK", 2) == 0 || strncmp(&(new_data[0]), "ready", 5) == 0
    || strncmp(&(new_data[0]), "no change", 9) == 0 || strncmp(&(new_data[0]), "SEND OK", 7) == 0) {
        	debug_printf("w: %s\n", new_data);
          //Set the last task passed flag
          lastTaskPassed = TRUE;

        } else if(new_data[0] == '>'){
          prompt = TRUE;

        } else if (strncmp(new_data, "192.168", 7) == 0) {
        	memset(ip_addr_string, 0, 20);
			    memcpy(ip_addr_string, new_data, 20);
			    debug_printf("IP Address: %s\n", ip_addr_string);

  	    } else if (strncmp(new_data, "+CWLAP:", 7) == 0){
  				handle_Access_Point(new_data);
  	    }
      }
      vTaskDelay(1);
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

void handle_Access_Point (char* apString) { //(0,"Visitor-UQconnect",-71,"00:25:45:a2:ea:92",6)
	 char zero;
	 char essid[30];
	 char rssi[3];
   int rssii;
	 char bssid[30];
	 char channel[5];

   //debug_printf("WiFi AP found: %s\n", apString);
	 sscanf(apString, "+CWLAP:(%c,\"%[^\"]\",-%[^,],\"%[^\"]\",%[^)])", zero, &essid, rssi, bssid, channel);
  //  debug_printf("essid: %s\n", essid);
	//  debug_printf("rssi: %s\n", rssi);
  //  debug_printf("bssid: %s\n", bssid);
  //  debug_printf("channel: %s\n", channel);

   rssii = atoi(rssi);

   //Testing the led thing
   if(strncmp(essid, "NUCLEOWSN", 9) == 0){
      debug_printf("RSSI: %d Distance: %f\n", rssii, RSSItoDistance(rssii));

      HAL_GPIO_WritePin(BRD_D3_GPIO_PORT, BRD_D3_PIN, rssii < -67.5);
      HAL_GPIO_WritePin(BRD_D4_GPIO_PORT, BRD_D4_PIN, rssii < -60);
      HAL_GPIO_WritePin(BRD_D5_GPIO_PORT, BRD_D5_PIN, rssii < -52.5);
      HAL_GPIO_WritePin(BRD_D6_GPIO_PORT, BRD_D6_PIN, rssii < -45);
      HAL_GPIO_WritePin(BRD_D7_GPIO_PORT, BRD_D7_PIN, rssii < -37.5);
      HAL_GPIO_WritePin(BRD_D8_GPIO_PORT, BRD_D8_PIN, rssii < -30);
   }
 }


void handle_data(char* data) {
  uint8_t pipe_no = 0, length = 0;
  char message[50];
  sscanf(data, "%d,%d%s", pipe_no, length, message);
  debug_printf("Received data! Pipe=%d, length=%d, message=%s\n", pipe_no, length, message);
}

//############################ HELPER FUNCTIONS ###############################


void waitForPassed(int timeout){
  while(!lastTaskPassed){
    vTaskDelay(1);
  }

  if(timeout == 0){
    debug_printf("Failure.\n\n");
  } else {
    debug_printf("Success.\n\n");
  }

  lastTaskPassed = FALSE;
}

void waitForPrompt(){
  while(!prompt){
    vTaskDelay(100);
  }
  prompt = FALSE;
}

/* Resets the wifi module */
void Wifi_reset(){
  char command[20] = WIFI_CMD_RST;

  debug_printf("Reseting module... Please wait\n");

  HAL_UART_Transmit(&UART_Handler, &(command[0]), WIFI_LEN_RST, 10);

  waitForPassed(5000);

  waitForPassed(5000);
}

/* Joins my home network */
void Wifi_join(char SSID[50], char password[50]){
  char command[50];
  int len = 0;
  len = sprintf(&(command[0]), WIFI_CMD_JOIN, SSID, password);

  debug_printf("Joining network\n");

  HAL_UART_Transmit(&UART_Handler, &(command[0]), len, 10);

  waitForPassed(5000);
}

/* Currently sets mode to 3 -Both AP and ST) */
void Wifi_setmode(){
  char command[50] = WIFI_CMD_MODE_BOTH;

  debug_printf("Setting module mode\n");

  HAL_UART_Transmit(&UART_Handler, &(command[0]), WIFI_LEN_MODE_BOTH, 10);

  waitForPassed(5000);
}

/* Lists the AP names in return type
 * PROBLEMS
 * ===============
 * - For some reason the uart receive code wont work. Just gives 'A' once
 * @unfinsihed
 */
void Wifi_listAPs(){
  char command[50] = WIFI_CMD_LIST_APS;

  HAL_UART_Transmit(&UART_Handler, &(command[0]), WIFI_LEN_LIST_APS, 10);

  HAL_GPIO_WritePin(BRD_D3_GPIO_PORT, BRD_D3_PIN, 0);
  HAL_GPIO_WritePin(BRD_D4_GPIO_PORT, BRD_D4_PIN, 0);
  HAL_GPIO_WritePin(BRD_D5_GPIO_PORT, BRD_D5_PIN, 0);
  HAL_GPIO_WritePin(BRD_D6_GPIO_PORT, BRD_D6_PIN, 0);
  HAL_GPIO_WritePin(BRD_D7_GPIO_PORT, BRD_D7_PIN, 0);
  HAL_GPIO_WritePin(BRD_D8_GPIO_PORT, BRD_D8_PIN, 0);

  debug_printf("Getting AP Names\n");

  waitForPassed(5000);
}

/* Sends the status command
 * @unfinished
 */
void Wifi_status(){
  char command[50] = WIFI_CMD_STATUS;
  HAL_UART_Transmit(&UART_Handler, &(command[0]), WIFI_LEN_STATUS, 10);
}

/* Sets the wifi ap
 * @param sec 0 for no password
 */
void Wifi_setAP(char SSID[50], char password[50], uint8_t chan, uint8_t sec){
  char command[50];
  int len;

  debug_printf("Setting AP details (probably crashing the wifi)\n");

  len = sprintf(&(command[0]), WIFI_CMD_SET_AP, SSID, password, chan, sec);
  HAL_UART_Transmit(&UART_Handler, &(command[0]), len, 10);

  waitForPassed(5000);
}

/* Checks the IP address */
void Wifi_checkcon(){
  char command[50] = "AT+CWJAP\n\r";
  HAL_UART_Transmit(&UART_Handler, &(command[0]), 12, 10);
}

void Wifi_get_station_IP(){
  char command[50] = WIFI_CMD_GET_IP_STA;
  HAL_UART_Transmit(&UART_Handler, &(command[0]), WIFI_LEN_GET_IP_STA, 10);

  waitForPassed(5000);
}

void Wifi_get_AP_IP(){
  char command[50] = WIFI_CMD_GET_IP_AP;
  HAL_UART_Transmit(&UART_Handler, &(command[0]), WIFI_LEN_GET_IP_AP, 10);

  waitForPassed(5000);
}

void Wifi_set_station_IP(char* IP_Addr){
	int len;
	char command[50];

	len = sprintf(command, WIFI_CMD_SET_IP_STA, IP_Addr);
	HAL_UART_Transmit(&UART_Handler, command, len, 10);

  waitForPassed(5000);
}

void Wifi_set_AP_IP(char* IP_Addr){
	int len;
	char command[50];

	len = sprintf(command, WIFI_CMD_SET_IP_AP, IP_Addr);
	HAL_UART_Transmit(&UART_Handler, command, len, 10);

  waitForPassed(5000);
}

/*
 * Enables a TCP server on port 8888
 */
void Wifi_enserver(){
  char command[50] = WIFI_CMD_MUX_1;

  debug_printf("Enabling a server on 8888\n");

  //Set MUX to 1
  HAL_UART_Transmit(&UART_Handler, &(command[0]), WIFI_LEN_MUX_1, 10);

  waitForPassed(5000);

  //Enable the TCP server on 8888
  memcpy(&(command[0]), WIFI_CMD_SERVE, WIFI_LEN_SERVE);
  HAL_UART_Transmit(&UART_Handler, &(command[0]), WIFI_LEN_SERVE, 10);

  waitForPassed(5000);
}

void Wifi_synctime(){
  char packet[20];
  debug_printf("Current time is %d\n", time);

  Wifi_join("NUCLEOWSN2", "");
  waitForPassed(5000);


  sprintf(&(packet[0]), "+TIME:%d", time);

}

void Wifi_connecttest(){
  HAL_UART_Transmit(&UART_Handler, "AT+CIPSTART=0,\"TCP\",\"192.168.4.1\",8888\r\n", 40, 10);
  waitForPassed(5000);
}

void Wifi_checkfirmware(){
  HAL_UART_Transmit(&UART_Handler, "AT+GMR\r\n", 8, 10);

  waitForPassed(5000);
}

void Wifi_connectTCP( char ip[50], int port){
  char command[50];
  int len = sprintf(command, "AT+CIPSTART=0,\"TCP\",\"%s\",%d\r\n", ip, 8888);
  HAL_UART_Transmit(&UART_Handler, command, len, 10);

  waitForPassed(5000);
}

void Wifi_senddata(char data[50]){
  char command[50] = WIFI_CMD_SEND_DATA;
  char send_data[50];
  int len = sprintf(command, "%s\n\r", data);

  HAL_UART_Transmit(&UART_Handler, &(command[0]), WIFI_LEN_SEND_DATA, 10);

  vTaskDelay(500);

  HAL_UART_Transmit(&UART_Handler, send_data, len, 10);
  waitForPassed(5000);
}


/* Returns the distance in meters */
float RSSItoDistance(int rssi){
  return 0.0039*rssi*rssi - 0.0935*rssi + 0.6208;
}




/*      AP list helpers     */
void index_Add_AP(Access_Point* access_Point, uint8_t index) {
    uint8_t i = 0;
    if (index == 0) {
        // add to start
    } else {
        for ( i = 0; i < index+1; i++) {

        }
    }
}

// // AP list helpers
// void add_AP(Access_Point* access_Point) {
//     // search list for bssid
//     //if found, remove from list
//     //
//     uint8_t i = 0;
//     if (index == 0) {
//         // add to start
//     } else {
//         for ( i = 0; i < index+1; i++) {
//
//         }
//     }
// }
//
// void remove_AP(Access_Point* access_Point) {
//     Access_Point* current_AP = APs->HEAD;
//     for(int i = 0; i < APs->size; i++) {
//         if (strncmp(current_AP, access_Point, 20) == 0) {
//             if (i = 0) {
//                 // first entry, change head change next
//                 current_AP->NEXT->PREV = null;
//                 APs->HEAD = current_AP->NEXT;
//             }
//             if (i = APs->size-1){
//                 // last entry
//                 current_AP->PREV->NEXT = null;
//                 APs->TAIL = current_AP->PREV;
//             }
//             current_AP->PREV->NEXT = current_AP->NEXT;
//             current_AP->NEXT->PREV = current_AP->PREV;
//         }
//     }
// }
//
