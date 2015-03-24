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

#define SIMULATE

UART_HandleTypeDef UART_Handler;
QueueHandle_t Data_Queue;	/* Queue used */

volatile int lastTaskPassed = FALSE;


char test_message[100] = "+IPD=Hellakshdfijhalsjdhfa\n";
char ok[5] = "OK";


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

  /* Configure settings for USART 6 */
<<<<<<< HEAD
//  UART_Handler.Instance = (USART_TypeDef *)USART1_BASE;		//USART 1
//  UART_Handler.Init.BaudRate   = 9600;	             			//Baudrate
//  UART_Handler.Init.WordLength = UART_WORDLENGTH_8B;    	//8 bits data length
//  UART_Handler.Init.StopBits   = UART_STOPBITS_1;	      	//1 stop bit
//  UART_Handler.Init.Parity     = UART_PARITY_NONE;    		//No paraity
//  UART_Handler.Init.Mode = UART_MODE_TX_RX;		           	//Set for Transmit and Receive mode
//  UART_Handler.Init.HwFlowCtl = UART_HWCONTROL_NONE;	   	//Set HW Flow control to none.
//
=======
  UART_Handler.Instance = (USART_TypeDef *)USART1_BASE;		//USART 1
  UART_Handler.Init.BaudRate   = 9600;	             			//Baudrate
  UART_Handler.Init.WordLength = UART_WORDLENGTH_8B;    	//8 bits data length
  UART_Handler.Init.StopBits   = UART_STOPBITS_1;	      	//1 stop bit
  UART_Handler.Init.Parity     = UART_PARITY_NONE;    		//No paraity
  UART_Handler.Init.Mode = USART_MODE_TX_RX;		           	//Set for Transmit and Receive mode
  UART_Handler.Init.HwFlowCtl = UART_HWCONTROL_NONE;	   	//Set HW Flow control to none.

>>>>>>> c2736b622255b4d77bc02cf0b4d9669d06e79ae7
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

<<<<<<< HEAD

  UART_Handler.Instance          = USART1;

  UART_Handler.Init.BaudRate     = 9600;
  UART_Handler.Init.WordLength   = UART_WORDLENGTH_8B;
  UART_Handler.Init.StopBits     = UART_STOPBITS_1;
  UART_Handler.Init.Parity       = UART_PARITY_NONE;
  UART_Handler.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  UART_Handler.Init.Mode         = UART_MODE_TX_RX;
  UART_Handler.Init.OverSampling = UART_OVERSAMPLING_16;


  HAL_NVIC_SetPriority(USART1_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(USART1_IRQn);


  if(HAL_UART_Init(&UART_Handler) != HAL_OK)
  {
    /* Initialization Error */
    debug_printf("UART initialisation FAIL");
  } else {
	  debug_printf("UART initialisation PASS");
  }




















  /*  Enable RXNE interrupt on USART_1 */

  // enable this to go to infinite loop, ref: esp8266.c; line 96  - 2nd time receive -> task_exit_critical
  //__HAL_UART_ENABLE_IT(&UART_Handler, USART_IT_RXNE);
  if (HAL_UART_Receive_IT((UART_HandleTypeDef*)&UART_Handler, (uint8_t *)uart_buffer, 100) != HAL_OK) {
	  debug_printf("UART Interrupt init FAIL");
  }
  /* Enable the UART Parity Error Interrupt */
  //__HAL_UART_ENABLE_IT(&UART_Handler, UART_IT_PE);

      /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
  //__HAL_UART_ENABLE_IT(&UART_Handler, UART_IT_ERR);


=======
>>>>>>> c2736b622255b4d77bc02cf0b4d9669d06e79ae7
  /* Initialise USART */
  //HAL_UART_Init(&UART_Handler);


  /* configure Nested Vectored Interrupt Controller for USART_1 */
<<<<<<< HEAD
  //HAL_NVIC_SetPriority(USART1_IRQn, 10, 0);
  //NVIC_SetVector(USART1_IRQn, (UART_HandleTypeDef *)&UART_Handler);
  //NVIC_EnableIRQ(USART1_IRQn);

=======
  HAL_NVIC_SetPriority(USART1_IRQn, 10, 1);
  NVIC_SetVector(USART1_IRQn, (uint32_t)&USART1_IRQHandler);
  NVIC_EnableIRQ(USART1_IRQn);
>>>>>>> c2736b622255b4d77bc02cf0b4d9669d06e79ae7

  /*  Enable RXNE interrupt on USART_1 */
  // enable this to go to infinite loop, ref: esp8266.c; line 96  - 2nd time receive -> task_exit_critical
  __HAL_USART_ENABLE_IT(&UART_Handler, USART_IT_RXNE);

  //xTaskCreate( (void *) &UART_Processor, (const signed char *) "DATA", mainLED_TASK_STACK_SIZE * 5, NULL, mainLED_PRIORITY + 1, NULL );

  Data_Queue = xQueueCreate(5, sizeof(char[100]));

  debug_printf("Wifi init success\n");
}

/*
 * Task for processing UART data once it has been added to Data_Queue by the
 * interupt RX handler.
 */
void UART_Processor( void ){
  char new_data[100];

#ifdef SIMULATE
    //Adds some test data to the Queue
  xQueueSendToBack(Data_Queue, ( void * ) &(test_message[0]), ( portTickType ) 10 );
  xQueueSendToBack(Data_Queue, ( void * ) &(ok[0]), ( portTickType ) 10 );
#endif

  for(;;){
      if(xQueueReceive(Data_Queue, &new_data, 10)){
        //We have new data analyze it
        if(strncmp(&(new_data[0]), "+IPD", 4) == 0){
          debug_printf("Message from node: %s\n", &(new_data[5]));
        } else if(strncmp(&(new_data[0]), "OK", 2) == 0){
          //Set the last task passed flag
          lastTaskPassed = TRUE;
        } else {
            debug_printf("New data found: %s\n", new_data);
        }
      }
      vTaskDelay(1000);
  }
}

/*
 * Usart_1 interrupt
 */

void UART1_IRQHandler(void)
{
<<<<<<< HEAD
	//uint8_t c;
	// check data available
   // if ((USART1->SR & USART_FLAG_RXNE) != (uint16_t)RESET) {
    	// clear the flag
    	//__HAL_USART_CLEAR_FLAG(&UART_Handler, USART_IT_RXNE);
    	//c = USART1->DR & 0xFF;		/* don't need no HAL */


    	// add to queue

    //}
	HAL_UART_IRQHandler((UART_HandleTypeDef *)&UART_Handler);
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
debug_printf("rx\n\r");
=======
	uint8_t c;
	//check data available
<<<<<<< HEAD
   if ((USART1->SR & USART_FLAG_RXNE) != (uint16_t)RESET) {
    	//clear the flag
    	__HAL_USART_CLEAR_FLAG(&UART_Handler, USART_IT_RXNE);
    	c = USART1->DR & 0xFF;		/* don't need no HAL */

      //TODO add to queue
    }
>>>>>>> dfc7e10236e8bdc42bc03ef5096d8662e257d009
=======
  if ((USART1->SR & USART_FLAG_RXNE) != (uint16_t)RESET) {
    __HAL_USART_CLEAR_FLAG(&UART_Handler, USART_IT_RXNE);
  	c = USART1->DR & 0xFF;		/* don't need no HAL */
    debug_printf("Data: %c\n", c);
    //TODO add to queue
  }
>>>>>>> c2736b622255b4d77bc02cf0b4d9669d06e79ae7
}


//############################ HELPER FUNCTIONS ###############################

/* Waits for "OK" to be replied from the Wifi module
 * @warning This function is the most blocking */
void waitForPassed(){
  char rx_char = 0;
  while(!lastTaskPassed){
    vTaskDelay(1000);
  }
  lastTaskPassed = FALSE;
  debug_printf("Success.\n\n");
}

/* Resets the wifi module */
void Wifi_reset(){
  char command[20] = WIFI_CMD_RST;
  char rx_char = 0;
  int timeout = 0;

  debug_printf("Reseting module... Please wait\n");

  HAL_UART_Transmit(&UART_Handler, &(command[0]), WIFI_LEN_RST, 10);

  waitForPassed();
}

/* Joins my home network */
void Wifi_join(){
  char command[50] = WIFI_CMD_JOIN_TIMMY_HOME;

  debug_printf("Joining network\n");

  HAL_UART_Transmit(&UART_Handler, &(command[0]), WIFI_LEN_JOIN_TIMMY_HOME, 10);

  waitForPassed();
}

/* Currently sets mode to 3 -Both AP and ST) */
void Wifi_setmode(){
  char command[50] = WIFI_CMD_MODE_BOTH;

  debug_printf("Setting module mode\n");

  HAL_UART_Transmit(&UART_Handler, &(command[0]), WIFI_LEN_MODE_BOTH, 10);

  waitForPassed();
}

/* Lists the AP names in return type
 * PROBLEMS
 * ===============
 * - For some reason the uart receive code wont work. Just gives 'A' once
 * @unfinsihed
 */
void Wifi_listAPs(){
  char command[50] = WIFI_CMD_LIST_APS;

  debug_printf("Getting AP Names\n");

  HAL_UART_Transmit(&UART_Handler, &(command[0]), WIFI_LEN_LIST_APS, 10);

  waitForPassed();
}

/* Sends the status command
 * @unfinished
 */
void Wifi_status(){
  char command[50] = WIFI_CMD_STATUS;

  HAL_UART_Transmit(&UART_Handler, &(command[0]), WIFI_LEN_STATUS, 10);

  waitForPassed();
}

/* Sets the wifi ap
 * @unfinsihed
 */
void Wifi_setAP(){
  char command[50] = WIFI_CMD_SET_AP;

  HAL_UART_Transmit(&UART_Handler, &(command[0]), WIFI_LEN_SET_AP, 10);

  waitForPassed();
}

/* Checks the IP address
 * @unfinished
 * @broken
 */
void Wifi_checkcon(){
  char command[50] = "AT+CWJAP\n\r";

  HAL_UART_Transmit(&UART_Handler, &(command[0]), 12, 10);

  waitForPassed();
}

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
}

/* @deprectaed
 * @replaced vTaskDelay( ms )*/
void Delay(int x){
  while(x--);
  debug_printf("WARNING: Deprecated Delay used.. Use vTaskDelay( ms )\n");
}
