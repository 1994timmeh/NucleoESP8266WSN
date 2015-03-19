#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "debug_printf.h"

UART_HandleTypeDef UART_Handler;
char test_message[30] = "NUCLEO-F401RE TEST MESSAGE\n";

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

  HAL_UART_Transmit(&UART_Handler, &(test_message[0]), 25, 100);
}

/*
 * Sends a test sequence of UART data
 */
void ESP8622_send_test(){
  uint8_t i, tx_count;
  char tx_char;
  debug_printf("Sending data\n");
  if(HAL_UART_Transmit(&UART_Handler, &(test_message[0]), 25, 100) == HAL_BUSY)
    debug_printf("HAL WAS BUSY\n");

  if(HAL_UART_Transmit(&UART_Handler, &(test_message[0]), 25, 100) == HAL_TIMEOUT)
    debug_printf("HAL TIMED OUT\n");
  }


void connect( void );
void send(char* data, int len);
//
// tx_char = '0' + tx_count;			//Send characters '0' to '9' in ASCII
//
// //Transmit character
// if (HAL_UART_Transmit(&UART_test, &tx_char, 1, 10) != HAL_OK) {
//   debug_printf("Transmit ERROR\n\r");
// }
//
// tx_count = (tx_count +1)%10;		//Only send characters '0' to '9'.
//
// //Check for received characters
// if (HAL_UART_Receive(&UART_test, &rx_char, 1, 20) == HAL_OK) {
//   debug_printf("RX %c\n\r", rx_char);
// }
