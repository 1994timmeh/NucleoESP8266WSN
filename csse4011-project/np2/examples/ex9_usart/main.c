/**
  ******************************************************************************
  * @file    ex6_usart/main.c 
  * @author  MDS
  * @date    02022015
  * @brief   USART 6 polling example program - transmits '0' to '9' via USART 6.
  *			 Display received characters from USART 6.
  *			 NOTE: This uses the HAL UART API, not the HAL USART API.
  ******************************************************************************
  *  
  */ 

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "debug_printf.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef UART_test;

/* Private function prototypes -----------------------------------------------*/
void Delay(__IO unsigned long nCount);
void Hardware_init();


/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
void main(void) {
	
	char tx_char;
	char rx_char;
	int tx_count = 0;
	
	BRD_init();	//Initalise NP2
	Hardware_init();	//Initalise hardware modules

	/* Main processing loop */
    while (1) {
  
		tx_char = '0' + tx_count;			//Send characters '0' to '9' in ASCII
		
		//Transmit character
		if (HAL_UART_Transmit(&UART_test, &tx_char, 1, 10) != HAL_OK) {
			debug_printf("Transmit ERROR\n\r");
		}
		
		tx_count = (tx_count +1)%10;		//Only send characters '0' to '9'.
		
		//Check for received characters
		if (HAL_UART_Receive(&UART_test, &rx_char, 1, 20) == HAL_OK) {
			debug_printf("RX %c\n\r", rx_char);
		}

    	BRD_LEDToggle();	//Toggle 'Alive' LED on/off
    	Delay(0x7FFF00);	//Delay function
  	}
}

/**
  * @brief  Initialise Hardware 
  * @param  None
  * @retval None
  */
void Hardware_init() {

	GPIO_InitTypeDef GPIO_serial;	

	BRD_LEDInit();		//Initialise Blue LED
	BRD_LEDOff();		//Turn off Blue LED

	/* Enable USART6 Clock */
	__USART6_CLK_ENABLE();

	/* Enable the D0 & D1 Clock */
  	__BRD_D0_GPIO_CLK();
	__BRD_D1_GPIO_CLK();

	/* Configure settings for USART 6 */
	UART_test.Instance = (USART_TypeDef *)USART6_BASE;		//USART 6
    UART_test.Init.BaudRate   = 9600;					//Baudrate
    UART_test.Init.WordLength = UART_WORDLENGTH_8B;	//8 bits data length
    UART_test.Init.StopBits   = UART_STOPBITS_1;		//1 stop bit
    UART_test.Init.Parity     = UART_PARITY_NONE;		//No paraity
	UART_test.Init.Mode = UART_MODE_TX_RX;			//Set for Transmit and Receive mode
	UART_test.Init.HwFlowCtl = UART_HWCONTROL_NONE;		//Set HW Flow control to none.

  	/* Configure the D0 as the RX pin for USART6 */ 
  	GPIO_serial.Pin = BRD_D0_PIN;
  	GPIO_serial.Mode = GPIO_MODE_AF_PP;					//Enable alternate mode setting
  	GPIO_serial.Pull = GPIO_PULLDOWN;
  	GPIO_serial.Speed = GPIO_SPEED_HIGH;
	GPIO_serial.Alternate = GPIO_AF8_USART6;			//Set alternate setting to USART 6
  	HAL_GPIO_Init(BRD_D0_GPIO_PORT, &GPIO_serial);

	/* Configure the D1 as the tX pin for USART6 */ 
	GPIO_serial.Pin = BRD_D1_PIN;				
  	GPIO_serial.Mode = GPIO_MODE_AF_PP;					//Enable alternate mode setting
  	GPIO_serial.Pull = GPIO_PULLUP;
  	GPIO_serial.Speed = GPIO_SPEED_HIGH;
	GPIO_serial.Alternate = GPIO_AF8_USART6;			//Set alternate setting to USART 6
  	HAL_GPIO_Init(BRD_D1_GPIO_PORT, &GPIO_serial);

	/* Initialise USART */
	HAL_UART_Init(&UART_test);
}

/**
  * @brief  Delay Function.
  * @param  nCount:specifies the Delay time length.
  * @retval None
  */
void Delay(__IO unsigned long nCount) {
	while(nCount--) {
  	}
}


