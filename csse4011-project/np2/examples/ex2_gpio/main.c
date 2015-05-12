/**
  ******************************************************************************
  * @file    ex2_gpio.c 
  * @author  MDS
  * @date    02022015
  * @brief   Toggle D0 high/low and read D1 pin.
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
/* Private function prototypes -----------------------------------------------*/
void Delay(__IO unsigned long nCount);
void Hardware_init();


/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
void main(void) {

	int read_value;
	int write_value = 0;

	BRD_init();			//Initalise NP2 board.
	Hardware_init();	//Initalise hardware modules  	

	/* Main processing loop */
    while (1) {
		/* Toggle D0 high or low */
		write_value = ~write_value;
		HAL_GPIO_WritePin(BRD_D0_GPIO_PORT, BRD_D0_PIN, write_value & 0x01);	//Write Digital 0 bit value
	
		/* Read D1 pin */
		read_value = HAL_GPIO_ReadPin(BRD_D1_GPIO_PORT, BRD_D1_PIN);

		debug_printf("D1 value %d\n\r", read_value);	//Display D1 value

    	BRD_LEDToggle();	//Toggle LED on/off
    	Delay(0x7FFF00);	//Delay function
  	}
}

/**
  * @brief  Initialise Hardware 
  * @param  None
  * @retval None
  */
void Hardware_init() {

	GPIO_InitTypeDef  GPIO_InitStructure;

	BRD_LEDInit();		//Initialise Blue LED
	BRD_LEDOff();		//Turn off Blue LED

	/* Enable the D0 & D1 Clock */
  	__BRD_D0_GPIO_CLK();
	__BRD_D1_GPIO_CLK();

  	/* Configure the D0 pin as an output */
	GPIO_InitStructure.Pin = BRD_D0_PIN;				//Pin
  	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;		//Output Mode
  	GPIO_InitStructure.Pull = GPIO_PULLDOWN;			//Enable Pull up, down or no pull resister
  	GPIO_InitStructure.Speed = GPIO_SPEED_MEDIUM;			//Pin latency
  	HAL_GPIO_Init(BRD_D0_GPIO_PORT, &GPIO_InitStructure);	//Initialise Pin

	/* Configure the D1 pin as an input */
	GPIO_InitStructure.Pin = BRD_D1_PIN;				//Pin
  	GPIO_InitStructure.Mode = GPIO_MODE_INPUT;			//Input Mode
  	GPIO_InitStructure.Pull = GPIO_PULLDOWN;			//Enable Pull up, down or no pull resister
  	GPIO_InitStructure.Speed = GPIO_SPEED_MEDIUM;			//Pin latency
  	HAL_GPIO_Init(BRD_D1_GPIO_PORT, &GPIO_InitStructure);	//Initialise Pin

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


