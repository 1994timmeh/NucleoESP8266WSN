/**
  ******************************************************************************
  * @file    ex1_led/main.c 
  * @author  MDS
  * @date    05032015
  * @brief   NP2 onboard blue LED flashing example.
  ******************************************************************************
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
void Hardware_init(void);
void Delay(__IO unsigned long nCount);


/**
  * @brief  Main program - flashes onboard LED
  * @param  None
  * @retval None
  */
void main(void)  {

	int i;
	char rxchar;

	BRD_init();			//Initalise Board
	Hardware_init();	//Initalise hardware modules

	i = 0;  	

	/* Main processing loop */
    while (1) {

		debug_printf("LED Toggle %d\n\r", i);	//Print debug message
		
		i++;    			//Increment counter
		BRD_LEDToggle();	//Toggle LED on/off
    	Delay(0x7FFF00);	//Delay 1s.

	}

}

/**
  * @brief  Initialise Hardware 
  * @param  None
  * @retval None
  */
void Hardware_init(void) {

	BRD_LEDInit();		//Initialise Blue LED
	BRD_LEDOff();		//Turn off Blue LED
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