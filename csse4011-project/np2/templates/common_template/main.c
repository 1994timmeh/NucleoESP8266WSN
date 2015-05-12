/**
  ******************************************************************************
  * @file    prac1/main.c 
  * @author  MY FIRST NAME + SURNAME
  * @date    10-January-2014
  * @brief   Prac 1 Template C main file - BCD up counter and press counter.
  *			 NOTE: THIS CODE IS PSEUDOCODE AND DOES NOT COMPILE. 
  *				   GUIDELINES ARE GIVEN AS COMMENTS.
  *			 REFERENCES: ex1_led, ex2_gpio, ex3_gpio, ex11_character
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "netduinoplus2.h"
#include "stm32f4xx_conf.h"
#include "debug_printf.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint16_t Counter_value  = 0;
uint16_t Press_counter_value = 0;

/* Private function prototypes -----------------------------------------------*/
void Hardware_init(void);
void Delay(__IO unsigned long nCount);

/**
  * @brief  Main program - up counter and press counter.
  * @param  None
  * @retval None
  */
void main(void) {

	NP2_boardinit();	//Initalise NP2
	Hardware_init();	//Initalise hardware modules
  	
	/* Main processing loop */
  	while (1) {

		Counter_value++;	//Increment counter

		/****************** Display counter. ***************/
		/* First, turn off each LED light bar segment
			write 0 to D0
			Write 0 to D1
			....
			Write 0 to D9
		*/ 

		
		/* Check if bit of counter_value is 1 or 0.
			
			if usCounter_value_bit0 is 1 do:
				write 1 to D0 pin
			else
				write 0 to D0 pin

			
			if usCounter_value_bit1 is 1 do:
				write 1 to D1 pin
			else
				write 0 to D1 pin

			......

			if usCounter_value_bit9 is 1 do:
				write 1 to D9 pin
			else
				write 0 to D0 pin

		*/


		/* Insert delay */
    	Delay(0x7FFF00);	//Delay 1s

	}
}

/**
  * @brief  Initialise Hardware 
  * @param  None
  * @retval None
  */
void Hardware_init(void) {

	GPIO_InitTypeDef  GPIO_InitStructure;	

	NP2_LEDInit();		//Initialise Blue LED
	NP2_LEDOff();		//Turn off Blue LED

	/* Enable the GPIO D0 Clock */
  	RCC_AHB1PeriphClockCmd(NP2_D0_GPIO_CLK, ENABLE);

  	/* Configure the GPIO_D0 pin */
  	GPIO_InitStructure.GPIO_Pin = NP2_D0_PIN;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(NP2_D0_GPIO_PORT, &GPIO_InitStructure);

	/* Configure the GPIO_D1 pin
	
	 	.... 

		Configure the GPIO_D9 pin */

	/* Configure A2 interrupt for Prac 1, Task 2 or 3 only */

}

/**
  * @brief  Delay Function.
  * @param  nCount:specifies the Delay time length.
  * @retval None
  */
void Delay(__IO unsigned long nCount) {
  
	/* Delay a specific amount before returning */
	while(nCount--)	{
  	}
}

/**
  * @brief  NP2_A2_EXTI Interrupt handler - see netduinoplus2.h
  * @param  None.
  * @retval None
  */
void NP2_A2_EXTI_IRQ_HANDLER(void) {


	/* Check if interrupt has occured - see ex3_gpio_interrupt */

	/* Convert counter BCD. */
	/* BCD value = 3rd_digit*100 + 2nd_digit*10 + 1st_digit */
	
	/* Extract 3rd digit digit 
	   if (usPress_counter_value/100 > 0) {
		 .... 
	   } 

	   Extract 2nd digit
	   .... 

	   Extract 1st digit 
	   .... 
	*/
}
