/**
  ******************************************************************************
  * @file    ex3_gpio_interrupt.c 
  * @author  MDS
  * @date    02022015
  * @brief   Enable external interrupt on pin D0.
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
volatile int press_count;

/* Private function prototypes -----------------------------------------------*/
void Hardware_init();
void exti_d0_irqhandler(void);


/**
  * @brief  Main program - enable D0 external interrupt.
  * @param  None
  * @retval None
  */
void main(void) {
	
	BRD_init();			//Initalise NP2 board
	Hardware_init();	//Initalise hardware modules
  	
	/* Main processing loop waiting for interrupt */
    while (1) {
  	}

}

/**
  * @brief  Initialise Hardware Peripherals used.
  * @param  None
  * @retval None
  */
void Hardware_init() {

	GPIO_InitTypeDef  GPIO_InitStructure;
  	
	BRD_LEDInit();		//Initialise Blue LED
	BRD_LEDOff();		//Turn off Blue LED

  	/* Enable DO clock */
  	__BRD_D0_GPIO_CLK();

	/* Set priority of external GPIO Interrupt [0 (HIGH priority) to 15(LOW priority)] */
	/* 	DO NOT SET INTERRUPT PRIORITY HIGHER THAN 3 */
	HAL_NVIC_SetPriority(BRD_D0_EXTI_IRQ, 10, 0);	//Set Main priority ot 10 and sub-priority ot 0

	//Enable external GPIO interrupt and interrupt vector for pin DO
	NVIC_SetVector(BRD_D0_EXTI_IRQ, (uint32_t)&exti_d0_irqhandler);  
	NVIC_EnableIRQ(BRD_D0_EXTI_IRQ);

  	/* Configure D0 pin as pull down input */
	GPIO_InitStructure.Pin = BRD_D0_PIN;				//Pin
  	GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;		//interrupt Mode
  	GPIO_InitStructure.Pull = GPIO_PULLUP;			//Enable Pull up, down or no pull resister
  	GPIO_InitStructure.Speed = GPIO_SPEED_MEDIUM;			//Pin latency
  	HAL_GPIO_Init(BRD_D0_GPIO_PORT, &GPIO_InitStructure);	//Initialise Pin
}


/**
  * @brief  NP2_D0_EXTI Interrupt handler - see netduinoplus2.h
  * @param  None.
  * @retval None
  */
void exti_d0_irqhandler(void) {
	
	HAL_GPIO_EXTI_IRQHandler(BRD_D0_PIN);				//Clear D0 pin external interrupt flag

    BRD_LEDToggle();
	debug_printf("Triggered - %d\n\r", press_count);    //Print press count value

	press_count++;		//increment press count, everytime the interrupt occurs

}

