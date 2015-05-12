/**
  ******************************************************************************
  * @file    ex5_timer_interrupt.c 
  * @author  MDS
  * @date    02022015
  * @brief   Enable Timer 2 update compare interrupt. Use interrupt to flash
  *			 LED every second.
  *			 See Section 18 (TIM2), P576 of the STM32F4xx Reference Manual.
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
TIM_HandleTypeDef TIM_Init;
volatile int count_interrupt;	//increment each time a timer interrupt occurs

/* Private function prototypes -----------------------------------------------*/
void Hardware_init(void);
void tim2_irqhandler (void);


/**
  * @brief  Main program - Timer 2 update compare interrupt.
  * @param  None
  * @retval None
  */
void main(void) {

	BRD_init();	//Initalise NP2
	Hardware_init();	//Initalise hardware modules

  	/* Main processing loop waiting for interrupt */
  	while (1) {
	}
}

/**
  * @brief  Configure the hardware, 
  * @param  None
  * @retval None
  */
void Hardware_init(void) {

	unsigned short PrescalerValue;

	BRD_LEDInit();		//Initialise onboard LED
	BRD_LEDOff();		//Turn off onboard LED
  
  	/* Timer 2 clock enable */
	__TIM2_CLK_ENABLE();

	/* Compute the prescaler value */
  	PrescalerValue = (uint16_t) ((SystemCoreClock /2)/50000) - 1;		//Set clock prescaler to 50kHz - SystemCoreClock is the system clock frequency.

  	/* Time base configuration */
	TIM_Init.Instance = TIM2;				//Enable Timer 2
  	TIM_Init.Init.Period = 50000/1000;			//Set period count to be 1ms, so timer interrupt occurs every 1ms.
  	TIM_Init.Init.Prescaler = PrescalerValue;	//Set presale value
  	TIM_Init.Init.ClockDivision = 0;			//Set clock division
	TIM_Init.Init.RepetitionCounter = 0;	// Set Reload Value
  	TIM_Init.Init.CounterMode = TIM_COUNTERMODE_UP;	//Set timer to count up.

	/* Initialise Timer */
	HAL_TIM_Base_Init(&TIM_Init);

	/* Set priority of Timer 2 update Interrupt [0 (HIGH priority) to 15(LOW priority)] */
	/* 	DO NOT SET INTERRUPT PRIORITY HIGHER THAN 3 */
	HAL_NVIC_SetPriority(TIM2_IRQn, 10, 0);		//Set Main priority ot 10 and sub-priority ot 0.

	/* Enable timer update interrupt and interrupt vector for Timer  */
	NVIC_SetVector(TIM2_IRQn, (uint32_t)&tim2_irqhandler);  
	NVIC_EnableIRQ(TIM2_IRQn);

	/* Start Timer */
	HAL_TIM_Base_Start_IT(&TIM_Init);
}

/**
  * @brief  Timer 2 Interrupt handler
  * @param  None.
  * @retval None
  */
void tim2_irqhandler (void) {

	//Clear Update Flag
	__HAL_TIM_CLEAR_IT(&TIM_Init, TIM_IT_UPDATE);

	
	//Toggle LED every second (timer interrupt should occur every 1ms)
	if (count_interrupt > 1000) {

		/* Toggle LED */		
		BRD_LEDToggle();
		count_interrupt = 0;
	}
		
	count_interrupt++;		//increment counter, when the interrupt occurs
	

}


