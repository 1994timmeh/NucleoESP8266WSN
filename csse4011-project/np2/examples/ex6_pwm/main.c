/**
  ******************************************************************************
  * @file    ex6_pwm/main.c 
  * @author  MDS
  * @date    02022015
  * @brief   Enable the PWM output on pin DO.
  *			 See Section 18 (TIM3), P576 of the STM32F4xx Reference Manual.
  ******************************************************************************
  *  
  */ 

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "stm32f4xx_hal_conf.h"

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

	BRD_init();	//Initalise NP2
	Hardware_init();	//Initalise hardware modules

  	while (1) {

		BRD_LEDToggle();	//Toggle 'Alive' LED on/off
    	Delay(0x7FFF00);	//Delay function
	}
}

/**
  * @brief  Configure the hardware, 
  * @param  None
  * @retval None
  */
void Hardware_init(void) {

	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_OC_InitTypeDef PWMConfig;
	TIM_HandleTypeDef TIM_Init;

	uint16_t PrescalerValue = 0;

	BRD_LEDInit();		//Initialise Blue LED
	BRD_LEDOff();		//Turn off Blue LED

  	/* Timer 3 clock enable */
  	__TIM3_CLK_ENABLE();

  	/* Enable the D0 Clock */
  	__BRD_D0_GPIO_CLK();

  	/* Configure the D0 pin with TIM3 output*/
	GPIO_InitStructure.Pin = BRD_D0_PIN;				//Pin
  	GPIO_InitStructure.Mode =GPIO_MODE_AF_PP; 		//Set mode to be output alternate
  	GPIO_InitStructure.Pull = GPIO_NOPULL;			//Enable Pull up, down or no pull resister
  	GPIO_InitStructure.Speed = GPIO_SPEED_MEDIUM;			//Pin latency
	GPIO_InitStructure.Alternate = GPIO_AF2_TIM3;	//Set alternate function to be timer 3
  	HAL_GPIO_Init(BRD_D0_GPIO_PORT, &GPIO_InitStructure);	//Initialise Pin

	/* Compute the prescaler value. SystemCoreClock = 168000000 - set for 50Khz clock */
  	PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 50000) - 1;

	/* Configure Timer settings */
	TIM_Init.Instance = TIM3;					//Enable Timer 3
  	TIM_Init.Init.Period = 2*50000/10;			//Set for 200ms (5Hz) period
  	TIM_Init.Init.Prescaler = PrescalerValue;	//Set presale value
  	TIM_Init.Init.ClockDivision = 0;			//Set clock division
	TIM_Init.Init.RepetitionCounter = 0; 		// Set Reload Value
  	TIM_Init.Init.CounterMode = TIM_COUNTERMODE_UP;	//Set timer to count up.
	
	/* PWM Mode configuration for Channel 2 - set pulse width*/
	PWMConfig.OCMode       = TIM_OCMODE_PWM1;	//Set PWM MODE (1 or 2 - NOT CHANNEL)
    PWMConfig.Pulse        = 2*50000/100;		//1ms pulse width to 10ms
    PWMConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;
    PWMConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    PWMConfig.OCFastMode   = TIM_OCFAST_DISABLE;
    PWMConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;
    PWMConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;

	/* Enable PWM for Timer 3, channel 2 */
	HAL_TIM_PWM_Init(&TIM_Init);	
	HAL_TIM_PWM_ConfigChannel(&TIM_Init, &PWMConfig, TIM_CHANNEL_2);	

	/* Start PWM */
	HAL_TIM_PWM_Start(&TIM_Init, TIM_CHANNEL_2);

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


