/**
  ******************************************************************************
  * @file    ex4_adc.c 
  * @author  MDS
  * @date    10-January-2014
  * @brief   Enable the ADC1 on pin A0.
  *			 See Section 13 (ADC), P385 of the STM32F4xx Reference Manual.
  ******************************************************************************
  *  
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal_conf.h"
#include "debug_printf.h"
#include "board.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef AdcHandle1;
ADC_HandleTypeDef AdcHandle2;
ADC_ChannelConfTypeDef AdcChanConfig1;
ADC_ChannelConfTypeDef AdcChanConfig2;
TIM_HandleTypeDef TIM_Init;

int sampleNo = 0;
volatile int data1[1000];
volatile int data2[1000];

/* Private function prototypes -----------------------------------------------*/
void Delay(__IO unsigned long nCount);
void hardware_init();
void adc_switch_channel_0( void );
void adc_switch_channel_1( void );
void tim2_irqhandler( void );
void timer_interupt_init();


/**
  * @brief  Main program - enable ADC input on A0
  * @param  None
  * @retval None
  */
int main(void) {
	unsigned int adc_value;

	BRD_init();	//Initialise the NP2 board.
	hardware_init();	//Initialise Hardware peripherals
	timer_interupt_init();

	while (1) {
		int i = 0, ht = 0;
		for(i; i < 500; i++){
			debug_printf("%X %X\n", data1[i], data2[i]);
		}
	}
}


void timer_interupt_init(){
	unsigned short PrescalerValue;

	BRD_LEDInit();

	/* Timer 4 clock enable */
	__TIM4_CLK_ENABLE();

	/* Compute the prescaler value */
	PrescalerValue = (uint16_t) ((SystemCoreClock /2)/480000) - 1; //Set this to be 480kHz

	/* Time base configuration */
	TIM_Init.Instance = TIM4;						//Enable Timer 4
	TIM_Init.Init.Period = 20;						//Set period count to be 1ms, so timer interrupt occurs every 1ms.
	TIM_Init.Init.Prescaler = PrescalerValue;		//Set presale value
	TIM_Init.Init.ClockDivision = 0;				//Set clock division
	TIM_Init.Init.RepetitionCounter = 0;			// Set Reload Value
	TIM_Init.Init.CounterMode = TIM_COUNTERMODE_UP;	//Set timer to count up.

	/* Initialise Timer */
	HAL_TIM_Base_Init(&TIM_Init);

	/* Set priority of Timer 2 update Interrupt [0 (HIGH priority) to 15(LOW priority)] */
	/* 	DO NOT SET INTERRUPT PRIORITY HIGHER THAN 3 */
	HAL_NVIC_SetPriority(TIM4_IRQn, 3, 0);		//Set Main priority ot 10 and sub-priority ot 0.

	/* Enable timer update interrupt and interrupt vector for Timer  */
	NVIC_SetVector(TIM4_IRQn, (uint32_t)&tim2_irqhandler);
	NVIC_EnableIRQ(TIM4_IRQn);

	/* Start Timer */
	HAL_TIM_Base_Start_IT(&TIM_Init);
}

void tim2_irqhandler( void ){
	//Clear Update Flag
	__HAL_TIM_CLEAR_IT(&TIM_Init, TIM_IT_UPDATE);

	//Start the conversion for both ADCs
	HAL_ADC_Start(&AdcHandle1);
	HAL_ADC_Start(&AdcHandle2);

	//Wait for first ADC Conversion to complete
	while (HAL_ADC_PollForConversion(&AdcHandle1, 1) != HAL_OK);
	data1[sampleNo%1000] = (uint16_t)(HAL_ADC_GetValue(&AdcHandle1));

	//Second one will have definatly completed by now however still make sure
	while (HAL_ADC_PollForConversion(&AdcHandle2, 1) != HAL_OK);
	data2[sampleNo%1000] = (uint16_t)(HAL_ADC_GetValue(&AdcHandle2));

	//Increment sample number to store next sample
	sampleNo++;
}
/**
  * @brief  Initialise Hardware Peripherals used.
  * @param  None
  * @retval None
  */
void hardware_init() {
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable A0 GPIO Clock */
	__BRD_A0_GPIO_CLK();
	__BRD_A1_GPIO_CLK();

	/* Enable ADC1 clock */
	__ADC1_CLK_ENABLE();
	__ADC2_CLK_ENABLE();

	/* Configure A0, A1 as analog input */
	GPIO_InitStructure.Pin = BRD_A0_PIN;			//Set A0 pin
	GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;		//Set to Analog input
	GPIO_InitStructure.Pull = GPIO_NOPULL ;			//No Pull up resister
	HAL_GPIO_Init(BRD_A0_GPIO_PORT, &GPIO_InitStructure);	//Initialise AO

	GPIO_InitStructure.Pin = BRD_A1_PIN;			//Set A0 pin
	HAL_GPIO_Init(BRD_A1_GPIO_PORT, &GPIO_InitStructure);	//Initialise AO

	/* ADC For A0 - Microphone 1 */
	AdcHandle1.Instance = (ADC_TypeDef *)(ADC1_BASE);						//Use ADC1
	AdcHandle1.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV2;	//Set clock prescaler
	AdcHandle1.Init.Resolution            = ADC_RESOLUTION12b;				//Set 12-bit data resolution
	AdcHandle1.Init.ScanConvMode          = DISABLE;
	AdcHandle1.Init.ContinuousConvMode    = DISABLE;
	AdcHandle1.Init.DiscontinuousConvMode = DISABLE;
	AdcHandle1.Init.NbrOfDiscConversion   = 0;
	AdcHandle1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;	//No Trigger
	AdcHandle1.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T1_CC1;		//No Trigger
	AdcHandle1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;				//Right align data
	AdcHandle1.Init.NbrOfConversion       = 1;
	AdcHandle1.Init.DMAContinuousRequests = DISABLE;
	AdcHandle1.Init.EOCSelection          = DISABLE;
	HAL_ADC_Init(&AdcHandle1);		//Initialise ADC

	/* Configure ADC Channel */
	AdcChanConfig1.Channel = BRD_A0_ADC_CHAN;							//Use AO pin
	AdcChanConfig1.Rank         = 1;
	AdcChanConfig1.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	AdcChanConfig1.Offset       = 0;
	HAL_ADC_ConfigChannel(&AdcHandle1, &AdcChanConfig1);		//Initialise ADC channel

	/* ADC For A1 - Microphone 2 */
	AdcHandle2.Instance = (ADC_TypeDef *)(ADC2_BASE);						//Use ADC1
	AdcHandle2.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV2;	//Set clock prescaler
	AdcHandle2.Init.Resolution            = ADC_RESOLUTION12b;				//Set 12-bit data resolution
	AdcHandle2.Init.ScanConvMode          = DISABLE;
	AdcHandle2.Init.ContinuousConvMode    = DISABLE;
	AdcHandle2.Init.DiscontinuousConvMode = DISABLE;
	AdcHandle2.Init.NbrOfDiscConversion   = 0;
	AdcHandle2.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;	//No Trigger
	AdcHandle2.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T1_CC1;		//No Trigger
	AdcHandle2.Init.DataAlign             = ADC_DATAALIGN_RIGHT;				//Right align data
	AdcHandle2.Init.NbrOfConversion       = 1;
	AdcHandle2.Init.DMAContinuousRequests = DISABLE;
	AdcHandle2.Init.EOCSelection          = DISABLE;
	HAL_ADC_Init(&AdcHandle2);		//Initialise ADC

	/* Configure ADC Channel */
	AdcChanConfig1.Channel = BRD_A1_ADC_CHAN;							//Use AO pin
	HAL_ADC_ConfigChannel(&AdcHandle2, &AdcChanConfig1);		//Initialise ADC channel
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

