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
DMA_HandleTypeDef DMAHandle;

int sampleNo = 0;

//Temp storage for data as we fill the buffer
uint16_t data1[256];
uint32_t data2[256];

uint8_t freq_out = 0, freq_out2 = 0;

uint16_t value = 0;

/* Private function prototypes -----------------------------------------------*/
void Delay(__IO unsigned long nCount);
void hardware_init();
void adc_switch_channel_0( void );
void adc_switch_channel_1( void );
void tim2_irqhandler( void );
void tim3_dma( void );
void timer_interupt_init( void );
void DMACompleteISR( void );
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle);

void Error_Handler( void );


/**
  * @brief  Main program - enable ADC input on A0
  * @param  None
  * @retval None
  */
int main(void) {
	unsigned int adc_value;

	BRD_init();	//Initialise the NP2 board.
	hardware_init();	//Initialise Hardware peripheral
	timer_interupt_init();

	while (1){
	}
}


void timer_interupt_init(){
	uint32_t PrescalerValue;
	TIM_MasterConfigTypeDef Tim3MasterConfigHandle;

	/* Timer 4 clock enable */
	__TIM3_CLK_ENABLE();

	/* Compute the prescaler value */
	PrescalerValue = (uint16_t) ((SystemCoreClock/4)/515000) - 1;		//Set clock prescaler to 50kHz - SystemCoreClock is the system clock frequency.

	TIM_Init.Instance = TIM3;				//Enable Timer 2
	TIM_Init.Init.Period = 10;			//Set period count to be 1ms, so timer interrupt occurs every 1ms.
	TIM_Init.Init.Prescaler = PrescalerValue;	//Set presale value
	TIM_Init.Init.ClockDivision = 0;			//Set clock division
	TIM_Init.Init.RepetitionCounter = 0;	// Set Reload Value
	TIM_Init.Init.CounterMode = TIM_COUNTERMODE_UP;	//Set timer to count up.

	/* Initialise Timer */
	HAL_TIM_Base_Init(&TIM_Init);

	/* Set priority of Timer 2 update Interrupt [0 (HIGH priority) to 15(LOW priority)] */
	/* 	DO NOT SET INTERRUPT PRIORITY HIGHER THAN 3 */
	HAL_NVIC_SetPriority(TIM3_IRQn, 3, 0);		//Set Main priority ot 10 and sub-priority ot 0.
	NVIC_SetVector(TIM3_IRQn, (uint32_t)&tim2_irqhandler);

	Tim3MasterConfigHandle.MasterOutputTrigger	= TIM_TRGO_UPDATE;
	Tim3MasterConfigHandle.MasterSlaveMode 		= TIM_MASTERSLAVEMODE_ENABLE;
	HAL_TIMEx_MasterConfigSynchronization(&TIM_Init, &Tim3MasterConfigHandle);

	NVIC_EnableIRQ(TIM3_IRQn);

	/* Start Timer */
	HAL_TIM_Base_Start_IT(&TIM_Init);
}

void tim2_irqhandler( void ){
	__HAL_TIM_CLEAR_IT(&TIM_Init, TIM_IT_UPDATE);
	HAL_GPIO_WritePin(BRD_D1_GPIO_PORT, BRD_D1_PIN, 1);
	Delay(1);
	HAL_GPIO_WritePin(BRD_D1_GPIO_PORT, BRD_D1_PIN, 0);
}

/**
  * @brief  Initialise Hardware Peripherals used.
  * @param  None
  * @retval None
  */
void hardware_init() {
	BRD_LEDInit();
	BRD_LEDOff();

	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable A0 GPIO Clock */
	__BRD_A0_GPIO_CLK();
	__BRD_A1_GPIO_CLK();
	__BRD_D0_GPIO_CLK();
	__BRD_A5_GPIO_CLK();

	/* Enable ADC1 clock */
	__ADC1_CLK_ENABLE();
	__ADC2_CLK_ENABLE();

	__DMA2_CLK_ENABLE();

	/* Configure A0, A1 as analog input */
	GPIO_InitStructure.Pin 		= BRD_A0_PIN;			//Set A0 pin
	GPIO_InitStructure.Mode 	= GPIO_MODE_ANALOG;		//Set to Analog input
	GPIO_InitStructure.Pull 	= GPIO_NOPULL ;			//No Pull up resister
	HAL_GPIO_Init(BRD_A0_GPIO_PORT, &GPIO_InitStructure);	//Initialise AO

	GPIO_InitStructure.Pin 		= BRD_A1_PIN;			//Set A0 pin
	HAL_GPIO_Init(BRD_A1_GPIO_PORT, &GPIO_InitStructure);	//Initialise AO

	GPIO_InitStructure.Pin 		= BRD_D0_PIN;			//Set A0 pin
	GPIO_InitStructure.Mode 	= GPIO_MODE_OUTPUT_PP;		//Set to Analog input
	GPIO_InitStructure.Pull 	= GPIO_PULLDOWN ;			//No Pull up resister
	HAL_GPIO_Init(BRD_D0_GPIO_PORT, &GPIO_InitStructure);	//Initialise AO

	GPIO_InitStructure.Pin 		= BRD_D1_PIN;			//Set A0 pin
	GPIO_InitStructure.Mode 	= GPIO_MODE_OUTPUT_PP;		//Set to Analog input
	GPIO_InitStructure.Pull 	= GPIO_PULLDOWN ;			//No Pull up resister
	HAL_GPIO_Init(BRD_D1_GPIO_PORT, &GPIO_InitStructure);	//Initialise AO

	GPIO_InitStructure.Pin 		= BRD_A5_PIN;			//Set A0 pin
	GPIO_InitStructure.Mode 	= GPIO_MODE_ANALOG;		//Set to Analog input
	GPIO_InitStructure.Pull 	= GPIO_NOPULL ;			//No Pull up resister
	HAL_GPIO_Init(BRD_A5_GPIO_PORT, &GPIO_InitStructure);	//Initialise AO

	DMAHandle.Instance 					= DMA2_Stream0;
	DMAHandle.Init.Channel				= DMA_CHANNEL_0;
	DMAHandle.Init.Direction 			= DMA_PERIPH_TO_MEMORY;
	DMAHandle.Init.PeriphInc 			= DMA_PINC_DISABLE;
	DMAHandle.Init.MemInc 				= DMA_MINC_ENABLE;
	DMAHandle.Init.PeriphDataAlignment 	= DMA_PDATAALIGN_HALFWORD;
	DMAHandle.Init.MemDataAlignment 	= DMA_MDATAALIGN_HALFWORD;
	DMAHandle.Init.Mode 				= DMA_CIRCULAR; //DMA_NORMAL
	DMAHandle.Init.Priority 			= DMA_PRIORITY_LOW;
	DMAHandle.Init.FIFOMode 			= DMA_FIFOMODE_ENABLE;
	HAL_DMA_Init(&DMAHandle);

	/* ADC For A0 - Microphone 1 */
	AdcHandle1.Instance = (ADC_TypeDef *)ADC1_BASE;
	AdcHandle1.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV8;
	AdcHandle1.Init.Resolution = ADC_RESOLUTION_12B;
	AdcHandle1.Init.ScanConvMode = DISABLE;
	AdcHandle1.Init.ContinuousConvMode = DISABLE;
	AdcHandle1.Init.DiscontinuousConvMode = DISABLE;
	AdcHandle1.Init.NbrOfDiscConversion = 1;
	AdcHandle1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISINGFALLING;
	AdcHandle1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
	AdcHandle1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	AdcHandle1.Init.NbrOfConversion = 1;
	AdcHandle1.Init.DMAContinuousRequests = ENABLE;
	AdcHandle1.Init.EOCSelection = DISABLE;

	AdcHandle1.DMA_Handle 		= &DMAHandle;
	DMAHandle.Parent 			= &AdcHandle1;

	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 10, 1);
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
	NVIC_SetVector(DMA2_Stream0_IRQn, (uint16_t)&DMACompleteISR);

	__HAL_LINKDMA(&AdcHandle1, DMA_Handle, DMAHandle);

	if(HAL_ADC_Init(&AdcHandle1) != HAL_OK){
		//Init failed
		debug_printf("ERROR");
		BRD_LEDOn();
		for(;;);
	}

	HAL_ADC_Start_IT(&AdcHandle1);

	/* Configure ADC Channel */
	AdcChanConfig1.Channel 		= BRD_A0_ADC_CHAN;	//Use AO pin
	AdcChanConfig1.Rank         = 1;
	AdcChanConfig1.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	AdcChanConfig1.Offset       = 0;

	if( HAL_ADC_ConfigChannel(&AdcHandle1, &AdcChanConfig1) != HAL_OK){
		debug_printf("ERROR");
		BRD_LEDOn();
		for(;;);
	}

	if(HAL_ADC_Start_DMA(&AdcHandle1, (uint32_t*)data1, 1) != HAL_OK){
		//Init failed
		BRD_LEDOn();
		debug_printf("ERROR");
		for(;;);
	}

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle) {
	//debug_printf("ADC: %d\n", data1[0]);
}

void HAL_ADC_ErrorCallback (ADC_HandleTypeDef * hadc){
	debug_printf("ADC ERROR\n");
}

void DMACompleteISR( void ){
	//Data transfer is complete!
	HAL_DMA_IRQHandler(AdcHandle1.DMA_Handle);
	HAL_GPIO_WritePin(BRD_D0_GPIO_PORT, BRD_D0_PIN, 1);
	Delay(1);
	HAL_GPIO_WritePin(BRD_D0_GPIO_PORT, BRD_D0_PIN, 0);
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


/**
 * This is the old code for the other ADC
 */
//	/* ADC For A1 - Microphone 2 */
//	AdcHandle2.Instance = (ADC_TypeDef *)(ADC2_BASE);						//Use ADC1
//	AdcHandle2.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV2;	//Set clock prescaler
//	AdcHandle2.Init.Resolution            = ADC_RESOLUTION12b;				//Set 12-bit data resolution
//	AdcHandle2.Init.ScanConvMode          = DISABLE;
//	AdcHandle2.Init.ContinuousConvMode    = DISABLE;
//	AdcHandle2.Init.DiscontinuousConvMode = DISABLE;
//	AdcHandle2.Init.NbrOfDiscConversion   = 0;
//	AdcHandle2.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;	//No Trigger
//	AdcHandle2.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T1_CC1;		//No Trigger
//	AdcHandle2.Init.DataAlign             = ADC_DATAALIGN_RIGHT;				//Right align data
//	AdcHandle2.Init.NbrOfConversion       = 1;
//	AdcHandle2.Init.DMAContinuousRequests = DISABLE;
//	AdcHandle2.Init.EOCSelection          = DISABLE;
//	HAL_ADC_Init(&AdcHandle2);		//Initialise ADC
//
//	/* Configure ADC Channel */
//	AdcChanConfig1.Channel = BRD_A1_ADC_CHAN;							//Use AO pin
//	HAL_ADC_ConfigChannel(&AdcHandle2, &AdcChanConfig1);		//Initialise ADC channel
//
//	GPIO_InitStructure.Pin = BRD_D2_PIN;				//Pin
//	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;		//Output Mode
//	GPIO_InitStructure.Pull = GPIO_PULLDOWN;			//Enable Pull up, down or no pull resister
//	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;			//Pin latency
//	HAL_GPIO_Init(BRD_D2_GPIO_PORT, &GPIO_InitStructure);	//Initialise Pin


void Error_Handler(){
	debug_printf("ERROR");
	BRD_LEDOff();
}