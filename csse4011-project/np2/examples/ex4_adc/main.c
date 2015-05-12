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
#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "debug_printf.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef AdcHandle;
ADC_ChannelConfTypeDef AdcChanConfig;

/* Private function prototypes -----------------------------------------------*/
void Delay(__IO unsigned long nCount);
void hardware_init();


/**
  * @brief  Main program - enable ADC input on A0
  * @param  None
  * @retval None
  */
int main(void) {
	unsigned int adc_value;

	BRD_init();	//Initialise the NP2 board.
	hardware_init();	//Initialise Hardware peripherals
	
	/* Main processing loop performing ADC conversion every 1s. */
    while (1) {
  
		HAL_ADC_Start(&AdcHandle); // Start ADC conversion

		//Wait for ADC Conversion to complete
	    while (HAL_ADC_PollForConversion(&AdcHandle, 10) != HAL_OK);
    	adc_value = (uint16_t)(HAL_ADC_GetValue(&AdcHandle));

		/* Print ADC conversion values */
		debug_printf("ADC Value: %X\n\r", adc_value);

		BRD_LEDToggle();	//Toggle LED on/off
		Delay(0x7FFF00);	//Delay function
		 
  	}
}

/**
  * @brief  Initialise Hardware Peripherals used.
  * @param  None
  * @retval None
  */
void hardware_init() {

	GPIO_InitTypeDef GPIO_InitStructure;	
	
	BRD_LEDInit();		//Initialise Blue LED
	BRD_LEDOff();		//Turn off Blue LED

	/* Enable A0 GPIO Clock */
	__BRD_A0_GPIO_CLK();

	/* Configure A0 as analog input */
  	GPIO_InitStructure.Pin = BRD_A0_PIN;			//Set A0 pin
  	GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;		//Set to Analog input
  	GPIO_InitStructure.Pull = GPIO_NOPULL ;			//No Pull up resister

  	HAL_GPIO_Init(BRD_A0_GPIO_PORT, &GPIO_InitStructure);	//Initialise AO

	/* Enable ADC1 clock */
	__ADC1_CLK_ENABLE();

    /* Configure ADC1 */
    AdcHandle.Instance = (ADC_TypeDef *)(ADC1_BASE);						//Use ADC1
    AdcHandle.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV2;	//Set clock prescaler
    AdcHandle.Init.Resolution            = ADC_RESOLUTION12b;				//Set 12-bit data resolution
    AdcHandle.Init.ScanConvMode          = DISABLE;
    AdcHandle.Init.ContinuousConvMode    = DISABLE;
    AdcHandle.Init.DiscontinuousConvMode = DISABLE;
    AdcHandle.Init.NbrOfDiscConversion   = 0;
    AdcHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;	//No Trigger
    AdcHandle.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T1_CC1;		//No Trigger
    AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;				//Right align data
    AdcHandle.Init.NbrOfConversion       = 1;
    AdcHandle.Init.DMAContinuousRequests = DISABLE;
    AdcHandle.Init.EOCSelection          = DISABLE;

    HAL_ADC_Init(&AdcHandle);		//Initialise ADC

	/* Configure ADC Channel */
	AdcChanConfig.Channel = BRD_A0_ADC_CHAN;							//Use AO pin
	AdcChanConfig.Rank         = 1;
    AdcChanConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    AdcChanConfig.Offset       = 0;    

	HAL_ADC_ConfigChannel(&AdcHandle, &AdcChanConfig);		//Initialise ADC channel
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

