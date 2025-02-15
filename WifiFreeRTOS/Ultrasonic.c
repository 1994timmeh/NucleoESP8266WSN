#include "Ultrasonic.h"
#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "debug_printf.h"
#include <stdint.h>
#include <string.h>

GPIO_InitTypeDef GPIO_InitStructure;
TIM_IC_InitTypeDef  TIM_ICInitStructure;
TIM_HandleTypeDef TIM_IC_Init;
uint16_t Last_distance = 0;
uint16_t last_edge = 0;
uint16_t last_width = 0;

extern uint8_t client_Pipe;

/**
  * Initialises pins and timers for ultrasonic ranger
  */
void Ultrasonic_init(){
  uint16_t PrescalerValue = 0;
  __TIM3_CLK_ENABLE();

  __BRD_D11_GPIO_CLK();
  __BRD_D12_GPIO_CLK();

  //PIN 11 - ECHO
  //PIN 12 - PULSE

  //Init 2 pins for ECHO and PULSE
  GPIO_InitStructure.Pin = BRD_D12_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull = GPIO_PULLDOWN;
  GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(BRD_D12_GPIO_PORT, &GPIO_InitStructure);

  /* Configure the D0 pin with TIM3 input capture */
  GPIO_InitStructure.Pin = BRD_D11_PIN;				//Pin
  GPIO_InitStructure.Mode = GPIO_MODE_AF_PP; 		//Set mode to be output alternate
  GPIO_InitStructure.Pull = GPIO_NOPULL;			//Enable Pull up, down or no pull resister
  GPIO_InitStructure.Speed = GPIO_SPEED_FAST;			//Pin latency
  GPIO_InitStructure.Alternate = GPIO_AF2_TIM3;	//Set alternate function to be timer 2
  HAL_GPIO_Init(BRD_D11_GPIO_PORT, &GPIO_InitStructure);	//Initialise Pin

  /* Compute the prescaler value. SystemCoreClock = 168000000 - set for 50Khz clock */
  PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 500000) - 1;

  /* Configure Timer 3 settings */
  TIM_IC_Init.Instance = TIM3;					//Enable Timer 3
  TIM_IC_Init.Init.Period = 100000;			//Set for 100ms (10Hz) period
  TIM_IC_Init.Init.Prescaler = PrescalerValue;	//Set presale value
  TIM_IC_Init.Init.ClockDivision = 0;			//Set clock division
  TIM_IC_Init.Init.RepetitionCounter = 0; 		// Set Reload Value
  TIM_IC_Init.Init.CounterMode = TIM_COUNTERMODE_UP;	//Set timer to count up.

  /* Configure TIM3 Input capture */
  TIM_ICInitStructure.ICPolarity = TIM_ICPOLARITY_BOTHEDGE;			//Set to trigger on rising edge
  TIM_ICInitStructure.ICSelection = TIM_ICSELECTION_DIRECTTI;
  TIM_ICInitStructure.ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.ICFilter = 0;

  /* Set priority of Timer 3 Interrupt [0 (HIGH priority) to 15(LOW priority)] */
  HAL_NVIC_SetPriority(TIM3_IRQn, 10, 0);	//Set Main priority ot 10 and sub-priority ot 0.

  //Enable Timer 3 interrupt and interrupt vector
  NVIC_SetVector(TIM3_IRQn, (uint32_t)&tim3_irqhandler);
  NVIC_EnableIRQ(TIM3_IRQn);

  /* Enable input capture for Timer 3, channel 2 */
  HAL_TIM_IC_Init(&TIM_IC_Init);
  HAL_TIM_IC_ConfigChannel(&TIM_IC_Init, &TIM_ICInitStructure, TIM_CHANNEL_2);

  /* Start Input Capture */
  HAL_TIM_IC_Start_IT(&TIM_IC_Init, TIM_CHANNEL_2);
}

/**
  * @brief  Timer 3 Input Capture Interrupt handler
  * @param  None.
  * @retval None
  */
void tim3_irqhandler(void) {
  unsigned int input_capture_value;
  //Clear Input Capture Flag
  __HAL_TIM_CLEAR_IT(&TIM_IC_Init, TIM_IT_TRIGGER);

  /* Read and display the Input Capture value of Timer 3, channel 2 */
  int edge = HAL_TIM_ReadCapturedValue(&TIM_IC_Init, TIM_CHANNEL_2);

  int difference = edge - last_edge;

  if(difference < 30000 && difference > 0){
    Last_distance = (int)(difference/58);
    last_width = difference;
  }

  last_edge = edge;
}

uint16_t Ultrasonic_getdist(){
  return Last_distance;
}

uint16_t Ultrasonic_getwidth(){
  return last_width;
}

/**
  * Starts ranging
  */
void Ultrasonic_start(){
  HAL_GPIO_WritePin(BRD_D12_GPIO_PORT, BRD_D12_PIN, 1);
  //TODO Check this with logic analyzer
  uDelay(SEC*0.000014); //10uS hopefully
  HAL_GPIO_WritePin(BRD_D12_GPIO_PORT, BRD_D12_PIN, 0);
}



void handle_Ultrasonic_Data(uint8_t node, uint8_t* data_String, uint8_t* raw_data){
	debug_printf("Ultrasonic data from Node: %d - %dcm\n\r", node, atoi(data_String));

	/* forward to client	*/
		if (client_Pipe >= 0 && client_Pipe <= 4) {
			Wifi_senddata(client_Pipe, raw_data, strlen(raw_data));
			/* HACK HERE	*/
		}

}



/**
  * @brief  Delay Function.
  * @param  nCount:specifies the Delay time length.
  * @retval None
  */
void uDelay(__IO unsigned long nCount) {
  	while(nCount--) {
  	}
}
