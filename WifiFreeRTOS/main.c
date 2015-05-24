/**
  ******************************************************************************
  * @file    ex14_fr_ledflashing/main.c
  * @author  MDS
  * @date    04022015
  * @brief   FreeRTOS LED Flashing program.Creates a task to flash the onboard
  *			 Blue LED. Note the Idle task will also flash the Blue LED.
  ******************************************************************************
  *
  */

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "debug_printf.h"
#include "ESP8622.h"
#include "Ultrasonic.h"
#include "stddef.h"
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#define NODE_ID 1

/* Private typedef -----------------------------------------------------------*/
ADC_HandleTypeDef AdcHandle1;
ADC_HandleTypeDef AdcHandle2;
ADC_ChannelConfTypeDef AdcChanConfig1;
ADC_ChannelConfTypeDef AdcChanConfig2;
TIM_HandleTypeDef TIM_Init;
DMA_HandleTypeDef DMAHandle1;
DMA_HandleTypeDef DMAHandle2;


/* Private define ------------------------------------------------------------*/
#define BUFFER_SIZE 256

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
int sampleNo = 0;
// Audio sample buffers
uint16_t data1[BUFFER_SIZE];
uint16_t data2[BUFFER_SIZE];
uint16_t ready_data1[BUFFER_SIZE];
uint16_t ready_data2[BUFFER_SIZE];







//extern typedef struct Ap Access_Point;
int8_t client_Pipe = -1;
int32_t time_Offset = 0;	// time offset relative to master -time means infront
SemaphoreHandle_t esp_Semaphore;


/* Private function prototypes -----------------------------------------------*/
void Hardware_init();
void ApplicationIdleHook( void ); /* The idle hook is used to blink the Blue 'Alive LED' every second */
void testing_Task( void );
void Testing_Task( void );
void Software_timer( void );
void node_Scan();
void node_Send();
void adc_hardware_init();
void adc_switch_channel_0( void );
void adc_switch_channel_1( void );
void tim2_irqhandler( void );
void tim3_dma( void );
void timer_interupt_init( void );
void DMACompleteISR1( void );
void DMACompleteISR2( void );
void Error_Handler( void );
void Delay(uint32_t cycles);

/* Task Priorities ------------------------------------------------------------*/
#define TESTING_PRIORITY					( tskIDLE_PRIORITY + 4 )


/* Task Stack Allocations -----------------------------------------------------*/
#define TESTING_STACK_SIZE		( configMINIMAL_STACK_SIZE * 2 )


/**
  * @brief  Starts all the other tasks, then starts the scheduler.
  * @param  None
  * @retval None
  */
int main( void ) {

	BRD_init();
	Hardware_init();
	ESP8622_init(); //This initiates another task
	esp_Semaphore = xSemaphoreCreateMutex();


	/* adc sampling  setup */
	adc_hardware_init();
	timer_interupt_init();


	/* Start the task to flash the LED. */
	xTaskCreate( (void *) &Testing_Task, (const signed char *) "TEST", TESTING_STACK_SIZE, NULL, TESTING_PRIORITY, NULL );
	xTaskCreate( (void *) &Software_timer, (const signed char *) "TIME", TESTING_STACK_SIZE, NULL, TESTING_PRIORITY + 1, NULL );

	/* Start the scheduler.

	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used here. */

	vTaskStartScheduler();

	/* We should never get here as control is now taken by the scheduler. */
	return 0;
}




/**
  * @brief  LED Flashing Task.
  * @param  None
  * @retval None
  */
void Testing_Task( void ) {
	char SSID[50];
	char buffer[10];
	static const char test[50] = "aaaaaaaaaa1111111111222222222233333333334444444444";
	int i;
//	//Ultrasonic_init();
//	debug_printf("Begin testing\n\n");

	 Wifi_reset();

	 debug_printf("I AM NODE %d\n\n", NODE_ID);

	 Wifi_setmode();

	  sprintf(&(SSID[0]), "NUCLEOWSN%d", NODE_ID);
	  Wifi_setAP(SSID,"password", 5, 0);

	  Wifi_set_AP_IP("192.168.3.1");
//
//	 //Wifi_join("NUCLEOWSN1", "");
//
	 Wifi_enserver();
	 Wifi_get_AP_IP();

	 //Wifi_connectTCP("192.168.1.1", 8888);

	//Wifi_timesync();

	for (;;) {
		/* Toggle LED */

//		Wifi_listAPs();
//		Access_Point* ap = (Access_Point*)get_AP("NUCLEOWSN1");
//		if (ap != NULL) {
//		   //debug_printf("RSSI: %d Distance: %f\n", ap->RSSI, RSSItoDistance(ap->RSSI));
//		}
//
//		Ultrasonic_start();
		// vTaskDelay(1000);
		//
		// // int len = sprintf(&buffer, "DA:[125%d]", Ultrasonic_getdist());
		// // Wifi_senddata(0, buffer, len);
		//
		// vTaskDelay(1000);
		//
//		int len = sprintf(&buffer, "DA:[124%d]", ap->RSSI);
//		Wifi_senddata(0, buffer, len);
//
//		debug_printf("RSSI: %d\n", ap->RSSI);
//		debug_printf("Distance: %d\n", Ultrasonic_getdist());
//		debug_printf("Width: %d\n",Ultrasonic_getwidth());

		/* Delay the task for 1000ms */

		for(i = 0; i < BUFFER_SIZE; i++){
					debug_printf("%03X %03X\n", ready_data1[i], ready_data2[i]);
				}
		if (client_Pipe >= 0 && client_Pipe <= 4) {
				Wifi_senddata(client_Pipe, test, 50);
		}
		vTaskDelay(200);
	}
}



void timer_interupt_init(){
	uint32_t PrescalerValue;
	TIM_MasterConfigTypeDef Tim3MasterConfigHandle;

	/* Timer 4 clock enable */
	__TIM3_CLK_ENABLE();

	/* Compute the prescaler value */
	PrescalerValue = (uint16_t) ((SystemCoreClock/2)/500000);		//Set clock prescaler to 500kHz - SystemCoreClock is the system clock frequency.

	TIM_Init.Instance = TIM3;				//Enable Timer 2
	TIM_Init.Init.Period = 4;			//Set period count to be 1ms, so timer interrupt occurs every 1ms.
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
}


/**
  * @brief  Initialise Hardware Peripherals used.
  * @param  None
  * @retval None
  */
void adc_hardware_init() {


	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable A0 GPIO Clock */
	__BRD_A0_GPIO_CLK();
	__BRD_A1_GPIO_CLK();
	__BRD_D0_GPIO_CLK();
	__BRD_A5_GPIO_CLK();

	/* Enable ADC1 clock */
	__ADC1_CLK_ENABLE();
	__ADC2_CLK_ENABLE();

	__DMA1_CLK_ENABLE();
	__DMA2_CLK_ENABLE();

	/* Configure A0, A1 as analog input */
	GPIO_InitStructure.Pin 		= BRD_A0_PIN;			//Set A0 pin
	GPIO_InitStructure.Mode 	= GPIO_MODE_ANALOG;		//Set to Analog input
	GPIO_InitStructure.Pull 	= GPIO_NOPULL ;			//No Pull up resister
	HAL_GPIO_Init(BRD_A0_GPIO_PORT, &GPIO_InitStructure);	//Initialise AO

	GPIO_InitStructure.Pin 		= BRD_A1_PIN;			//Set A0 pin
	HAL_GPIO_Init(BRD_A1_GPIO_PORT, &GPIO_InitStructure);	//Initialise AO

	DMAHandle1.Instance 					= DMA2_Stream0;
	DMAHandle1.Init.Channel				= DMA_CHANNEL_0;
	DMAHandle1.Init.Direction 			= DMA_PERIPH_TO_MEMORY;
	DMAHandle1.Init.PeriphInc 			= DMA_PINC_DISABLE;
	DMAHandle1.Init.MemInc 				= DMA_MINC_ENABLE;
	DMAHandle1.Init.PeriphDataAlignment 	= DMA_PDATAALIGN_HALFWORD;
	DMAHandle1.Init.MemDataAlignment 	= DMA_MDATAALIGN_HALFWORD;
	DMAHandle1.Init.Mode 				= DMA_CIRCULAR; //DMA_NORMAL
	DMAHandle1.Init.Priority 			= DMA_PRIORITY_LOW;
	DMAHandle1.Init.FIFOMode 			= DMA_FIFOMODE_ENABLE;
	HAL_DMA_Init(&DMAHandle1);

	/* ADC For A0 - Microphone 1 */
	AdcHandle1.Instance = (ADC_TypeDef *)ADC1_BASE;
	AdcHandle1.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV8;
	AdcHandle1.Init.Resolution = ADC_RESOLUTION_12B;
	AdcHandle1.Init.ScanConvMode = ENABLE;
	AdcHandle1.Init.ContinuousConvMode = DISABLE;
	AdcHandle1.Init.DiscontinuousConvMode = DISABLE;
	AdcHandle1.Init.NbrOfDiscConversion = 1;
	AdcHandle1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISINGFALLING;
	AdcHandle1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
	AdcHandle1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	AdcHandle1.Init.NbrOfConversion = 1;
	AdcHandle1.Init.DMAContinuousRequests = ENABLE;
	AdcHandle1.Init.EOCSelection = DISABLE;

	AdcHandle1.DMA_Handle 		= &DMAHandle1;
	DMAHandle1.Parent 			= &AdcHandle1;

	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 10, 1);
	NVIC_SetVector(DMA2_Stream0_IRQn, (uint16_t)&DMACompleteISR1);
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);


	__HAL_LINKDMA(&AdcHandle1, DMA_Handle, DMAHandle1);

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

	if(HAL_ADC_Start_DMA(&AdcHandle1, (uint32_t*)data1, BUFFER_SIZE) != HAL_OK){
		//Init failed
		BRD_LEDOn();
		debug_printf("ERROR");
		for(;;);
	}

	// WE NEED TO DO THE SAME THING FOR THE SECOND ADC :'( DMA1 A1

	DMAHandle2.Instance 				= DMA2_Stream2;
	DMAHandle2.Init.Channel				= DMA_CHANNEL_1;
	DMAHandle2.Init.Direction 			= DMA_PERIPH_TO_MEMORY;
	DMAHandle2.Init.PeriphInc 			= DMA_PINC_DISABLE;
	DMAHandle2.Init.MemInc 				= DMA_MINC_ENABLE;
	DMAHandle2.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
	DMAHandle2.Init.MemDataAlignment 	= DMA_MDATAALIGN_HALFWORD;
	DMAHandle2.Init.Mode 				= DMA_CIRCULAR; //DMA_NORMAL
	DMAHandle2.Init.Priority 			= DMA_PRIORITY_LOW;
	DMAHandle2.Init.FIFOMode 			= DMA_FIFOMODE_ENABLE;
	HAL_DMA_Init(&DMAHandle2);

	/* ADC For A1 - Microphone 2 */
	AdcHandle2.Instance = (ADC_TypeDef *)ADC2_BASE;
	AdcHandle2.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV8;
	AdcHandle2.Init.Resolution = ADC_RESOLUTION_12B;
	AdcHandle2.Init.ScanConvMode = ENABLE;
	AdcHandle2.Init.ContinuousConvMode = DISABLE;
	AdcHandle2.Init.DiscontinuousConvMode = DISABLE;
	AdcHandle2.Init.NbrOfDiscConversion = 1;
	AdcHandle2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISINGFALLING;
	AdcHandle2.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
	AdcHandle2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	AdcHandle2.Init.NbrOfConversion = 1;
	AdcHandle2.Init.DMAContinuousRequests = ENABLE;
	AdcHandle2.Init.EOCSelection = DISABLE;

	AdcHandle2.DMA_Handle 		= &DMAHandle2;
	DMAHandle2.Parent 			= &AdcHandle2;

	HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 10, 1);
	NVIC_SetVector(DMA2_Stream2_IRQn, (uint16_t)&DMACompleteISR2);
	HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);


	__HAL_LINKDMA(&AdcHandle2, DMA_Handle, DMAHandle2);

	if(HAL_ADC_Init(&AdcHandle2) != HAL_OK){
		//Init failed
		debug_printf("ERROR");
		BRD_LEDOn();
		for(;;);
	}

	HAL_ADC_Start_IT(&AdcHandle2);

	/* Configure ADC Channel */
	AdcChanConfig2.Channel 		= BRD_A1_ADC_CHAN;	//Use AO pin
	AdcChanConfig2.Rank         = 1;
	AdcChanConfig2.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	AdcChanConfig2.Offset       = 0;

	if( HAL_ADC_ConfigChannel(&AdcHandle2, &AdcChanConfig2) != HAL_OK){
		debug_printf("ERROR");
		BRD_LEDOn();
		for(;;);
	}

	if(HAL_ADC_Start_DMA(&AdcHandle2, (uint32_t*)data2, BUFFER_SIZE) != HAL_OK){
		//Init failed
		BRD_LEDOn();
		debug_printf("ERROR");
		for(;;);
	}

}

void HAL_ADC_ErrorCallback (ADC_HandleTypeDef * hadc){
	debug_printf("ADC ERROR\n");
}

void DMACompleteISR1( void ){
	//Data transfer is complete! Handle the interupt.
	HAL_DMA_IRQHandler(AdcHandle1.DMA_Handle);

	int i;
	for(i = 0; i < BUFFER_SIZE; i++){
		ready_data1[i] = data1[i];
	}
}

void DMACompleteISR2( void ){
	//Data transfer is complete! Handle the interupt.
	HAL_DMA_IRQHandler(AdcHandle2.DMA_Handle);


	int i;
	for(i = 0; i < BUFFER_SIZE; i++){
		ready_data2[i] = data2[i];
	}
}


void Software_timer(){
	uint16_t ledsync  = 0;
	uint16_t node_Sync  = 0;
	uint8_t flag  = 1;
	uint8_t node_scan = 0;
	uint8_t nodes_send = 0;

	uint16_t node_period = 10000;
	// time variable is
	for (;;) {

		ledsync = ((xTaskGetTickCount()+time_Offset) % 1000);
		node_Sync = ((xTaskGetTickCount()+time_Offset) % node_period);

		/*	led sync	*/
		if (ledsync > (500 / portTICK_RATE_MS) && flag == 1) {
			//debug_printf("IDLE Tick %d\n", xLastTx);
			flag = 0;
			BRD_LEDToggle();

		}
		if (ledsync < (10 / portTICK_RATE_MS) && flag == 0) {
			flag = 1;
		}



		/*		node sync		*/

		if (node_Sync > ((NODE_ID*3000) / portTICK_RATE_MS) && node_scan == 1) {
			//debug_printf("IDLE Tick %d\n", xLastTx);
			node_scan = 0;
			node_Scan();

		}
		if (node_Sync < (10 / portTICK_RATE_MS) && node_scan == 0) {
			node_scan = 1;
		}

		if (node_Sync % 2000 > 1980 && nodes_send == 1) {
			//debug_printf("IDLE Tick %d\n", xLastTx);
			nodes_send = 0;
			node_Send();

		}
		if (node_Sync % 2000 < 20 && nodes_send == 0) {
			nodes_send = 1;
		}



		vTaskDelay(1);
	}
}





void node_Scan() {
	if (esp_Semaphore != NULL) {
		if( xSemaphoreTake( esp_Semaphore, ( TickType_t ) 10 ) == pdTRUE ) {
			Wifi_listAPs();
			 Access_Point* ap = (Access_Point*)get_AP("Wu-Tang LAN");
			 if (ap != NULL) {
				debug_printf("RSSI: %d Distance: %f\n", ap->RSSI, RSSItoDistance(ap->RSSI));
			 }
			 xSemaphoreGive(esp_Semaphore);
		}
	}
}


void node_Send() {

}


/**
  * @brief  Hardware Initialisation.
  * @param  None
  * @retval None
  */
void Hardware_init( void ) {

	portDISABLE_INTERRUPTS();	//Disable interrupts

	BRD_LEDInit();				//Initialise Blue LED
	BRD_LEDOff();				//Turn off Blue LED

	portENABLE_INTERRUPTS();	//Enable interrupts

}

/**
  * @brief  Application Tick Task.
  * @param  None
  * @retval None
  */
void vApplicationTickHook( void ) {
	BRD_LEDOff();
}

/**
  * @brief  Idle Application Task
  * @param  None
  * @retval None
  */
void vApplicationIdleHook( void ) {
	static portTickType xLastTx = 0;

	BRD_LEDOff();

	for (;;) {

		/* The idle hook simply prints the idle tick count, every second */
		if ((xTaskGetTickCount() - xLastTx ) > (1000 / portTICK_RATE_MS)) {

			xLastTx = xTaskGetTickCount();

			//debug_printf("IDLE Tick %d\n", xLastTx);

			/* Blink Alive LED */
			BRD_LEDToggle();
		}
	}
}
//Comment
/**
  * @brief  vApplicationStackOverflowHook
  * @param  Task Handler and Task Name
  * @retval None
  */
void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName ) {
	/* This function will get called if a task overflows its stack.   If the
	parameters are corrupt then inspect pxCurrentTCB to find which was the
	offending task. */

	BRD_LEDOff();
	( void ) pxTask;
	( void ) pcTaskName;

	for( ;; );
}


void Delay(uint32_t cycles) {
	uint32_t i = 0;
	for (i = 0; i < cycles; i++) {
		//nothing
	}
}
