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
#include "main.h"
#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "debug_printf.h"
#include "ESP8622.h"
#include "Ultrasonic.h"
#include <stddef.h>
#include <stdio.h>
#include <string.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "audioProcessing.h"
#include "base64.h"

/* Private typedef -----------------------------------------------------------*/
ADC_HandleTypeDef AdcHandle1;
ADC_HandleTypeDef AdcHandle2;
ADC_ChannelConfTypeDef AdcChanConfig1;
ADC_ChannelConfTypeDef AdcChanConfig2;
TIM_HandleTypeDef TIM_Init;
DMA_HandleTypeDef DMAHandle1;
DMA_HandleTypeDef DMAHandle2;

/* Private variables ---------------------------------------------------------*/
volatile int sampleNo1 = 0, sampleNo2 = 0;
// Audio sample buffers
float data1[BUFFER_SIZE];
float data2[BUFFER_SIZE];
float32_t ready_data1[BUFFER_SIZE];
float32_t ready_data2[BUFFER_SIZE];
uint32_t buffer1;
uint32_t buffer2;

volatile uint16_t frameNumber = 0;

//extern typedef struct Ap Access_Point;
int8_t client_Pipe = -1;
int32_t time_Offset = 0;	// time offset relative to master -time means infront
SemaphoreHandle_t esp_Semaphore;
SemaphoreHandle_t processing_Semaphore;

QueueHandle_t validDataQueue;

TaskHandle_t AudioTaskHandle;

//#define MASTERNODE

/**
  * @brief  Starts all the other tasks, then starts the scheduler.
  * @param  None
  * @retval None
  */
int main( void ) {

	BRD_init();
	Hardware_init();
	esp_Semaphore = xSemaphoreCreateMutex();
	vSemaphoreCreateBinary(processing_Semaphore);


	/* adc sampling  setup */
	adc_hardware_init();
	timer_interupt_init();
	audioProcessingInit();

	validDataQueue = xQueueCreate(50, sizeof(struct frameResults));

	/* Start the task to flash the LED. */
	#ifndef MASTERNODE
		xTaskCreate( (void *) &Audio_Task, (const signed char *) "AUD", AUDIO_STACK_SIZE, NULL, AUDIO_PRIORITY , &AudioTaskHandle );
	#endif

	xTaskCreate( (void *) &TX_Task, (const signed char *) "TX", TX_STACK_SIZE, NULL, TX_PRIORITY, NULL );

	//xTaskCreate( (void *) &Software_timer, (const signed char *) "TIME", TESTING_STACK_SIZE, NULL, TESTING_PRIORITY + 1, NULL );

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

#ifdef TESTDATA
float32_t test1[] = {-1.2935,1.9267,0.23325,-0.54502,1.0328,0.37714,0.10926,-0.68019,-1.7654,-0.96357,-0.64563,0.45253,0.6145,-0.7853,0.82818,0.66742,-0.1255,-0.95123,0.89204,-0.12972,-1.0135,0.25229,0.52359,-1.4616,1.8664,-2.1491,-1.6352,1.224,0.32517,-0.42891,0.011596,0.73132,-0.86802,0.92816,0.69732,-0.04515,0.19121,-0.62876,-0.85661,-0.38871,1.0285,-0.23972,-0.45162,-0.97927,-1.1334,0.022121,1.0402,1.2315,0.56018,-0.30904,-0.3676,0.36563,0.93744,0.97424,-2.1189,-0.046465,-0.96678,-0.47125,1.7352,-0.77753,-1.6838,0.17703,-0.12448,-0.41555,0.87946,0.56613,0.37592,-0.27695,0.3501,-0.29131,0.18608,0.5766,0.33966,-0.67283,-0.53701,-1.0402,0.99728,-0.026054,-0.657,0.67775,-0.51083,0.446,1.5166,0.93776,-0.16016,-0.11384,1.9371,-0.1708,0.30117,0.32599,0.92484,0.21529,0.36625,0.32215,2.689,0.87607,-0.87649,-1.2174,-1.6148,1.8401,-0.81893,0.99214,0.53382,-1.5267,2.0229,0.5523,1.8201,0.34264,0.17956,-1.0193,0.037599,0.13712,-1.5211,-0.018918,0.16325,-0.72115,0.4106,-1.2126,-0.57374,0.10539,-0.6051,0.42179,-0.36281,-0.87412,0.93159,0.60004,0.57136,0.68511,1.0094,0.99087,0.033691,-0.45027,-0.11066,1.2379,-1.1979,-0.56439,1.0433,0.846,-0.49554,-0.20682,-0.1558,-0.2754,-2.4432,-0.42732,0.30906,2.4516,-1.4626,-0.63554,-0.38547,-0.94228,-0.67377,-1.9242,-0.11242,-0.51851,0.53498,0.057408,1.0658,1.6207,0.12185,-1.238,0.24413,1.3983,-0.095473,0.38762,-0.96631,1.5092,0.40383,-0.42214,-1.674,-0.68763,-1.0272,-0.49256,0.34681,0.82935,0.15563,0.083714,0.37462,2.6532,0.33271,0.14077,1.5778,0.089548,-0.67298,0.93189,-0.35789,0.2021,0.87625,0.80795,-1.6033,-2.3621,-0.7017,1.6519,0.23507,-0.15175,-0.15589,1.0382,0.33048,0.47581,-2.0905,-0.17403,0.019188,-0.86003,-0.022949,-0.60232,0.8699,-0.57081,1.3047,-0.042644,0.89553,2.2849,0.066827,1.4946,-1.0725,1.8233,-1.2084,-0.065392,-0.32684,-1.1492,-1.308,0.64638,-1.3892,-1.3407,1.1289,0.0016961,0.37968,-0.90133,-0.19679,0.30694,0.16746,1.7881,-0.62408,-0.14996,0.83222,0.94809,-1.9737,-0.39188,-0.67671,-0.016021,0.51517,0.44483,1.1409,0.44768,0.31544,0.94559,0.42866,-1.3246,0.10982,-1.6547,1.1107,-2.1079,-0.54984,0.094261,-0.03822,1.8829,0.055461,-0.61389,0.58702,-1.2067,0.54533,0.25093,-0.39275,-0.62204,-1.1905,-1.8785,-0.42401,0.77724,-0.71393,1.5846,-0.88831,2.1408,-0.69218,0.099297,1.435,1.2334,-0.75764,0.73858,-1.1144,-1.7059,0.66116,-1.7296,-2.1381,-0.060048,1.3857,1.2178,-1.4951,0.037283,0.80287,0.97385,1.5607,1.5862,0.8563,-1.4245,0.039702,-1.3799,1.2331,1.7421,-2.0015,0.8355,-0.34282,-0.47796,-0.8891,1.2634,0.3832,-0.11887,0.41725,1.0132,-0.86952,-0.79468,0.6885,1.5857,1.2502,-0.1156,-1.3318,-2.3428,-0.92656,1.1296,-0.54913,0.28373,0.21278,-2.2028,1.2511,2.0247,-0.038897,0.99858,-0.75733,0.59615,2.1232,1.3117,-0.69994,-1.0196,0.053115,-0.23702,-0.062736,1.2711,0.22114,1.664,-0.04296,-0.13324,0.7932,0.42627,-0.2099,-1.5882,-1.1164,0.61686,0.54346,-1.4679,0.11991,0.52936,-1.1752,-0.99822,1.194,-1.7537,0.44653,0.97806,-1.7913,-0.13019,-1.1752,0.089962,-0.42217,-1.6509,0.61797,0.59616,-0.61844,0.52788,-1.3443,-1.2348,-1.1534,-0.10933,-0.52419,1.9113,-0.071106,0.56184,0.57407,0.37173,0.19169,1.2441,-1.1642,1.6353,0.68633,-1.1495,1.1102,1.1826,1.7073,-0.42032,1.6394,0.26347,-1.4635,-0.77617,0.92949,1.793,-1.1832,-0.11118,-0.65677,1.8797,-0.89608,1.3315,-0.62457,0.77668,-1.3243,1.5003,-2.2029,0.32251,0.421,-0.8207,-0.96646,0.60935,-1.0362,-0.028238,-2.8198,-2.081,-0.044753,1.118,-1.6495,0.67867,0.49432,-0.58847,-0.023925,2.1913,-1.4006,0.4801,0.20755,0.32208,-0.0058805,0.28139,-0.25134,-1.6935,-0.58402,0.23444,0.25879,0.60366,2.221,-1.6541,0.68041,0.13583,-0.037987,-0.69314,-0.12753,0.68036,0.42621,-1.6057,0.90621,0.24951,1.8942,-1.1124,-0.72204,-1.7522,0.0036123,0.99514,-0.47897,-0.5191,-0.29466,-0.16055,1.0235,0.015895,-0.48266,0.63467,-1.3644,0.59862,-0.2065,2.1394,-0.64881,0.42359,0.11787,-1.0808,0.87553,-2.2263,-1.9461,-1.3096,1.3056,0.98397,-1.2514,-0.17975,-0.74341,0.23324,2.1013,-0.87667,1.9488,-0.46527,-0.65192,0.60963,0.70912,0.2798,-1.4675,-0.69132,-0.86799,-0.51251,-0.78209,0.42579,1.0115,-0.25067,0.55424,0.75888,0.13384,-0.27256,-0.54263,-1.1631,-0.27352,0.1595,-0.18671,0.2179,0.046238,0.67032,-0.6305,-0.11841,0.24206,-0.37608,0.28044,-1.7745,0.45858,0.46182,-0.0011875,-0.35294
};
float32_t test2[] = {0.63577,1.0957,-1.7048,-0.43299,-0.35294,-0.0011875,0.46182,0.45858,-1.7745,0.28044,-0.37608,0.24206,-0.11841,-0.6305,0.67032,0.046238,0.2179,-0.18671,0.1595,-0.27352,-1.1631,-0.54263,-0.27256,0.13384,0.75888,0.55424,-0.25067,1.0115,0.42579,-0.78209,-0.51251,-0.86799,-0.69132,-1.4675,0.2798,0.70912,0.60963,-0.65192,-0.46527,1.9488,-0.87667,2.1013,0.23324,-0.74341,-0.17975,-1.2514,0.98397,1.3056,-1.3096,-1.9461,-2.2263,0.87553,-1.0808,0.11787,0.42359,-0.64881,2.1394,-0.2065,0.59862,-1.3644,0.63467,-0.48266,0.015895,1.0235,-0.16055,-0.29466,-0.5191,-0.47897,0.99514,0.0036123,-1.7522,-0.72204,-1.1124,1.8942,0.24951,0.90621,-1.6057,0.42621,0.68036,-0.12753,-0.69314,-0.037987,0.13583,0.68041,-1.6541,2.221,0.60366,0.25879,0.23444,-0.58402,-1.6935,-0.25134,0.28139,-0.0058805,0.32208,0.20755,0.4801,-1.4006,2.1913,-0.023925,-0.58847,0.49432,0.67867,-1.6495,1.118,-0.044753,-2.081,-2.8198,-0.028238,-1.0362,0.60935,-0.96646,-0.8207,0.421,0.32251,-2.2029,1.5003,-1.3243,0.77668,-0.62457,1.3315,-0.89608,1.8797,-0.65677,-0.11118,-1.1832,1.793,0.92949,-0.77617,-1.4635,0.26347,1.6394,-0.42032,1.7073,1.1826,1.1102,-1.1495,0.68633,1.6353,-1.1642,1.2441,0.19169,0.37173,0.57407,0.56184,-0.071106,1.9113,-0.52419,-0.10933,-1.1534,-1.2348,-1.3443,0.52788,-0.61844,0.59616,0.61797,-1.6509,-0.42217,0.089962,-1.1752,-0.13019,-1.7913,0.97806,0.44653,-1.7537,1.194,-0.99822,-1.1752,0.52936,0.11991,-1.4679,0.54346,0.61686,-1.1164,-1.5882,-0.2099,0.42627,0.7932,-0.13324,-0.04296,1.664,0.22114,1.2711,-0.062736,-0.23702,0.053115,-1.0196,-0.69994,1.3117,2.1232,0.59615,-0.75733,0.99858,-0.038897,2.0247,1.2511,-2.2028,0.21278,0.28373,-0.54913,1.1296,-0.92656,-2.3428,-1.3318,-0.1156,1.2502,1.5857,0.6885,-0.79468,-0.86952,1.0132,0.41725,-0.11887,0.3832,1.2634,-0.8891,-0.47796,-0.34282,0.8355,-2.0015,1.7421,1.2331,-1.3799,0.039702,-1.4245,0.8563,1.5862,1.5607,0.97385,0.80287,0.037283,-1.4951,1.2178,1.3857,-0.060048,-2.1381,-1.7296,0.66116,-1.7059,-1.1144,0.73858,-0.75764,1.2334,1.435,0.099297,-0.69218,2.1408,-0.88831,1.5846,-0.71393,0.77724,-0.42401,-1.8785,-1.1905,-0.62204,-0.39275,0.25093,0.54533,-1.2067,0.58702,-0.61389,0.055461,1.8829,-0.03822,0.094261,-0.54984,-2.1079,1.1107,-1.6547,0.10982,-1.3246,0.42866,0.94559,0.31544,0.44768,1.1409,0.44483,0.51517,-0.016021,-0.67671,-0.39188,-1.9737,0.94809,0.83222,-0.14996,-0.62408,1.7881,0.16746,0.30694,-0.19679,-0.90133,0.37968,0.0016961,1.1289,-1.3407,-1.3892,0.64638,-1.308,-1.1492,-0.32684,-0.065392,-1.2084,1.8233,-1.0725,1.4946,0.066827,2.2849,0.89553,-0.042644,1.3047,-0.57081,0.8699,-0.60232,-0.022949,-0.86003,0.019188,-0.17403,-2.0905,0.47581,0.33048,1.0382,-0.15589,-0.15175,0.23507,1.6519,-0.7017,-2.3621,-1.6033,0.80795,0.87625,0.2021,-0.35789,0.93189,-0.67298,0.089548,1.5778,0.14077,0.33271,2.6532,0.37462,0.083714,0.15563,0.82935,0.34681,-0.49256,-1.0272,-0.68763,-1.674,-0.42214,0.40383,1.5092,-0.96631,0.38762,-0.095473,1.3983,0.24413,-1.238,0.12185,1.6207,1.0658,0.057408,0.53498,-0.51851,-0.11242,-1.9242,-0.67377,-0.94228,-0.38547,-0.63554,-1.4626,2.4516,0.30906,-0.42732,-2.4432,-0.2754,-0.1558,-0.20682,-0.49554,0.846,1.0433,-0.56439,-1.1979,1.2379,-0.11066,-0.45027,0.033691,0.99087,1.0094,0.68511,0.57136,0.60004,0.93159,-0.87412,-0.36281,0.42179,-0.6051,0.10539,-0.57374,-1.2126,0.4106,-0.72115,0.16325,-0.018918,-1.5211,0.13712,0.037599,-1.0193,0.17956,0.34264,1.8201,0.5523,2.0229,-1.5267,0.53382,0.99214,-0.81893,1.8401,-1.6148,-1.2174,-0.87649,0.87607,2.689,0.32215,0.36625,0.21529,0.92484,0.32599,0.30117,-0.1708,1.9371,-0.11384,-0.16016,0.93776,1.5166,0.446,-0.51083,0.67775,-0.657,-0.026054,0.99728,-1.0402,-0.53701,-0.67283,0.33966,0.5766,0.18608,-0.29131,0.3501,-0.27695,0.37592,0.56613,0.87946,-0.41555,-0.12448,0.17703,-1.6838,-0.77753,1.7352,-0.47125,-0.96678,-0.046465,-2.1189,0.97424,0.93744,0.36563,-0.3676,-0.30904,0.56018,1.2315,1.0402,0.022121,-1.1334,-0.97927,-0.45162,-0.23972,1.0285,-0.38871,-0.85661,-0.62876,0.19121,-0.04515,0.69732,0.92816,-0.86802,0.73132,0.011596,-0.42891,0.32517,1.224,-1.6352,-2.1491,1.8664,-1.4616,0.52359,0.25229,-1.0135,-0.12972,0.89204,-0.95123,-0.1255,0.66742,0.82818,-0.7853,0.6145,0.45253,-0.64563,-0.96357,-1.7654,-0.68019,0.10926,0.37714,1.0328
};
#endif
/**
  * @brief  LED Flashing Task.
  * @param  None
  * @retval None
  */
void Audio_Task( void ) {
	__BRD_D4_GPIO_CLK();
	__BRD_D5_GPIO_CLK();
	GPIO_InitTypeDef  GPIO_InitStructure;

	GPIO_InitStructure.Pin = BRD_D4_PIN;				//Pin
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;		//Output Mode
	GPIO_InitStructure.Pull = GPIO_PULLDOWN;			//Enable Pull up, down or no pull resister
	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;			//Pin latency
	HAL_GPIO_Init(BRD_D4_GPIO_PORT, &GPIO_InitStructure);	//Initialise Pin

	//vTaskSuspend(NULL);
	for (;;) {
		struct frameResults results;
		if(xSemaphoreTake(processing_Semaphore, 1) == pdTRUE) {

			HAL_GPIO_WritePin(BRD_D4_GPIO_PORT, BRD_D4_PIN, 1);
			audioProcessFrame(ready_data1, ready_data2, &results);
			HAL_GPIO_WritePin(BRD_D4_GPIO_PORT, BRD_D4_PIN, 0);
			// Give this frame a number
			results.frameNo = frameNumber++;
			debug_printf("%d\n", results.maxBin);

			if(results.validFrame){
				//print_results(results);
				if(xQueueSendToBack(validDataQueue, (void *)&results, 1) == pdFALSE) {
					//debug_printf("validFrame queue is full\n");
				}
			}
		}
	}
}

/**
 * Grabs audio results from a queue and sends them once a threshold has been reached
 * Delay 1000
 * Priority
 */
void TX_Task( void ){
	char SSID[50];
	unsigned char data[500];
	unsigned char b64_data[500];
	int reading_count = 0, string_len = 0;

	int seq = 0;
	vTaskDelay(1000);
	ESP8622_init(115200);

	Wifi_setmode();

	#ifndef MASTERNODE
		sprintf(&(SSID[0]), "NUCLEOWSN%d", NODE_ID);
		Wifi_setAP(SSID,"password", 5, 0);

		Wifi_enserver();

		debug_printf("Connecting to MASTERNODE\n");
		Wifi_join("MASTERNODE", "");
		vTaskDelay(1000);
		Wifi_connectTCP("192.168.1.1", 8888);

		Wifi_senddata(0, "RG:[0]", 6);
		vTaskResume(AudioTaskHandle);
	#else
		Wifi_setAP("MASTERNODE","password", 5, 0);
		Wifi_set_AP_IP("192.168.1.1");
		Wifi_enserver();
	#endif

	struct frameResults results;

	GPIO_InitTypeDef  GPIO_InitStructure;

	GPIO_InitStructure.Pin = BRD_D5_PIN;				//Pin
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;		//Output Mode
	GPIO_InitStructure.Pull = GPIO_PULLDOWN;			//Enable Pull up, down or no pull resister
	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;			//Pin latency
	HAL_GPIO_Init(BRD_D5_GPIO_PORT, &GPIO_InitStructure);	//Initialise Pin

	for(;;) {
		if (xQueueReceive( validDataQueue, &results, 1)) {
			if(string_len < 350){ //65 bytes for an encoded packet + 5 buffer len = 4(n/3)
				serialize_results(&results, &(data[string_len]));
				string_len += 51;
				reading_count++;
			} else {
				int len = b64_encode(data, b64_data, string_len);

				uint8_t buffer[500];
				int data_len = sprintf(buffer, PACKETFORMAT, 0, NODE_ID, 5, b64_data);
				debug_printf("Sending data of length: %d\n", data_len);
				Wifi_senddata(0, buffer, data_len);
				HAL_GPIO_WritePin(BRD_D5_GPIO_PORT, BRD_D5_PIN, 1);
				HAL_GPIO_WritePin(BRD_D5_GPIO_PORT, BRD_D5_PIN, 0);

				//Clean up for next time
				reading_count = 0;
				string_len = 0;

				memset(data, 0, 500);
				memset(b64_data, 0, 500);
			}

		}
	}
}


void timer_interupt_init(){
	uint32_t PrescalerValue;
	TIM_MasterConfigTypeDef Tim3MasterConfigHandle;

	/* Timer 4 clock enable */
	__TIM3_CLK_ENABLE();

	/* Compute the prescaler value */
	//Set clock prescaler to 500kHz - SystemCoreClock is the system clock frequency.
	PrescalerValue = (uint16_t) ((SystemCoreClock/2)/500000);

	TIM_Init.Instance = TIM3;						//Enable Timer 2
	TIM_Init.Init.Period = 4;						//Set period count to be 1ms, so timer interrupt occurs every 1ms.
	TIM_Init.Init.Prescaler = PrescalerValue;		//Set presale value
	TIM_Init.Init.ClockDivision = 0;				//Set clock division
	TIM_Init.Init.RepetitionCounter = 0;			// Set Reload Value
	TIM_Init.Init.CounterMode = TIM_COUNTERMODE_UP;	//Set timer to count up.

	/* Initialise Timer */
	HAL_TIM_Base_Init(&TIM_Init);

	/* Set priority of Timer 2 update Interrupt [0 (HIGH priority) to 15(LOW priority)] */
	HAL_NVIC_SetPriority(TIM3_IRQn, 3, 0);
	NVIC_SetVector(TIM3_IRQn, (uint32_t)&tim2_irqhandler);

	Tim3MasterConfigHandle.MasterOutputTrigger	= TIM_TRGO_UPDATE;
	Tim3MasterConfigHandle.MasterSlaveMode 		= TIM_MASTERSLAVEMODE_ENABLE;
	HAL_TIMEx_MasterConfigSynchronization(&TIM_Init, &Tim3MasterConfigHandle);

	NVIC_EnableIRQ(TIM3_IRQn);

	/* Start Timer */
	HAL_TIM_Base_Start_IT(&TIM_Init);

#ifdef ADCDEBUG
		debug_printf("DEBUG: Timer init has completed\n");
	#endif
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
	NVIC_SetVector(DMA2_Stream0_IRQn, (uint32_t)&DMACompleteISR1);
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

	if(HAL_ADC_Start_DMA(&AdcHandle1, (uint32_t*)(&buffer1), 1) != HAL_OK){
		//Init failed
		BRD_LEDOn();
		debug_printf("ERROR");
		for(;;);
	}

	/* WE NEED TO DO THE SAME THING FOR THE SECOND ADC :'( DMA1 A1 */
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
	NVIC_SetVector(DMA2_Stream2_IRQn, (uint32_t)&DMACompleteISR2);
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

	if(HAL_ADC_Start_DMA(&AdcHandle2, (uint32_t*)(&buffer2), 1) != HAL_OK){
		//Init failed
		BRD_LEDOn();
		debug_printf("ERROR");
		for(;;);
	}

	#ifdef ADCDEBUG
		debug_printf("DEBUG: ADC/DMA Init has completed\n");
	#endif

}

float32_t average1, min1, max1;
void DMACompleteISR1( void ){
	//Data transfer is complete! Handle the interupt.
	HAL_DMA_IRQHandler(AdcHandle1.DMA_Handle);

	data1[sampleNo1 % 256] = (float32_t)buffer1;

	//TODO Move these if there are issues with speed
	if(buffer1 > max1)
		max1 = buffer1;
	if(buffer1 < min1)
		min1 = buffer1;
	//TODO Move these if there are issues with speed
	if(sampleNo1 == 256){
		int i = 0;
		float32_t median = (max1-min1) + min1;
		for(i = 0; i < 256; i++){
			ready_data1[i] = (data1[i]-median)/10;
		}
		sampleNo1 = 0;
		max1 = 0;
		min1 = 0;
		average1 = 0;
		xSemaphoreGiveFromISR(processing_Semaphore, NULL);
	}
	sampleNo1++;
}

float32_t average2, min2, max2;
void DMACompleteISR2( void ){
	//Data transfer is complete! Handle the interupt.
	HAL_DMA_IRQHandler(AdcHandle2.DMA_Handle);

	//TODO Move these if there are issues with speed
	if(buffer2 > max2)
		max2 = buffer2;
	if(buffer2 < min2)
		min2 = buffer2;
	//TODO Move these if there are issues with speed

	data2[sampleNo2 % 256] = (float32_t)buffer2;
	if(sampleNo2 == 256){
		int i = 0;
		float32_t median = (max2-min2) + min2;
		for(i = 0; i < 256; i++){
			ready_data2[255-i] = (data2[i]-median)/10;
		}
		sampleNo2 = 0;
		max2 = 0;
		min2 = 0;
		average2 = 0;
	}
	sampleNo2++;
}


//void Software_timer(){
//	uint16_t ledsync  = 0;
//	uint16_t node_Sync  = 0;
//	uint8_t flag  = 1;
//	uint8_t node_scan = 0;
//	uint8_t nodes_send = 0;
//
//	uint16_t node_period = 10000;
//	// time variable is
//	for (;;) {
//
//		ledsync = ((xTaskGetTickCount()+time_Offset) % 1000);
//		node_Sync = ((xTaskGetTickCount()+time_Offset) % node_period);
//
//		/*	led sync	*/
//		if (ledsync > (500 / portTICK_RATE_MS) && flag == 1) {
//			//debug_printf("IDLE Tick %d\n", xLastTx);
//			flag = 0;
//			BRD_LEDToggle();
//
//		}
//		if (ledsync < (10 / portTICK_RATE_MS) && flag == 0) {
//			flag = 1;
//		}
//
//		/*		node sync		*/
//
//		if (node_Sync > ((NODE_ID*3000) / portTICK_RATE_MS) && node_scan == 1) {
//			//debug_printf("IDLE Tick %d\n", xLastTx);
//			node_scan = 0;
//			node_Scan();
//
//		}
//		if (node_Sync < (10 / portTICK_RATE_MS) && node_scan == 0) {
//			node_scan = 1;
//		}
//
//		if (node_Sync % 2000 > 1980 && nodes_send == 1) {
//			//debug_printf("IDLE Tick %d\n", xLastTx);
//			nodes_send = 0;
//			node_Send();
//
//		}
//		if (node_Sync % 2000 < 20 && nodes_send == 0) {
//			nodes_send = 1;
//		}
//		vTaskDelay(1);
//	}
//}
//
//
//
//
//
//void node_Scan() {
//	if (esp_Semaphore != NULL) {
//		if( xSemaphoreTake( esp_Semaphore, ( TickType_t ) 10 ) == pdTRUE ) {
//			Wifi_listAPs();
//			 Access_Point* ap = (Access_Point*)get_AP("Wu-Tang LAN");
//			 if (ap != NULL) {
//				debug_printf("RSSI: %d Distance: %f\n", ap->RSSI, RSSItoDistance(ap->RSSI));
//			 }
//			 xSemaphoreGive(esp_Semaphore);
//		}
//	}
//}
//
//
//void node_Send() {
//
//}


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