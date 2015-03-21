/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "debug_printf.h"
#include "ESP8622.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SEC 0x7FFF00
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void Delay(__IO unsigned long nCount);
void Hardware_init();

void main(void) {

	BRD_LEDToggle();

	Hardware_init();	//Initalise hardware modules
	Wifi_reset(); 		//Reset the module to stop any cross over when debugging
	Wifi_setmode();
	Wifi_listAPs();
	Wifi_join();

	BRD_LEDToggle();

  while (1) {
		Wifi_checkcon();
		BRD_LEDToggle();	//Toggle 'Alive' LED on/off
		Delay(SEC*5);	//Delay function
	}
}

void Hardware_init() {
	BRD_init();			//Initalise NP2 board.
	ESP8622_init();

	BRD_LEDInit();		//Initialise Blue LED
	BRD_LEDOff();		//Turn off Blue LED

	debug_printf("DEBUG_PRINTF TEST\n\r");
}

void Delay(__IO unsigned long nCount) {
  	while(nCount--) {
  	}
}
