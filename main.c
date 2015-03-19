/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "debug_printf.h"
#include "ESP8622.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void Delay(__IO unsigned long nCount);
void Hardware_init();

void main(void) {

	Hardware_init();	//Initalise hardware modules
	/* Main processing loop */
    while (1) {
			ESP8622_send_test();
			BRD_LEDToggle();	//Toggle 'Alive' LED on/off
			Delay(0x7FFF00);	//Delay function
  	}
}

void Hardware_init() {
	BRD_init();			//Initalise NP2 board.
	ESP8622_init();

	BRD_LEDInit();		//Initialise Blue LED
	BRD_LEDOff();		//Turn off Blue LED
}

void Delay(__IO unsigned long nCount) {
  	while(nCount--) {
  	}
}
