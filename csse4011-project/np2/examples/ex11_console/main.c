/**
  ******************************************************************************
  * @file    ex11_console/main.c 
  * @author  MDS
  * @date    02022015
  * @brief   Receive byte from serial console and print back to console using
  *		     debug printf or debug putc.
  *			 Use kermusb to start the kermit console.
  *			 Comment or uncomment PRINTF_REFLECT to choose either printf or putc.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "debug_printf.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void Hardware_init(void);
void Delay(__IO unsigned long nCount);

#define PRINTF_REFLECT			//Comment out to use putc otherwise printf.

/**
  * @brief  Main program - flashes onboard blue LED
  * @param  None
  * @retval None
  */
void main(void) {

	char RxChar;

	BRD_init();			//Initalise NP2 board
	Hardware_init();	//Initalise hardware modules
  	
	/* Main processing loop */
    while (1) {

		/* Receive characters using getc */
		RxChar = debug_getc();

		/* Check if character is not Null */
		if (RxChar != '\0') {

#ifdef PRINTF_REFLECT
			debug_printf("%c", RxChar);		//reflect byte using printf - must delay before calling printf again.
#else
			debug_putc(RxChar);				//reflect byte using putc - puts character into buffer
			debug_flush();					//Must call flush, to send character
#endif
		}

		BRD_LEDToggle();	//Toggle 'Alive' LED on/off		
      	
		/* Insert delay */
    	Delay(0x7FFF00 >> 4);	//Delay 125ms.
	}

}

/**
  * @brief  Initialise Hardware 
  * @param  None
  * @retval None
  */
void Hardware_init(void) {

	BRD_LEDInit();	//Initialise Blue LED
	BRD_LEDOff();	//Turn off Blue LED
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

