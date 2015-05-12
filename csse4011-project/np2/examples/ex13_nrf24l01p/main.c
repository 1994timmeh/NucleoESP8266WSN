/**
  ******************************************************************************
  * @file    ex12_nrf24l01p/main.c
  * @author  MDS
  * @date    04022015
  * @brief   Send or Receive 32 byte packets using the nrf9051plus radio transciever
  *			 See the nrf9051plus datasheet.
  ******************************************************************************
  *  
  */ 

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "debug_printf.h"
#include "nrf24l01plus.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define TXMODE	1		//Put program in transmit or receive mode.

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t packetbuffer[32];	/* Packet buffer initialised to 32 bytes (max length) */

/* Private function prototypes -----------------------------------------------*/
void Delay(__IO unsigned long nCount);
void HardwareInit();

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void) {

	int i;

	BRD_init();
	HardwareInit();
	
	/* Initialise NRF24l01plus */ 
	nrf24l01plus_init();
	
	/* Put NRF24l01plus in RX mode */
	nrf24l01plus_mode_rx();

    while (1) {		

/* Transmit Mode */
#ifdef TXMODE

		/* Fill packet with 'dummy' data */
		for (i = 0; i < 32; i++) {
			packetbuffer[i] = '0'+ (i%10);
		}

		debug_printf("sending...\n\r");

		/* Send packet */
		nrf24l01plus_send_packet(packetbuffer);

/* Receive Mode */
#else		
		
		/* Check for received packet and print if packet is received */
		if (nrf24l01plus_receive_packet(packetbuffer) == 1) {

			debug_printf("Received: ");
			for (i = 0; i < 32; i++ ) {
				debug_printf("%x ", packetbuffer[i]);
			}
			debug_printf("\n\r");
		}
#endif

		Delay(0x04FF00);			//Must Delay for 10us before switching back to RX mode.
		nrf24l01plus_mode_rx();

    	Delay(0x7FFF00);	//Delay for 1s.
		BRD_LEDToggle();
  	}

}


/**
  * @brief Hardware Initialisation Function.
  * @param  None
  * @retval None
  */
void HardwareInit() {

	BRD_LEDInit();		//Initialise Blue LED
	BRD_LEDOff();		//Turn off Blue LED

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

