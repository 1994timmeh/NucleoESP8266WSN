/**
  ******************************************************************************
  * @file    ex12_hamming.c 
  * @author  MDS & KB
  * @date    27032015
  * @brief   Hamming encoder example.
  *			 Bytes received from the VCP are Hamming encoded and displayed.
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

/* Private function prototypes -----------------------------------------------*/
void Delay(__IO unsigned long nCount);
void Hardware_init();
uint16_t hamming_byte_encoder(uint8_t input);


/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
void main(void) {

	uint8_t RxByte;
	uint16_t CodedWord;

	BRD_init();	//Initalise NP2
	Hardware_init();	//Initalise hardware modules  	

	/* Main processing loop */
    while (1) {

		/* Get any characters received */
		if (RxByte = debug_getc()) {
		
			/* Hamming encode received character */
			CodedWord = hamming_byte_encoder(RxByte);

			debug_printf("Byte: %c HEX:%02x | Encoded 0x%04x\r\n", RxByte, RxByte, CodedWord);

		}
		
    	BRD_LEDToggle();	//Toggle LED on/off
    	Delay(0x7FFF00);	//Delay function
  	}
}


/**
  * Implement Hamming Code + parity checking
  * Hamming code is based on the following generator and parity check matrices
  * G = [ 1 0 0 0 | 0 1 1
  * 	  0 1 0 0 | 1 0 1
  *		  0 0 1 0 | 1 1 0
  *	      0 0 0 1 | 1 1 1];
  *
  * hence H =
  * [ 0 1 1 1 | 1 0 0
  *	  1 0 1 1 | 0 1 0
  *	  1 1 0 1 | 0 0 1];
  *
  * y = x * G, syn = H * y'
  *
  *
  * NOTE: !! is used to get 1 out of non zeros
  */
uint8_t hamming_hbyte_encoder(uint8_t in) {

	uint8_t d0, d1, d2, d3;
	uint8_t p0 = 0, h0, h1, h2;
	uint8_t z;
	uint8_t out;
	
	/* extract bits */
	d0 = !!(in & 0x1);
	d1 = !!(in & 0x2);
	d2 = !!(in & 0x4);
	d3 = !!(in & 0x8);
	
	/* calculate hamming parity bits */
	h0 = d1 ^ d2 ^ d3;
	h1 = d0 ^ d2 ^ d3;
	h2 = d0 ^ d1 ^ d3;
	
	/* generate out byte without parity bit P0 */
	out = (h0 << 4) | (h1 << 5) | (h2 << 6) |
		(d0 << 0) | (d1 << 1) | (d2 << 2) | (d3 << 3);

	/* calculate even parity bit */
	for (z = 0; z<7; z++)		
		p0 = p0 ^ !!(out & (1 << z));
	
	out |= (p0 << 7);	//Put P0 into most significatn bit.

	return(out);

}

/**
  * Implement Hamming Code on a full byte of input
  * This means that 16-bits out output is needed
  */
uint16_t hamming_byte_encoder(uint8_t input) {

	uint16_t out;
	
	/* first encode D0..D3 (first 4 bits), 
	 * then D4..D7 (second 4 bits).
	 */
	out = hamming_hbyte_encoder(input & 0xF) | 
		(hamming_hbyte_encoder(input >> 4) << 8);
	
	return(out);

}

/**
  * @brief  Initialise Hardware 
  * @param  None
  * @retval None
  */
void Hardware_init() {

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


