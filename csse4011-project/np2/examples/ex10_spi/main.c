/**
  ******************************************************************************
  * @file    ex10_spi/main.c 
  * @author  MDS
  * @date    03022015
  * @brief   SPI Read 32-bit Register nrf24l01plus status (0x07) register
  *			 NOTE: This example does not send or transmit with the nrf24l01plus.
  *			 REFERENCE: nrf24l01p_datasheet.pdf
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
static SPI_HandleTypeDef SpiHandle;

/* Private function prototypes -----------------------------------------------*/
void Delay(__IO unsigned long nCount);
void HardwareInit();
uint8_t spi_sendbyte(uint8_t sendbyte);

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void) {

	uint8_t status;
	
	BRD_init();
	HardwareInit();

	/* Main Processing Loop */
    while (1) {

		HAL_GPIO_WritePin(BRD_SPI_CS_GPIO_PORT, BRD_SPI_CS_PIN, 0);	//Set Chip Select low

		spi_sendbyte(0x07);										//Send status register address
	
		status = spi_sendbyte(0xFF);							//Send dummy byte, to read status register values			
		debug_printf("nrf24L01 Status Register Value: %X ", status);	//See page 59 of nrf24l01plus datasheet for status register definition
																//NOTE: default value of the status register is 0xE0		
		debug_printf("\n\r");
		HAL_GPIO_WritePin(BRD_SPI_CS_GPIO_PORT, BRD_SPI_CS_PIN, 1);		//Set Chip Select high	

    	BRD_LEDToggle();	//Toggle LED on/off
    	Delay(0x7FFF00);	//Delay function
  	}

}

/**
  * @brief  Initialise hardware modules
  * @param  None
  * @retval None
  */
void HardwareInit() {
	
	GPIO_InitTypeDef GPIO_spi;	

	BRD_LEDInit();		//Initialise Blue LED
	BRD_LEDOff();		//Turn off Blue LED

	/* Set SPI clock */
	__BRD_SPI_CLK();

	/* Enable GPIO Pin clocks */
	__BRD_SPI_SCK_GPIO_CLK();
	__BRD_SPI_MISO_GPIO_CLK();
	__BRD_SPI_MOSI_GPIO_CLK();
	__BRD_SPI_CS_GPIO_CLK();
	
	/* Initialise SPI and Pin clocks*/
	/* SPI SCK pin configuration */
  	GPIO_spi.Pin = BRD_SPI_SCK_PIN;
  	GPIO_spi.Mode = GPIO_MODE_AF_PP;
  	GPIO_spi.Speed = GPIO_SPEED_HIGH;
	GPIO_spi.Pull = GPIO_PULLDOWN;
	GPIO_spi.Alternate = BRD_SPI_SCK_AF;
  	HAL_GPIO_Init(BRD_SPI_SCK_GPIO_PORT, &GPIO_spi);

  	/* SPI MISO pin configuration */
  	GPIO_spi.Pin = BRD_SPI_MISO_PIN;
	GPIO_spi.Mode = GPIO_MODE_AF_PP;
	GPIO_spi.Speed = GPIO_SPEED_FAST;
	GPIO_spi.Pull = GPIO_PULLUP;		//Must be set as pull up
	GPIO_spi.Alternate = BRD_SPI_MISO_AF;
  	HAL_GPIO_Init(BRD_SPI_MISO_GPIO_PORT, &GPIO_spi);

	/* SPI  MOSI pin configuration */
	GPIO_spi.Pin =  BRD_SPI_MOSI_PIN;
	GPIO_spi.Mode = GPIO_MODE_AF_PP;
	GPIO_spi.Pull = GPIO_PULLDOWN;
	GPIO_spi.Alternate = BRD_SPI_MOSI_AF;
  	HAL_GPIO_Init(BRD_SPI_MOSI_GPIO_PORT, &GPIO_spi);

	//Set SPI Module
	SpiHandle.Instance = (SPI_TypeDef *)(BRD_SPI);

	 __HAL_SPI_DISABLE(&SpiHandle);

	/* SPI to Master and 16 baudrate prescaler */
    SpiHandle.Init.Mode              = SPI_MODE_MASTER;
    SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    SpiHandle.Init.Direction         = SPI_DIRECTION_2LINES;
    SpiHandle.Init.CLKPhase          = SPI_PHASE_1EDGE;
    SpiHandle.Init.CLKPolarity       = SPI_POLARITY_LOW;
    SpiHandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLED;
    SpiHandle.Init.CRCPolynomial     = 0;
    SpiHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
    SpiHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    SpiHandle.Init.NSS               = SPI_NSS_SOFT;
    SpiHandle.Init.TIMode            = SPI_TIMODE_DISABLED;

	/* Initailise SPI parameters */
    HAL_SPI_Init(&SpiHandle);

	/* Enable SPI */
   __HAL_SPI_ENABLE(&SpiHandle);


	/* Configure GPIO PIN for SPI Chip select, TFT CS, TFT DC */
  	GPIO_spi.Pin = BRD_SPI_CS_PIN;				//Pin
  	GPIO_spi.Mode = GPIO_MODE_OUTPUT_PP;		//Output Mode
  	GPIO_spi.Pull = GPIO_PULLUP;			//Enable Pull up, down or no pull resister
  	GPIO_spi.Speed = GPIO_SPEED_FAST;			//Pin latency
  	HAL_GPIO_Init(BRD_SPI_CS_GPIO_PORT, &GPIO_spi);	//Initialise Pin

	/* Set chip select high */
	HAL_GPIO_WritePin(BRD_SPI_CS_GPIO_PORT, BRD_SPI_CS_PIN, 1);
}

/**
  * @brief  Send byte through SPI.
  * @param  sendbyte: byte to be transmitted via SPI.
  * @retval None
  */
uint8_t spi_sendbyte(uint8_t sendbyte) {

	uint8_t readbyte;

	HAL_SPI_TransmitReceive(&SpiHandle, &sendbyte, &readbyte, 1, 10);

	// Return the Byte read from the SPI bus 
	return readbyte;
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

