/**
  ******************************************************************************
  * @file    nrf24l01plus.c
  * @author  MDS
  * @date    10-January-2014
  * @brief   nrf9051 SPI operations, initialisation and read/write functions
  ******************************************************************************
  *  
  */ 

#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "nrf24l01plus.h"
#include "debug_printf.h"

//#define DEBUG	1	//debug enable messages


uint8_t default_addr[] = {0x12, 0x34, 0x56, 0x78, 0x90};

static SPI_HandleTypeDef SpiHandle;

/**
  * @brief  rfDelay Function.
  * @param  nCount:specifies the Delay time length.
  * @retval None
  */
void rfDelay(__IO unsigned long nCount) {
  while(nCount--) {
  }
}


/**
  * @brief  NRF24l01Plus SPI Intialisation Function.
  * @param  None
  * @retval None
  */
void nrf24l01plus_spi_init() {

	GPIO_InitTypeDef GPIO_spi;	
	
	/* Initialise SPI and Pin clocks*/
	__BRD_SPI_CLK();	//__SPI1_CLK_ENABLE();

	__BRD_SPI_SCK_GPIO_CLK();
	__BRD_SPI_MISO_GPIO_CLK();
	__BRD_SPI_MOSI_GPIO_CLK();
	__NRF_SPI_CS_GPIO_CLK;
	__NRF_MODE_GPIO_CLK;
	
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
	
	SpiHandle.Instance = (SPI_TypeDef *)(BRD_SPI);

	 __HAL_SPI_DISABLE(&SpiHandle);

    SpiHandle.Init.Mode              = SPI_MODE_MASTER;
    SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16; //56;
    SpiHandle.Init.Direction         = SPI_DIRECTION_2LINES;
    SpiHandle.Init.CLKPhase          = SPI_PHASE_1EDGE;
    SpiHandle.Init.CLKPolarity       = SPI_POLARITY_LOW;
    SpiHandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLED;
    SpiHandle.Init.CRCPolynomial     = 0; //7;
    SpiHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
    SpiHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    SpiHandle.Init.NSS               = SPI_NSS_SOFT;
    SpiHandle.Init.TIMode            = SPI_TIMODE_DISABLED;

    HAL_SPI_Init(&SpiHandle);

   __HAL_SPI_ENABLE(&SpiHandle);


	/* Configure GPIO PIN for SPI Chip select, TFT CS, TFT DC */
  	GPIO_spi.Pin = NRF_SPI_CS_PIN;				//Pin
  	GPIO_spi.Mode = GPIO_MODE_OUTPUT_PP;		//Output Mode
  	GPIO_spi.Pull = GPIO_PULLUP;			//Enable Pull up, down or no pull resister
  	GPIO_spi.Speed = GPIO_SPEED_FAST;			//Pin latency
  	HAL_GPIO_Init(NRF_SPI_CS_GPIO_PORT, &GPIO_spi);	//Initialise Pin
  	
	/* Configure GPIO PIN for RX/TX mode */
  	GPIO_spi.Pin = NRF_MODE_PIN;
  	GPIO_spi.Mode = GPIO_MODE_OUTPUT_PP;		//Output Mode
  	GPIO_spi.Pull = GPIO_PULLUP;			//Enable Pull up, down or no pull resister
  	GPIO_spi.Speed = GPIO_SPEED_FAST;			//Pin latency
  	HAL_GPIO_Init(NRF_MODE_GPIO_PORT, &GPIO_spi);	//Initialise Pin

	/* Set chip select high */
	NRF_CS_HIGH();
}

/**
  * @brief  nrf24L01plus SPI Send/Receive Byte Function.
  * @param  Byte to be transmitted.
  * @retval Returns byte received.
  */
uint8_t nrf24l01plus_spi_SendRecv_Byte(uint8_t byte) {

	uint8_t rxbyte;

	// *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout);
	HAL_SPI_TransmitReceive(&SpiHandle, &byte, &rxbyte, 1, 1);

	rfDelay(0x100);	
		
	return rxbyte; 
}

/**
  * @brief  NRF24l01Plus Write buffer Function. Write buffer to register.
  * @param  register address, buffer, buffer length
  * @retval None
  */
void nrf24l01plus_WriteBuffer(uint8_t reg_addr, uint8_t *buffer, int buffer_len) {

	int i;

	NRF_CS_LOW();

	rfDelay(0x8FF);

	nrf24l01plus_spi_SendRecv_Byte(NRF24L01P_WRITE_REG | reg_addr);

	rfDelay(0x8FF);

#ifdef DEBUG
	debug_printf("DEBUG: WB: ");
#endif
	for (i = 0; i < buffer_len; i++) {
		
		/* Return the Byte read from the SPI bus */
		nrf24l01plus_spi_SendRecv_Byte(buffer[i]);
		rfDelay(0x100);	
#ifdef DEBUG
		debug_printf("%X ", buffer[i]);
#endif
	}

#ifdef DEBUG
	debug_printf("\n\r");
#endif

	rfDelay(0x8FF);
	NRF_CS_HIGH();
	rfDelay(0x8FF);	
}

/**
  * @brief  NRF24l01Plus Read buffer Function. Read buffer from register.
  * @param  register address, buffer, buffer length
  * @retval None
  */
void nrf24l01plus_ReadBuffer(uint8_t reg_addr, uint8_t *buffer, int buffer_len) {

	int i;
	
	NRF_CS_LOW();

	nrf24l01plus_spi_SendRecv_Byte(reg_addr);

	rfDelay(0x8FF);

#ifdef DEBUG
	debug_printf("DEBUG:RB ");
#endif
	for (i = 0; i < buffer_len; i++) {
		
		/* Return the Byte read from the SPI bus */
		buffer[i] = nrf24l01plus_spi_SendRecv_Byte(0xFF);

#ifdef DEBUG
		debug_printf("%X ", buffer[i]);
#endif

		rfDelay(0x8FF);
	}

#ifdef DEBUG
	debug_printf("\n\r");
#endif

	rfDelay(0x8FF);
	NRF_CS_HIGH();
	rfDelay(0x8FF);		 
}

/**
  * @brief  NRF24l01Plus Write Register Function. Write value to register.
  * @param  register address, value
  * @retval None
  */
void nrf24l01plus_WriteRegister(uint8_t reg_addr, uint8_t val) {

	NRF_CS_LOW();

	nrf24l01plus_spi_SendRecv_Byte(NRF24L01P_WRITE_REG | reg_addr);
	nrf24l01plus_spi_SendRecv_Byte(val);

	NRF_CS_HIGH(); 
}

/**
  * @brief  NRF24l01Plus Read Register Function. Read register.
  * @param  register address, buffer, buffer length
  * @retval register value
  */
uint8_t nrf24l01plus_ReadRegister(uint8_t reg_addr) {

	uint8_t rxbyte;

	NRF_CS_LOW();

	rxbyte = nrf24l01plus_spi_SendRecv_Byte(reg_addr);
	rxbyte = nrf24l01plus_spi_SendRecv_Byte(0xFF);

	NRF_CS_HIGH();
		
	return rxbyte; 
}


/**
  * @brief  NRF24l01Plus Initalise Function. Initialise registers of NRF24l01Plus.
  * @param  None
  * @retval None
  */
void nrf24l01plus_init() {

	nrf24l01plus_spi_init();

	/* Set CE low for idle state */
	NRF_CE_LOW();
	
	nrf24l01plus_WriteBuffer(NRF24L01P_WRITE_REG | NRF24L01P_TX_ADDR, default_addr, 5);		// Writes TX_Address to nRF24L01
	nrf24l01plus_WriteBuffer(NRF24L01P_WRITE_REG | NRF24L01P_RX_ADDR_P0, default_addr, 5);	//NRF24L01P_TX_ADR_WIDTH);

    nrf24l01plus_WriteRegister(NRF24L01P_EN_AA, 0x00);      							// Disable Auto.Ack
    nrf24l01plus_WriteRegister(NRF24L01P_EN_RXADDR, 0x01);  							// Enable Pipe0
    nrf24l01plus_WriteRegister(NRF24L01P_RX_PW_P0, NRF24L01P_TX_PLOAD_WIDTH); 			// Select same RX payload width as TX Payload width

    //SPI_Write_Reg(WRITE_REG + SETUP_RETR, 0x1a);       // 500us + 86us, 10 retransmissions. (not needed - no auto ack)

    nrf24l01plus_WriteRegister(NRF24L01P_RF_CH, NRF24L01P_RF_CHANNEL_DEFAULT);        	// Select RF channel
    nrf24l01plus_WriteRegister(NRF24L01P_RF_SETUP, 0x06);   							// TX_PWR:0dBm, Datarate:1Mbps
    nrf24l01plus_WriteRegister(NRF24L01P_CONFIG, 0x02);	     							// Set PWR_UP bit, enable CRC(2 unsigned chars) & Prim:TX. MAX_RT & TX_DS enabled..
}

/**
  * @brief  NRF24l01Plus send packet Function.
  * @param  None
  * @retval None
  */
void nrf24l01plus_send_packet(uint8_t *tx_buf) {

    nrf24l01plus_WriteRegister(NRF24L01P_CONFIG, 0x72);     // Set PWR_UP bit, enable CRC(2 unsigned chars) & Prim:TX.
    //nrf24l01plus_WriteRegister(NRF24L01P_FLUSH_TX, 0);                                  
    nrf24l01plus_WriteBuffer(NRF24L01P_WR_TX_PLOAD, tx_buf, NRF24L01P_TX_PLOAD_WIDTH);   // write playload to TX_FIFO

	/* Generate 10us pulse on CE pin for transmission */
	rfDelay(0x40);//rfDelay(0x100);
	NRF_CE_LOW();
	rfDelay(0x40);//rfDelay(0x100);
    NRF_CE_HIGH();	//Set CE pin low to enable TX mode
	rfDelay(0x40*3);	//rfDelay(0x100);
	NRF_CE_LOW(); 
}


/**
  * @brief  NRF24l01Plus mode rx. Put NRF24l01Plus in receive mode.
  * @param  None
  * @retval None
  */
void nrf24l01plus_mode_rx(void) {

    nrf24l01plus_WriteRegister(NRF24L01P_CONFIG, 0x73);	//0x0f     	// Set PWR_UP bit, enable CRC(2 unsigned chars) & Prim:RX. 
    NRF_CE_HIGH();                             		// Set CE pin high to enable RX device
}


/**
  * @brief  NRF24l01Plus rxFifoEmpy Function. Check NRF24l01Plus rx fifo.
  * @param  None
  * @retval None
  */
int nrf24l01plus_rxFifoEmpty(void) {

    unsigned char fifoStatus;

    fifoStatus = nrf24l01plus_ReadRegister(NRF24L01P_FIFO_STATUS);

#ifdef DEBUG
	debug_printf("DEBUG: fifoStatus: %X\n\r", fifoStatus);
#endif

    return (fifoStatus & NRF24L01P_FIFO_RX_EMPTY);

}


/**
  * @brief  NRF24l01Plus receive packet Function.  
  *			Attempts to receive a packet and puts it into rx_buf. Returns 1 on successful receive.
  * @param  None
  * @retval None
  */ 
int nrf24l01plus_receive_packet(uint8_t *rx_buf) {

    int rec = 0;
    unsigned char status = nrf24l01plus_ReadRegister(NRF24L01P_STATUS);                  // read register STATUS's value

#ifdef DEBUG
	debug_printf("DEBUG:RCV packet status: %X\n\r", status);
#endif

	
    //if((status & NRF24L01P_RX_DR) && !nrf24l01plus_rxFifoEmpty()) {    // if receive data ready interrupt and FIFO full.
	if(status & NRF24L01P_RX_DR) {
		rfDelay(0x100);
        nrf24l01plus_ReadBuffer(NRF24L01P_RD_RX_PLOAD, rx_buf, NRF24L01P_TX_PLOAD_WIDTH);  // read playload to rx_buf
        nrf24l01plus_WriteRegister(NRF24L01P_FLUSH_RX,0);                             // clear RX_FIFO
        rec = 1;

		NRF_CE_LOW();
		rfDelay(0x100);

		nrf24l01plus_WriteRegister(NRF24L01P_STATUS, status);                  // clear RX_DR or TX_DS or MAX_RT interrupt flag
    }  

    return rec;
}

