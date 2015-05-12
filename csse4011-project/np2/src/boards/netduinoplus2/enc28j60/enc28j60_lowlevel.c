// modified for the ENC28J60 Rev. B7 Silicon Errata
// M.D'S
// modified for lwIP usage
//
// File Name	: 'enc28j60_lowlevel.c'
// Title		: Microchip ENC28J60 Ethernet Interface Driver
// Author		: Pascal Stang (c)2005
// Created		: 9/22/2005
// Revised		: 13/1/2014
// Version		: 0.1
// Target MCU	: STM32F4xx series
// Editor Tabs	: 4
//
// Description	: This driver provides initialization and transmit/receive
//	functions for the Microchip ENC28J60 10Mb Ethernet Controller and PHY.
// This chip is novel in that it is a full MAC+PHY interface all in a 28-pin
// chip, using an SPI interface to the host processor.
//

/************************************************************
*
*	This is the general driver for the ENC28J60
*	I changed some things to make it work with uIP
*	Some files of uIP have changes too.
*
*								edi87 [at] fibertel.com.ar
*								Jonathan Granade
*
************************************************************/

#include "arch/cc.h"
#include "lwipopts.h"
//#include <stdio.h>
//#include <stdint.h>
#include "enc28j60_lowlevel.h"
#include "enc28j60def.h"

#include "board.h"
#include "stm32f4xx_hal_conf.h"

//#define DEBUG	1	//Enable debug messages
#ifdef DEBUG
#include "debug_printf.h"
#endif

GPIO_InitTypeDef GPIO_InitStructure;
SPI_HandleTypeDef SpiHandle;

//#define EXT_ENC	1		//Enable external ENC28J60

//Set pin definitions
#ifdef EXT_ENC
	#define ENC_SPI					BRD_SPI						//External SPI
	#define ENC_SPI_CLK				BRD_SPI_CLK
  	
	#define ENC_SPI_SCK_GPIO_CLK	BRD_SPI_SCK_GPIO_CLK		//GPIO SPI SCK Clock
	#define ENC_SPI_MISO_GPIO_CLK	BRD_SPI_MISO_GPIO_CLK		//GPIO SPI SCK Clock
	#define ENC_SPI_CS_GPIO_CLK		BRD_D8_GPIO_CLK

	#define ENC_SPI_SCK_PIN			BRD_SPI_SCK_PIN
	#define ENC_SPI_SCK_GPIO_PORT	BRD_SPI_SCK_GPIO_PORT
	#define ENC_SPI_SCK_AF			BRD_SPI_SCK_AF

	#define ENC_SPI_MISO_PIN		BRD_SPI_MISO_PIN
  	#define ENC_SPI_MISO_GPIO_PORT	BRD_SPI_MISO_GPIO_PORT
	#define ENC_SPI_MISO_AF			BRD_SPI_MISO_AF

	#define ENC_SPI_MOSI_PIN		BRD_SPI_MOSI_PIN
  	#define ENC_SPI_MOSI_GPIO_PORT	BRD_SPI_MOSI_GPIO_PORT
	#define ENC_SPI_MOSI_AF			BRD_SPI_MOSI_AF  
    
	#define ENC_SPI_CS_PIN			BRD_D8_PIN
  	#define ENC_SPI_CS_GPIO_PORT	BRD_D8_GPIO_PORT

#else
	#define ENC_SPI					BRD_ENC_SPI						//onboard ethernet SPI
	#define ENC_SPI_CLK				BRD_ENC_SPI_CLK
  	
	#define __ENC_SPI_SCK_GPIO_CLK()	__BRD_ENC_SPI_SCK_GPIO_CLK()		//GPIO SPI SCK Clock
	#define __ENC_SPI_MISO_GPIO_CLK()	__BRD_ENC_SPI_MISO_GPIO_CLK()		//GPIO SPI SCK Clock
	#define __ENC_SPI_CS_GPIO_CLK()		__BRD_ENC_SPI_CS_GPIO_CLK()

	#define ENC_SPI_SCK_PIN			BRD_ENC_SPI_SCK_PIN
	#define ENC_SPI_SCK_GPIO_PORT	BRD_ENC_SPI_SCK_GPIO_PORT
	#define ENC_SPI_SCK_AF			BRD_ENC_SPI_SCK_AF

	#define ENC_SPI_MISO_PIN		BRD_ENC_SPI_MISO_PIN
  	#define ENC_SPI_MISO_GPIO_PORT	BRD_ENC_SPI_MISO_GPIO_PORT
	#define ENC_SPI_MISO_AF			BRD_ENC_SPI_MISO_AF

	#define ENC_SPI_MOSI_PIN		BRD_ENC_SPI_MOSI_PIN
  	#define ENC_SPI_MOSI_GPIO_PORT	BRD_ENC_SPI_MOSI_GPIO_PORT
	#define ENC_SPI_MOSI_AF			BRD_ENC_SPI_MOSI_AF  
    
	#define ENC_SPI_CS_PIN			BRD_ENC_SPI_CS_PIN
  	#define ENC_SPI_CS_GPIO_PORT	BRD_ENC_SPI_CS_GPIO_PORT

#endif


// set CS to 0 = active
#define CS_LOW() HAL_GPIO_WritePin(ENC_SPI_CS_GPIO_PORT, ENC_SPI_CS_PIN, 0);	//GPIO_ResetBits(ENC_SPI_CS_GPIO_PORT, ENC_SPI_CS_PIN)
// set CS to 1 = passive
#define CS_HIGH() HAL_GPIO_WritePin(ENC_SPI_CS_GPIO_PORT, ENC_SPI_CS_PIN, 1);	//GPIO_SetBits(ENC_SPI_CS_GPIO_PORT, ENC_SPI_CS_PIN)



u8_t Enc28j60Bank;
u16_t NextPacketPtr;


//Delay function
void DelayUs(int delay)
{
  	while(delay--)
  	{
		__asm volatile( "nop" );
  	}
}


u8_t enc_sendbyte(unsigned char txbyte) {

	u8_t rxbyte;

#ifdef DEBUG
	debug_printf("ENCSPI DEBUG: sendbyte: T-");
#endif		

	HAL_SPI_TransmitReceive(&SpiHandle, &txbyte, &rxbyte, 1, 1);	


	DelayUs(100);
	DelayUs(300);
	DelayUs(100);
	
#ifdef DEBUG
	//debug_printf("%X R-%X\n\r", txbyte, rxbyte);
#endif		

	return rxbyte;
}

u8_t enc28j60ReadOp(u8_t op, u8_t address)
{
	u8_t data;
   
	// assert CS
	CS_LOW();
	
	// issue read command
	enc_sendbyte( op | (address & ADDR_MASK));
	
	// read data
	data = enc_sendbyte(0x00);
	
	// do dummy read if needed
	if(address & 0x80)
	{
		data = enc_sendbyte(0x00);
	}
	
	// release CS
	CS_HIGH();

	return data;
}

void enc28j60WriteOp(u8_t op, u8_t address, u8_t data)
{
	// assert CS
	CS_LOW();

	// issue write command
	enc_sendbyte(op | (address & ADDR_MASK));
	
	// write data
	enc_sendbyte(data);

	// release CS
	CS_HIGH();
}

void enc28j60ReadBuffer(u16_t len, u8_t* data)
{
	// assert CS
	CS_LOW();
	
	// issue read command
	enc_sendbyte(ENC28J60_READ_BUF_MEM);
	
	while(len--)
	{
		// read data
		*data++ = enc_sendbyte(0x00);
	}	

	// release CS
	CS_HIGH();
}

void enc28j60WriteBuffer(u16_t len, u8_t* data)
{
	// assert CS
	CS_LOW();
	
	// issue write command
	enc_sendbyte(ENC28J60_WRITE_BUF_MEM);
	while(len--)
	{
		// write data
		enc_sendbyte(*data++);
	}	
	// release CS
	CS_HIGH();
}

void enc28j60SetBank(u8_t address)
{
	// set the bank (if needed)
	if((address & BANK_MASK) != Enc28j60Bank)
	{
		// set the bank
		enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, ECON1, (ECON1_BSEL1|ECON1_BSEL0));
		enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, (address & BANK_MASK)>>5);
		Enc28j60Bank = (address & BANK_MASK);
	}
}

u8_t enc28j60Read(u8_t address)
{
	// set the bank
	enc28j60SetBank(address);
	// do the read
	return enc28j60ReadOp(ENC28J60_READ_CTRL_REG, address);
}

u16_t enc28j60Read16(u8_t address)
{
  u16_t data=enc28j60Read(address);
  data|=enc28j60Read(address+1)<<8;
  return data;
}

void enc28j60Write(u8_t address, u8_t data)
{
	// set the bank
	enc28j60SetBank(address);
	// do the write
	enc28j60WriteOp(ENC28J60_WRITE_CTRL_REG, address, data);
}

void enc28j60Write16(u8_t address, u16_t data)
{
  enc28j60Write(address,data&0xff);
  enc28j60Write(address+1,data>>8);
}


u16_t enc28j60PhyRead(u8_t address)
{
	u16_t data;

	// Set the right address and start the register read operation
	enc28j60Write(MIREGADR, address);
	enc28j60Write(MICMD, MICMD_MIIRD);

	// wait until the PHY read completes
	while(enc28j60Read(MISTAT) & MISTAT_BUSY);

	// quit reading
	enc28j60Write(MICMD, 0x00);
	
	// get data value
	data  = enc28j60Read(MIRDL);
  data<<=8;
	data |= enc28j60Read(MIRDH);
	// return the data
	return data;
}

void enc28j60PhyWrite(u8_t address, u16_t data)
{
	// set the PHY register address
	enc28j60Write(MIREGADR, address);
	
	// write the PHY data
	enc28j60Write(MIWRL, data);	
	enc28j60Write(MIWRH, data>>8);

	// wait until the PHY write completes
	while(enc28j60Read(MISTAT) & MISTAT_BUSY);
}


void enc28j60SoftwareReset(void)
{

	// perform system reset
	enc28j60WriteOp(ENC28J60_SOFT_RESET, 0, ENC28J60_SOFT_RESET);
  
  //#ifdef ENC28J60_ERRATA_B7
  DelayUs(1000); //wait for 1 ms
  //#else
	// check CLKRDY bit to see if reset is complete
	//int i;
	//for(i=0;i<0x10;i++){
	//	for(j=0;j<0xff;j++) ;
	//}
	//while(!(enc28j60Read(ESTAT) & ESTAT_CLKRDY));
  //#endif //ENC28J60_ERRATA_B7
}


void enc28j60Init(u8_t *eth_addr,u8_t DuplexState)
{
	// initialize I/O
	  
#ifdef DEBUG
	debug_printf("ENCSPI DEBUG: Enc28j60Init %2X-%2X-%2X-%2X-%2X-%2X \n\r", eth_addr[0], eth_addr[1], eth_addr[2], eth_addr[3], eth_addr[4], eth_addr[5]);
#endif
	
/* Enable SCK, MOSI and MISO GPIO clocks */
	__BRD_ENC_SPI_CLK();
	__BRD_ENC_SPI_SCK_GPIO_CLK();
	__BRD_ENC_SPI_MISO_GPIO_CLK();
	__BRD_ENC_SPI_MOSI_GPIO_CLK();
	__BRD_ENC_SPI_CS_GPIO_CLK();

	/* SPI SCK pin configuration */	
	GPIO_InitStructure.Pin = BRD_ENC_SPI_SCK_PIN;
  	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
  	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStructure.Pull = GPIO_PULLDOWN;
	GPIO_InitStructure.Alternate = BRD_ENC_SPI_SCK_AF;
  	HAL_GPIO_Init(BRD_ENC_SPI_SCK_GPIO_PORT, &GPIO_InitStructure);

  	/* SPI MISO pin configuration */
	GPIO_InitStructure.Pin = BRD_ENC_SPI_MISO_PIN;
  	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
  	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	GPIO_InitStructure.Alternate = BRD_ENC_SPI_SCK_AF;
  	HAL_GPIO_Init(BRD_ENC_SPI_MISO_GPIO_PORT, &GPIO_InitStructure);
	

	/* SPI  MOSI pin configuration */
	GPIO_InitStructure.Pin = BRD_ENC_SPI_MOSI_PIN;
  	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
  	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStructure.Pull = GPIO_PULLDOWN;
	GPIO_InitStructure.Alternate = BRD_ENC_SPI_MOSI_AF;
  	HAL_GPIO_Init(BRD_ENC_SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);

  	/* SPI configuration -------------------------------------------------------*/
  	SpiHandle.Instance = (SPI_TypeDef *)(BRD_ENC_SPI);

	 __HAL_SPI_DISABLE(&SpiHandle);

    SpiHandle.Init.Mode              = SPI_MODE_MASTER;
    SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4; //64; //56;
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

  	/* Configure GPIO PIN for Enc Chip select */
	GPIO_InitStructure.Pin = BRD_ENC_SPI_CS_PIN;
  	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStructure.Pull = GPIO_PULLDOWN;
  	HAL_GPIO_Init(BRD_ENC_SPI_CS_GPIO_PORT, &GPIO_InitStructure);
  	
  	/* Deselect : Chip Select high */
	HAL_GPIO_WritePin(BRD_ENC_SPI_CS_GPIO_PORT, BRD_ENC_SPI_CS_PIN, GPIO_PIN_SET);
	
	/* Configure GPIO PIN for Power */
	GPIO_InitStructure.Pin = BRD_ENC_PWR_PIN;
  	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
  	HAL_GPIO_Init(BRD_ENC_PWR_GPIO_PORT, &GPIO_InitStructure);
  	
  	
	/* Turn Power off and on */	
	HAL_GPIO_WritePin(BRD_ENC_PWR_GPIO_PORT, BRD_ENC_PWR_PIN, GPIO_PIN_RESET);	

	/* Configure GPIO PIN for Reset */
	GPIO_InitStructure.Pin = BRD_ENC_RST_PIN;
  	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
  	HAL_GPIO_Init(BRD_ENC_RST_GPIO_PORT, &GPIO_InitStructure);
  	
	/* Turn Reset on */	
	HAL_GPIO_WritePin(BRD_ENC_RST_GPIO_PORT, BRD_ENC_RST_PIN, GPIO_PIN_RESET);	
	DelayUs(100);	
	HAL_GPIO_WritePin(BRD_ENC_RST_GPIO_PORT, BRD_ENC_RST_PIN, GPIO_PIN_SET);
	
  	DelayUs(100);  	


  enc28j60SoftwareReset();
  
	// do bank 0 stuff
	// initialize receive buffer
	// 16-bit transfers, must write low byte first
	// set receive buffer start address
	NextPacketPtr = ENC28J60_RXSTART_INIT;
    
	enc28j60Write16(ERXSTL, ENC28J60_RXSTART_INIT);
	// set receive pointer address
	enc28j60Write16(ERXRDPTL, ENC28J60_RXSTART_INIT);
	// set receive buffer end
	enc28j60Write16(ERXNDL, ENC28J60_RXSTOP_INIT);
	// set transmit buffer start
	enc28j60Write16(ETXSTL, ENC28J60_TXSTART_INIT);
  
  // allow Unicast to US and Broad cast packets with correct CRC
  enc28j60Write(ERXFCON, ERXFCON_CRCEN|ERXFCON_UCEN|ERXFCON_BCEN);//ERXFCON_UCEN||ERXFCON_PMEN

	// do bank 2 stuff
	// enable MAC receive
	enc28j60Write(MACON1, MACON1_MARXEN|MACON1_TXPAUS|MACON1_RXPAUS);
	// bring MAC out of reset
	enc28j60Write(MACON2, 0x00);
	// enable automatic padding and CRC operations
	enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, MACON3, MACON3_PADCFG0 | MACON3_TXCRCEN | MACON3_FRMLNEN);
	// enc28j60Write(MACON3, MACON3_PADCFG0|MACON3_TXCRCEN|MACON3_FRMLNEN);
	// set inter-frame gap (non-back-to-back)
	enc28j60Write(MAIPGL, 0x12);
	enc28j60Write(MAIPGH, 0x0C);//?
	// set inter-frame gap (back-to-back)
	enc28j60Write(MABBIPG, 0x12);
	// Set the maximum packet size which the controller will accept
	enc28j60Write16(MAMXFLL, ENC28J60_MAX_FRAMELEN);

	// do bank 3 stuff
	// write MAC address
	// NOTE: MAC address in ENC28J60 is byte-backward
	enc28j60Write(MAADR5, eth_addr[0]);
	enc28j60Write(MAADR4, eth_addr[1]);
	enc28j60Write(MAADR3, eth_addr[2]);
	enc28j60Write(MAADR2, eth_addr[3]);
	enc28j60Write(MAADR1, eth_addr[4]);
	enc28j60Write(MAADR0, eth_addr[5]);

	// no loopback of transmitted frames
	enc28j60PhyWrite(PHCON2, PHCON2_HDLDIS);
  
  // 0x476 is PHLCON LEDA=links status, LEDB=receive/transmit
  enc28j60PhyWrite(PHLCON,0x476);

	// switch to bank 0
	// enable interrutps
	enc28j60Write(EIE, EIE_INTIE|EIE_PKTIE);
	// enable packet reception
	enc28j60Write(ECON1, ECON1_RXEN);

	//enc28j60PhyWrite(PHLCON, 0x0AA2);

	// setup duplex ----------------------

	// Disable receive logic and abort any packets currently being transmitted
	enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_TXRTS|ECON1_RXEN);
  
	if(DuplexState)
	{
		u16_t temp;
		// Set the PHY to the proper duplex mode
		temp = enc28j60PhyRead(PHCON1);
		temp &= ~PHCON1_PDPXMD;
		enc28j60PhyWrite(PHCON1, temp);
		// Set the MAC to the proper duplex mode
		temp = enc28j60Read(MACON3);
		temp &= ~MACON3_FULDPX;
		enc28j60Write(MACON3, temp);
	}

	// Set the back-to-back inter-packet gap time to IEEE specified 
	// requirements.  The meaning of the MABBIPG value changes with the duplex
	// state, so it must be updated in this function.
	// In full duplex, 0x15 represents 9.6us; 0x12 is 9.6us in half duplex
	enc28j60Write(MABBIPG, DuplexState ? 0x15 : 0x12);	
	
	// Reenable receive logic
	enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_RXEN);

	// setup duplex ----------------------
}

// read the revision of the chip:
u8_t enc28j60getrev(void)
{
#ifdef DEBUG
	debug_printf("ENCSPI DEBUG: Enc28j60getrev\n\r");
#endif

  return(enc28j60Read(EREVID));
}

// link status
u8_t enc28j60linkup(void)
{
  // bit 10 (= bit 3 in upper reg)
  return (enc28j60PhyRead(PHSTAT2) && (1<<10))?1:0;
}

//! start sending packet
//TODO: make an internal counter to detect if we didn't send enough data?
void enc28j60BeginPacketSend(u16_t packetLength)
{
	//Errata: Transmit Logic reset
  #ifdef ENC28J60_ERRATA_B7
	enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_TXRST);
	enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_TXRST);
  enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, EIR, EIR_TXERIF);
  #endif //ENC28J60_ERRATA_B7

	// Set the write pointer to start of transmit buffer area
	enc28j60Write16(EWRPTL, ENC28J60_TXSTART_INIT);
	// Set the TXND pointer to correspond to the packet size given
	enc28j60Write16(ETXNDL, (ENC28J60_TXSTART_INIT+packetLength));
	
	// write per-packet control byte
	enc28j60WriteOp(ENC28J60_WRITE_BUF_MEM, 0, 0x00);


}

//! transfer a portion of data to output buffer
//TODO: make an internal counter to detect if we didn't send enough data?
void enc28j60PacketSend(u8_t *portion,u16_t length)
{
	// copy the packet into the transmit buffer
	enc28j60WriteBuffer(length, portion);
}

//! finish sending packet
//TODO: make an internal counter to detect if we didn't send enough data?
void enc28j60EndPacketSend(void)
{
	// send the contents of the transmit buffer onto the network
	enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_TXRTS);
}


u8_t enc28j60PollPacketSending(void)
{
	return enc28j60Read(ECON1) & ECON1_TXRTS;
}

u16_t enc28j60BeginPacketReceive(void)
{
	u16_t rxstat;
	u16_t len;

#ifdef DEBUG
	debug_printf("ENCSPI DEBUG: enc28j60BeginPacketReceive ");
#endif		

	// check if a packet has been received and buffered
  #ifdef ENC28J60_ERRATA_B7
  if( enc28j60Read(EPKTCNT) ==0 )
    return 0;
  #else
	if( !(enc28j60Read(EIR) & EIR_PKTIF) )
		return 0;
	#endif //ENC28J60_ERRATA_B7
  
	// Make absolutely certain that any previous packet was discarded	
	//if( WasDiscarded == FALSE)
	//	MACDiscardRx();

	// Set the read pointer to the start of the received packet
	enc28j60Write16(ERDPTL, NextPacketPtr);
  
	// read the next packet pointer
	NextPacketPtr  = enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0);
	NextPacketPtr |= enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0)<<8;
	// read the packet length
	len  = enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0);
	len |= enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0)<<8;
	// read the receive status
	rxstat  = enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0);
	rxstat |= enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0)<<8;

	// (we reduce the MAC-reported length by 4 to remove the CRC)
  len-=4;
  
//  limit retrieve length
//	if(len>maxlen) len=maxlen;
//	if(len<=0) return 0;
//	else return len;
  if ((rxstat & 0x80)==0){
    // invalid
    len=0;
  }

#ifdef DEBUG
	debug_printf("%d\n\r", len);
#endif

	return len;
}

void enc28j60PacketReceive(u8_t *portion,u16_t length)
{
#ifdef DEBUG
	debug_printf("ENCSPI DEBUG: enc28j60PacketReceive\n\r");
#endif		


	// copy the packet from the receive buffer
	enc28j60ReadBuffer(length, portion);
}

void enc28j60EndPacketReceive(void)
{

#ifdef DEBUG
	debug_printf("ENCSPI DEBUG: enc28j60EndPacketReceive\n\r");
#endif		



#ifdef ENC28J60_ERRATA_B7
  if( ((NextPacketPtr - 1) < ENC28J60_RXSTART_INIT) || ((NextPacketPtr - 1) > ENC28J60_RXSTOP_INIT))
   enc28j60Write16(ERXRDPTL, ENC28J60_RXSTOP_INIT);
  else
   enc28j60Write16(ERXRDPTL, NextPacketPtr-1);
#endif //ENC28J60_ERRATA_B7  
	// Move the RX read pointer to the start of the next received packet
	// This frees the memory we just read out
	// decrement the packet counter indicate we are done with this packet
	enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON2, ECON2_PKTDEC);
}

void enc28j60ReceiveOverflowRecover(void)
{
	// receive buffer overflow handling procedure

	// recovery completed
}


void enc28j60RegDump(void)
{
#ifdef DEBUGc
  debug_printf(PSTR("RevID: 0x%x\r\n"), enc28j60Read(EREVID));

  debug_printf("Cntrl: ECON1 ECON2 ESTAT  EIR  EIE\r\n");
  debug_printf("       0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\r\n",
    enc28j60Read(ECON1),
    enc28j60Read(ECON2),
    enc28j60Read(ESTAT),
    enc28j60Read(EIR),
    enc28j60Read(EIE));

  debug_printf("PHY  : PHCON1  PHSTAT1 PHSTAT2 PHID1 PHID2 PHCON2 PHIE PHIR PHLCON\r\n");
  debug_printf("       %04x    %04x    %04x    %04x  %04x   %04x  %04x %04x %04x\r\n",
     enc28j60PhyRead(PHCON1),
     enc28j60PhyRead(PHSTAT1),
     enc28j60PhyRead(PHSTAT2),
     enc28j60PhyRead(PHHID1),
     enc28j60PhyRead(PHHID2),
     enc28j60PhyRead(PHCON2),
     enc28j60PhyRead(PHIE),
     enc28j60PhyRead(PHIR),
     enc28j60PhyRead(PHLCON));

  debug_printf("MAC  : MACON1  MACON3  MACON4  MAC-Address\r\n");
  debug_printf("       0x%02x  0x%02x 0x%02x   %02x:%02x:%02x:%02x:%02x:%02x\r\n",
     enc28j60Read(MACON1),
     enc28j60Read(MACON3),
     enc28j60Read(MACON4),
     enc28j60Read(MAADR5),
     enc28j60Read(MAADR4),
     enc28j60Read(MAADR3),
     enc28j60Read(MAADR2),
     enc28j60Read(MAADR1),
     enc28j60Read(MAADR0));

  debug_printf("Rx   : ERXST  ERXND  ERXWRPT ERXRDPT ERXFCON EPKTCNT MAMXFL\r\n");
  debug_printf("       0x%04x 0x%04x 0x%04x   0x%04x 0x%02x 0x%02x 0x%04x\r\n",
     enc28j60Read16(ERXSTL),
     enc28j60Read16(ERXNDL),
     enc28j60Read16(ERXWRPTL),
     enc28j60Read16(ERXRDPTL),
     enc28j60Read(ERXFCON),
     enc28j60Read(EPKTCNT),
     enc28j60Read16(MAMXFLL));

  debug_printf("Tx   : ETXST  ETXND  MACLCON1 MACLCON2 MAPHSUP\r\n");
  debug_printf("       0x%04x 0x%04x 0x%04x   0x%02x   0x%02x\r\n",
           enc28j60Read16(ETXNDL),
     (enc28j60Read(MACLCON1)<<8)|enc28j60Read(ETXNDL));
#endif//DEBUG   */  
}
