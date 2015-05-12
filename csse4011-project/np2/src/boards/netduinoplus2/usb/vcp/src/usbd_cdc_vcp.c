/**
  ******************************************************************************
  * @file    usbd_cdc_vcp.c
  * @author  MDS
  * @date    01022015
  * @brief   USB CDC Virtual COM Port Implementation
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_vcp.h"


/* These are external variables imported from CDC core to be used for IN 
   transfer management. */

static int VCP_Rx_BufferLen = 0;
static int VCP_Rx_BufferGetLen = 0;


static int8_t VCP_Init     (void);
static int8_t VCP_DeInit   (void);
static int8_t VCP_Control  (uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t VCP_Receive  (uint8_t* pbuf, uint32_t *Len);
uint16_t VCP_Transmit (uint8_t* Buf, uint32_t Len);

static uint16_t VCP_COMConfig(uint8_t Conf);

USBD_CDC_ItfTypeDef USBD_CDC_VCP_fops = 
{
  VCP_Init,
  VCP_DeInit,
  VCP_Control,
  VCP_Receive
};

USBD_CDC_LineCodingTypeDef linecoding =
  {
    115200, /* baud rate*/
    0x00,   /* stop bits-1*/
    0x00,   /* parity - none*/
    0x08    /* nb. of bits 8*/
  };

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  vDelay Function.
  * @param  nCount:specifies the Delay time length.
  * @retval None
  */
void vDelay(__IO unsigned long nCount) {
  while(nCount--) {
  }
}


/**
  * @brief  VCP_Init
  *         Initializes the CDC media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t VCP_Init(void)
{
  /*
     Add your initialization code here 
  */  

	//USBD_CDC_SetTxBuffer(&hUSBDDevice, VCP_Tx_Buffer, APP_Rx_ptr_in);
	USBD_CDC_SetRxBuffer(&hUSBDDevice, VCP_Rx_Buffer);
	USBD_CDC_ReceivePacket(&hUSBDDevice);


  return (0);
}

/**
  * @brief  VCP_DeInit
  *         DeInitializes the CDC media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t VCP_DeInit(void)
{
  /*
     Add your deinitialization code here 
  */  
  return (0);
}


/**
  * @brief  VCP_Control
  *         Manage the CDC class requests
  * @param  Cmd: Command code            
  * @param  Buf: Buffer containing command data (request parameters)
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t VCP_Control  (uint8_t cmd, uint8_t* pbuf, uint16_t length)
{ 
  switch (cmd)
  {
  case CDC_SEND_ENCAPSULATED_COMMAND:
    /* Add your code here */
    break;

  case CDC_GET_ENCAPSULATED_RESPONSE:
    /* Add your code here */
    break;

  case CDC_SET_COMM_FEATURE:
    /* Add your code here */
    break;

  case CDC_GET_COMM_FEATURE:
    /* Add your code here */
    break;

  case CDC_CLEAR_COMM_FEATURE:
    /* Add your code here */
    break;

  case CDC_SET_LINE_CODING:
    linecoding.bitrate    = (uint32_t)(pbuf[0] | (pbuf[1] << 8) |\
                            (pbuf[2] << 16) | (pbuf[3] << 24));
    linecoding.format     = pbuf[4];
    linecoding.paritytype = pbuf[5];
    linecoding.datatype   = pbuf[6];
    
    /* Add your code here */
	VCP_COMConfig(OTHER_CONFIG);
    break;

  case CDC_GET_LINE_CODING:
    pbuf[0] = (uint8_t)(linecoding.bitrate);
    pbuf[1] = (uint8_t)(linecoding.bitrate >> 8);
    pbuf[2] = (uint8_t)(linecoding.bitrate >> 16);
    pbuf[3] = (uint8_t)(linecoding.bitrate >> 24);
    pbuf[4] = linecoding.format;
    pbuf[5] = linecoding.paritytype;
    pbuf[6] = linecoding.datatype;     
    
    /* Add your code here */
    break;

  case CDC_SET_CONTROL_LINE_STATE:
    /* Add your code here */
    break;

  case CDC_SEND_BREAK:
     /* Add your code here */
    break;    
    
  default:
    break;
  }

  return (0);
}

/**
  * @brief  VCP_Transmit
  *         Insert characters in to the transmit buffer - does not actually
  *			transmit. Must call VCP_txflush to transmit.
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the opeartion: USBD_OK if all operations are OK else VCP_FAIL
  */
uint16_t VCP_Transmit (uint8_t* Buf, uint32_t Len)
{

	uint32_t i;

	//memset(VCP_Tx_Buffer, 0, sizeof(VCP_Tx_Buffer));
	//strcpy(VCP_Tx_Buffer, "1234567890\n\r");
	//APP_Rx_ptr_in = 12;

	for (i = 0; i < Len; i++) {
    
		VCP_Tx_Buffer[APP_Rx_ptr_in] = *(Buf + i);
		
		if (APP_Rx_ptr_in < VCP_RX_BUFFER_SIZE) {
			APP_Rx_ptr_in++;
		}
 	}

	
  return USBD_OK;
}

/**
  * @brief  VCP_txflush
  *         Insert a single character into the tx buffer.
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
uint16_t VCP_putc(char c)
{
	
	VCP_Transmit((uint8_t *)&c, 1);
	
	return USBD_OK;

}


/**
  * @brief  VCP_txflush
  *         Transmit and flush tx buffer.
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
void VCP_txflush ()
{
	int i;

	USBD_CDC_SetTxBuffer(&hUSBDDevice, VCP_Tx_Buffer, APP_Rx_ptr_in);
	USBD_CDC_TransmitPacket(&hUSBDDevice);
	
	APP_Rx_ptr_in = 0;

	for (i = 0; i < VCP_RX_BUFFER_SIZE; i++) {
		VCP_Tx_Buffer[i] = 0;
	}

	//delay for 50us.
	//vDelay(0x4000);
	//USBD_CDC_ReceivePacket(&hUSBDDevice);

}

/**
  * @brief  VCP_Receive
  *         Data received over USB OUT endpoint are sent over CDC interface 
  *         through this function.
  *           
  *         @note
  *         This function will issue a NAK packet on any OUT packet received on 
  *         USB endpoint untill exiting this function. If you exit this function
  *         before transfer is complete on CDC interface (ie. using DMA controller)
  *         it will result in receiving more data while previous ones are still 
  *         not sent.
  *                 
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t VCP_Receive (uint8_t* Buf, uint32_t *Len)
{

 uint32_t i;
	
	
	for (i = 0; i < *Len; i++) {
		VCP_Rx_Buffer[VCP_Rx_BufferLen] = *(Buf + i);
		VCP_Rx_BufferLen = (VCP_Rx_BufferLen +1)%VCP_RX_BUFFER_SIZE;
	} 

	//USBD_CDC_SetRxBuffer(&hUSBDDevice, VCP_Rx_Buffer);
	//USBD_CDC_ReceivePacket(&hUSBDDevice);

	

  	return USBD_OK;
}

/**
  * @brief  VCP_getchar
  *         receive a single character.
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
int VCP_getc(uint8_t *rxbyte)
{

	USBD_CDC_SetRxBuffer(&hUSBDDevice, VCP_Rx_Buffer);
	USBD_CDC_ReceivePacket(&hUSBDDevice);

	//Check for boundary condition on rx ring buffer.	
	if (VCP_Rx_BufferGetLen == VCP_Rx_BufferLen) {
        // No data. 
		vDelay(0x4000);        
		return 0;
    } else  {
        *rxbyte = VCP_Rx_Buffer[VCP_Rx_BufferGetLen];
        VCP_Rx_BufferGetLen = (VCP_Rx_BufferGetLen + 1)%VCP_RX_BUFFER_SIZE;

		VCP_rxflush();

		//vDelay(0x4000);

        return 1;
    }
	
	return 0;

}

/**
  * @brief  VCP_txflush
  *         Flush rx buffer.
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
void VCP_rxflush(void)
{

	memset(VCP_Rx_Buffer, 0, sizeof(VCP_Rx_Buffer));

	VCP_Rx_BufferLen = 0;
	VCP_Rx_BufferGetLen = 0;
}
 
/**
  * @brief  VCP_COMConfig
  *         Configure the COM Port with default values or values received from host.
  * @param  Conf: can be DEFAULT_CONFIG to set the default configuration or OTHER_CONFIG
  *         to set a configuration received from the host.
  * @retval None.
  */
static uint16_t VCP_COMConfig(uint8_t Conf)
{


  return USBD_OK;
}

