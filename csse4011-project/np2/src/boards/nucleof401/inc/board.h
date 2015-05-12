/**
  ******************************************************************************
  * @file    board.h
  * @author  MDS
  * @date    26-February-2015
  * @brief   This file provides pin and peripheral definitions for the
  *          Netduino Plus 2.
  *
  *			NOTE: BRD refers to the Arduino board connectors
  *			NOTE: CN7 and CN10 refers to the Morpho connectors (CN7, CN10)
  *					(See Nucleo pinout)
  ******************************************************************************
  */ 
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __NUCLEOF401_H
#define __NUCLEOF401_H

#ifdef __cplusplus
 extern "C" {
#endif
                                              
/* Includes ------------------------------------------------------------------*/
 #include "stm32f4xx.h"

//NP2 ANALOG PIN Definitions
#define BRD_A0					0						//Mask
#define BRD_A0_PIN				GPIO_PIN_0				//Pin Index
#define BRD_A0_GPIO_PORT		GPIOA
#define __BRD_A0_GPIO_CLK()		__GPIOA_CLK_ENABLE()	//GPIO Port clock
#define BRD_A0_EXTI_IRQ			EXTI0_IRQn				//External Interrupt IRQ Index
#define BRD_A0_ADC_CHAN			ADC_CHANNEL_0			//NOTE only ADC 1,2 & 3 use

#define BRD_A1					1
#define BRD_A1_PIN				GPIO_PIN_1
#define BRD_A1_GPIO_PORT			GPIOA
#define __BRD_A1_GPIO_CLK()		__GPIOA_CLK_ENABLE()
#define BRD_A1_EXTI_IRQ				EXTI1_IRQn
#define BRD_A1_ADC_CHAN				ADC_CHANNEL_1			//NOTE only ADC 1,2 & 3 use


#define BRD_A2					4
#define BRD_A2_PIN				GPIO_PIN_4
#define BRD_A2_GPIO_PORT			GPIOA
#define __BRD_A2_GPIO_CLK()		__GPIOA_CLK_ENABLE()
#define BRD_A2_EXTI_IRQ				EXTI4_IRQn
#define BRD_A2_ADC_CHAN				ADC_CHANNEL_4			//NOTE only ADC 1,2 & 3 use

#define BRD_A3					0
#define BRD_A3_PIN				GPIO_PIN_0
#define BRD_A3_GPIO_PORT			GPIOB
#define __BRD_A3_GPIO_CLK()		__GPIOB_CLK_ENABLE()
#define BRD_A3_EXTI_IRQ				EXTI0_IRQn
#define BRD_A3_ADC_CHAN				ADC_CHANNEL_8			//NOTE only ADC 1,2 & 3 use

#define BRD_A4					1
#define BRD_A4_PIN				GPIO_PIN_1
#define BRD_A4_GPIO_PORT 			GPIOC
#define __BRD_A4_GPIO_CLK()		__GPIOC_CLK_ENABLE()
#define BRD_A4_EXTI_IRQ				EXTI1_IRQn
#define BRD_A4_ADC_CHAN				ADC_CHANNEL_11			//NOTE only ADC 1 & 2 use

#define BRD_A5					0
#define BRD_A5_PIN				GPIO_PIN_0
#define BRD_A5_GPIO_PORT			GPIOC
#define __BRD_A5_GPIO_CLK()		__GPIOC_CLK_ENABLE()
#define BRD_A5_EXTI_IRQ				EXTI0_IRQn
#define BRD_A5_ADC_CHAN				ADC_CHANNEL_10			//NOTE only ADC 1 & 2 use


//NP2 DIGITAL PIN Definitions
#define BRD_D0					3						//Mask
#define BRD_D0_PIN				GPIO_PIN_3				//Pin INdex
#define BRD_D0_GPIO_PORT			GPIOA					//GPIO Port for Pin
#define __BRD_D0_GPIO_CLK()			__GPIOA_CLK_ENABLE()	//GPIO Port Clock
#define BRD_D0_EXTI_IRQ				EXTI3_IRQn			//External Interrupt Index	

#define BRD_D1					2
#define BRD_D1_PIN				GPIO_PIN_2
#define BRD_D1_GPIO_PORT			GPIOA
#define __BRD_D1_GPIO_CLK()			__GPIOA_CLK_ENABLE()
#define BRD_D1_EXTI_IRQ				EXTI2_IRQn

#define BRD_D2					10
#define BRD_D2_PIN				GPIO_PIN_10
#define BRD_D2_GPIO_PORT			GPIOA
#define __BRD_D2_GPIO_CLK()			__GPIOA_CLK_ENABLE()
#define BRD_D2_EXTI_IRQ				EXTI15_10_IRQn

#define BRD_D3					3
#define BRD_D3_PIN				GPIO_PIN_3
#define BRD_D3_GPIO_PORT			GPIOB
#define __BRD_D3_GPIO_CLK()			__GPIOB_CLK_ENABLE()
#define BRD_D3_EXTI_IRQ				EXTI3_IRQn

#define BRD_D4						5
#define BRD_D4_PIN					GPIO_PIN_5
#define BRD_D4_GPIO_PORT 			GPIOB
#define __BRD_D4_GPIO_CLK()			__GPIOB_CLK_ENABLE()
#define BRD_D4_EXTI_IRQ				EXTI9_5_IRQn

#define BRD_D5						4
#define BRD_D5_PIN					GPIO_PIN_4
#define BRD_D5_GPIO_PORT			GPIOB
#define __BRD_D5_GPIO_CLK()			__GPIOB_CLK_ENABLE()
#define BRD_D5_EXTI_IRQ				EXTI9_5_IRQn

#define BRD_D6						10
#define BRD_D6_PIN					GPIO_PIN_10
#define BRD_D6_GPIO_PORT			GPIOB
#define __BRD_D6_GPIO_CLK()			__GPIOB_CLK_ENABLE()
#define BRD_D6_EXTI_IRQ				EXTI9_5_IRQn

#define BRD_D7						8
#define BRD_D7_PIN					GPIO_PIN_8
#define BRD_D7_GPIO_PORT 			GPIOA
#define __BRD_D7_GPIO_CLK()			__GPIOA_CLK_ENABLE()
#define BRD_D7_EXTI_IRQ				EXTI1_IRQn

#define BRD_D8						9
#define BRD_D8_PIN					GPIO_PIN_9
#define BRD_D8_GPIO_PORT 			GPIOA
#define __BRD_D8_GPIO_CLK()			__GPIOA_CLK_ENABLE()
#define BRD_D8_EXTI_IRQ				EXTI9_5_IRQn

#define BRD_D9						7
#define BRD_D9_PIN					GPIO_PIN_7
#define BRD_D9_GPIO_PORT 			GPIOC
#define __BRD_D9_GPIO_CLK()			__GPIOC_CLK_ENABLE()
#define BRD_D9_EXTI_IRQ				EXTI9_5_IRQn

#define BRD_D10						6
#define BRD_D10_PIN					GPIO_PIN_6
#define BRD_D10_GPIO_PORT			GPIOB
#define __BRD_D10_GPIO_CLK()			__GPIOB_CLK_ENABLE()
#define BRD_D10_EXTI_PORT			EXTI_PortSourceGPIOB
#define BRD_D10_EXTI_IRQ			EXTI15_10_IRQn

#define BRD_D11						7
#define BRD_D11_PIN					GPIO_PIN_7
#define BRD_D11_GPIO_PORT			GPIOA
#define __BRD_D11_GPIO_CLK()			__GPIOA_CLK_ENABLE()
#define BRD_D11_EXTI_IRQ			EXTI15_10_IRQn

#define BRD_D12						6
#define BRD_D12_PIN					GPIO_PIN_6
#define BRD_D12_GPIO_PORT			GPIOA
#define __BRD_D12_GPIO_CLK()			__GPIOA_CLK_ENABLE()
#define BRD_D12_EXTI_IRQ			EXTI9_5_IRQn

#define BRD_D13						5	
#define BRD_D13_PIN					GPIO_PIN_5
#define BRD_D13_GPIO_PORT			GPIOA
#define __BRD_D13_GPIO_CLK()		__GPIOA_CLK_ENABLE()
#define BRD_D13_EXTI_IRQ			EXTI9_5_IRQn

//Morpho CN7 and CN10 connectors (See Nucleo pinout for pin numbers)
#define CN10_26						15
#define CN10_26_PIN					GPIO_PIN_15
#define CN10_26_GPIO_PORT			GPIOB
#define __CN10_26_GPIO_CLK()		__GPIOB_CLK_ENABLE()
#define CN10_26_EXTI_IRQ			EXTI15_10_IRQn

#define CN10_28						14
#define CN10_28_PIN					GPIO_PIN_14
#define CN10_28_GPIO_PORT			GPIOB
#define __CN10_28_GPIO_CLK()		__GPIOB_CLK_ENABLE()
#define CN10_28_EXTI_IRQ			EXTI15_10_IRQn

#define CN10_30						13	
#define CN10_30_PIN					GPIO_PIN_13
#define CN10_30_GPIO_PORT			GPIOB
#define __CN10_30_GPIO_CLK()		__GPIOB_CLK_ENABLE()
#define CN10_30_EXTI_IRQ			EXTI15_10_IRQn

//NP2 I2C SDA/SCL PIN Definitions
#define BRD_I2C				I2C1
#define BRD_I2C_CLK         

#define BRD_SDA_PIN			GPIO_PIN_9
#define BRD_SDA_GPIO_PORT	GPIOB
#define BRD_SDA_AF          GPIO_AF_I2C1

#define BRD_SCL_PIN			GPIO_PIN_8
#define BRD_SCL_GPIO_PORT	GPIOB
#define BRD_SCL_AF          GPIO_AF_I2C1

//NP2 debug printf uart definitions
#define BRD_DEBUG_UART				USART2
#define BRD_DEBUG_UART_BAUDRATE		115200
#define __BRD_DEBUG_UART_CLK()		__USART2_CLK_ENABLE();
#define BRD_DEBUG_UART_AF			GPIO_AF7_USART2
#define BRD_DEBUG_UART_TX_PIN		GPIO_PIN_2
#define BRD_DEBUG_UART_TX_GPIO_PORT	GPIOA
#define __BRD_DEBUG_UART_TX_GPIO_CLK()   __GPIOA_CLK_ENABLE()

#define BRD_DEBUG_UART_RX_PIN		GPIO_PIN_3
#define BRD_DEBUG_UART_RX_GPIO_PORT	GPIOA
#define __BRD_DEBUG_UART_RX_GPIO_CLK()   __GPIOA_CLK_ENABLE()


//NP2 SPI definitions
#define BRD_SPI                       SPI1_BASE
#define __BRD_SPI_CLK()               __SPI1_CLK_ENABLE()

#define BRD_SPI_SCK_PIN               GPIO_PIN_5                 /* PC.13 */
#define BRD_SPI_SCK_GPIO_PORT         GPIOA                       /* GPIOB */
#define __BRD_SPI_SCK_GPIO_CLK()      __GPIOA_CLK_ENABLE()
#define BRD_SPI_SCK_AF                GPIO_AF5_SPI1

#define BRD_SPI_MISO_PIN              GPIO_PIN_6                 /* PC.14 */
#define BRD_SPI_MISO_GPIO_PORT        GPIOA                       /* GPIOB */
#define __BRD_SPI_MISO_GPIO_CLK()      __GPIOA_CLK_ENABLE()
#define BRD_SPI_MISO_AF               GPIO_AF5_SPI1

#define BRD_SPI_MOSI_PIN              GPIO_PIN_7                 /* PC.15 */
#define BRD_SPI_MOSI_GPIO_PORT        GPIOA                       /* GPIOB */
#define __BRD_SPI_MOSI_GPIO_CLK()      __GPIOA_CLK_ENABLE()
#define BRD_SPI_MOSI_AF               GPIO_AF5_SPI1

#define BRD_SPI_CS_PIN                GPIO_PIN_6                 /* PB.10 */
#define BRD_SPI_CS_GPIO_PORT          GPIOB                       /* GPIOB */
#define __BRD_SPI_CS_GPIO_CLK()      __GPIOB_CLK_ENABLE()

#define BRD_LED				10
#define BRD_LED_PIN                	GPIO_PIN_5	//12
#define BRD_LED_GPIO_PORT          	GPIOA		//D
#define __BRD_LED_GPIO_CLK()           	__GPIOA_CLK_ENABLE()	//D  
   
/**
 * @brief push-button
 */
#define BRD_PB_PIN                	GPIO_PIN_13
#define BRD_PB_GPIO_PORT          	GPIOC
#define BRD_PB_GPIO_CLK           	RCC_AHB1Periph_GPIOC
#define BRD_PB_EXTI_LINE          	EXTI_Line13
#define BRD_PB_EXTI_PORT   			EXTI_PortSourceGPIOC
#define BRD_PB_EXTI_SOURCE    		EXTI_PinSource13
#define BRD_PB_EXTI_IRQ          	EXTI15_10_IRQn
#define BRD_PB_EXTI_IRQ_HANDLER		EXTI15_10_IRQHandler 


//Temperature Sensor
#define BRD_TEMP_ADC_CHAN		ADC_Channel_18	//NOTE only ADC 1,2 & 3 use
#define BRD_VBATT_ADC_CHAN		ADC_Channel_18	//NOTE only ADC 1,2 & 3 use

void BRD_init();

/** @defgroup NETDUINOPLUS2_LOW_LEVEL_Exported_Functions
  * @{
  */
void BRD_LEDInit();
void BRD_LEDOn();
void BRD_LEDOff();
void BRD_LEDToggle();

#ifdef ENABLE_DEBUG_UART
void BRD_debuguart_putc(unsigned char c);
unsigned char BRD_debuguart_getc();
#endif
  
#ifdef __cplusplus
}
#endif

#endif /* __NETDUINOPLUS2_H */

