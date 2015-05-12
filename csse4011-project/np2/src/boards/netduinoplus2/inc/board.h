/**
  ******************************************************************************
  * @file    board.h
  * @author  MDS
  * @date    30012015
  * @brief   This file provides pin and peripheral definitions for the
  *          Netduino Plus 2.
  ******************************************************************************
  */ 
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BOARD_H
#define __BOARD_H

#ifdef __cplusplus
 extern "C" {
#endif
                                              
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f405xx.h"

//NP2 ANALOG PIN Definitions
#define BRD_A0						0						//Mask
#define BRD_A0_PIN					GPIO_PIN_0				//Pin Index
#define BRD_A0_GPIO_PORT			GPIOC					//GPIO Port for pin
#define __BRD_A0_GPIO_CLK()			__GPIOC_CLK_ENABLE()	//GPIO Port clock
#define BRD_A0_EXTI_IRQ				EXTI0_IRQn				//External Interrupt IRQ Index
#define BRD_A0_ADC_CHAN				ADC_CHANNEL_10			//NOTE only ADC 1,2 & 3 use

#define BRD_A1						1
#define BRD_A1_PIN					GPIO_PIN_1
#define BRD_A1_GPIO_PORT			GPIOC
#define __BRD_A1_GPIO_CLK()			__GPIOC_CLK_ENABLE()
#define BRD_A1_EXTI_IRQ				EXTI1_IRQn
#define BRD_A1_ADC_CHAN				ADC_CHANNEL_11			//NOTE only ADC 1,2 & 3 use


#define BRD_A2						2
#define BRD_A2_PIN					GPIO_PIN_2
#define BRD_A2_GPIO_PORT			GPIOC
#define __BRD_A2_GPIO_CLK()			__GPIOC_CLK_ENABLE()
#define BRD_A2_EXTI_IRQ				EXTI2_IRQn
#define BRD_A2_ADC_CHAN				ADC_CHANNEL_12			//NOTE only ADC 1,2 & 3 use

#define BRD_A3						3
#define BRD_A3_PIN					GPIO_PIN_3
#define BRD_A3_GPIO_PORT			GPIOC
#define __BRD_A3_GPIO_CLK()			__GPIOC_CLK_ENABLE()
#define BRD_A3_EXTI_IRQ				EXTI3_IRQn
#define BRD_A3_ADC_CHAN				ADC_CHANNEL_13			//NOTE only ADC 1,2 & 3 use

#define BRD_A4						4
#define BRD_A4_PIN					GPIO_PIN_4
#define BRD_A4_GPIO_PORT 			GPIOC
#define __BRD_A4_GPIO_CLK()			__GPIOC_CLK_ENABLE()
#define BRD_A4_EXTI_IRQ				EXTI4_IRQn
#define BRD_A4_ADC_CHAN				ADC_CHANNEL_14			//NOTE only ADC 1 & 2 use

#define BRD_A5						5
#define BRD_A5_PIN					GPIO_PIN_5
#define BRD_A5_GPIO_PORT			GPIOC
#define __BRD_A5_GPIO_CLK()			__GPIOC_CLK_ENABLE()
#define BRD_A5_EXTI_IRQ				EXTI9_5_IRQn
#define BRD_A5_ADC_CHAN				ADC_CHANNEL_15			//NOTE only ADC 1 & 2 use


//NP2 DIGITAL PIN Definitions
#define BRD_D0						7						//Mask
#define BRD_D0_PIN					GPIO_PIN_7				//Pin INdex
#define BRD_D0_GPIO_PORT			GPIOC					//GPIO Port for Pin
#define __BRD_D0_GPIO_CLK()			__GPIOC_CLK_ENABLE()	//GPIO Port Clock
#define BRD_D0_EXTI_IRQ				EXTI9_5_IRQn			//External Interrupt Index	

#define BRD_D1						6
#define BRD_D1_PIN					GPIO_PIN_6
#define BRD_D1_GPIO_PORT			GPIOC
#define __BRD_D1_GPIO_CLK()			__GPIOC_CLK_ENABLE()
#define BRD_D1_EXTI_IRQ				EXTI9_5_IRQn

#define BRD_D2						3
#define BRD_D2_PIN					GPIO_PIN_3
#define BRD_D2_GPIO_PORT			GPIOA
#define __BRD_D2_GPIO_CLK()			__GPIOA_CLK_ENABLE()
#define BRD_D2_EXTI_IRQ				EXTI3_IRQn

#define BRD_D3						2
#define BRD_D3_PIN					GPIO_PIN_2
#define BRD_D3_GPIO_PORT			GPIOA
#define __BRD_D3_GPIO_CLK()			__GPIOA_CLK_ENABLE()
#define BRD_D3_EXTI_IRQ				EXTI2_IRQn

#define BRD_D4						12
#define BRD_D4_PIN					GPIO_PIN_12
#define BRD_D4_GPIO_PORT 			GPIOB
#define __BRD_D4_GPIO_CLK()			__GPIOB_CLK_ENABLE()
#define BRD_D4_EXTI_IRQ				EXTI15_10_IRQn

#define BRD_D5						8
#define BRD_D5_PIN					GPIO_PIN_8
#define BRD_D5_GPIO_PORT			GPIOB
#define __BRD_D5_GPIO_CLK()			__GPIOB_CLK_ENABLE()
#define BRD_D5_EXTI_IRQ				EXTI9_5_IRQn

#define BRD_D6						9
#define BRD_D6_PIN					GPIO_PIN_9	
#define BRD_D6_GPIO_PORT			GPIOB
#define __BRD_D6_GPIO_CLK()			__GPIOB_CLK_ENABLE()
#define BRD_D6_EXTI_IRQ				EXTI9_5_IRQn

#define BRD_D7						1
#define BRD_D7_PIN					GPIO_PIN_1
#define BRD_D7_GPIO_PORT 			GPIOA
#define __BRD_D7_GPIO_CLK()			__GPIOA_CLK_ENABLE()
#define BRD_D7_EXTI_IRQ				EXTI1_IRQn

#define BRD_D8						0
#define BRD_D8_PIN					GPIO_PIN_0
#define BRD_D8_GPIO_PORT 			GPIOA
#define __BRD_D8_GPIO_CLK()			__GPIOA_CLK_ENABLE()
#define BRD_D8_EXTI_IRQ				EXTI0_IRQn

#define BRD_D9						6
#define BRD_D9_PIN					GPIO_PIN_6
#define BRD_D9_GPIO_PORT 			GPIOA
#define __BRD_D9_GPIO_CLK()			__GPIOA_CLK_ENABLE()
#define BRD_D9_EXTI_IRQ				EXTI9_5_IRQn

#define BRD_D10						10
#define BRD_D10_PIN					GPIO_PIN_10
#define BRD_D10_GPIO_PORT			GPIOB
#define __BRD_D10_GPIO_CLK()		__GPIOB_CLK_ENABLE()
#define BRD_D10_EXTI_IRQ			EXTI15_10_IRQn

#define BRD_D11						15
#define BRD_D11_PIN					GPIO_PIN_15
#define BRD_D11_PINSOURCE			GPIO_PinSource15
#define BRD_D11_GPIO_PORT			GPIOB
#define __BRD_D11_GPIO_CLK()		__GPIOB_CLK_ENABLE()
#define BRD_D11_EXTI_IRQ			EXTI15_10_IRQn

#define BRD_D12						14
#define BRD_D12_PIN					GPIO_PIN_14
#define BRD_D12_GPIO_PORT			GPIOB
#define __BRD_D12_GPIO_CLK()		__GPIOB_CLK_ENABLE()
#define BRD_D12_EXTI_IRQ			EXTI15_10_IRQn

#define BRD_D13						13	
#define BRD_D13_PIN					GPIO_PIN_13
#define BRD_D13_GPIO_PORT			GPIOB
#define __BRD_D13_GPIO_CLK()		__GPIOB_CLK_ENABLE()
#define BRD_D13_EXTI_IRQ			EXTI15_10_IRQn

#define BRD_PWR_CRTL_HEADERS_PIN					GPIO_PIN_2
#define BRD_PWR_CRTL_HEADERS_GPIO_PORT			GPIOB
#define __BRD_PWR_CRTL_HEADERS_GPIO_CLK()		__GPIOB_CLK_ENABLE()
#define BRD_PWR_CTRL_ETHERNET

//NP2 I2C SDA/SCL PIN Definitions
#define BRD_I2C				I2C1
#define BRD_I2C_CLK         RCC_APB1Periph_I2C1

#define BRD_SDA_PIN				GPIO_PIN_7
#define BRD_SDA_GPIO_PORT		GPIOB
#define __BRD_SDA_GPIO_CLK()	__GPIOB_CLK_ENABLE()
#define BRD_SDA_AF          	GPIO_AF_I2C1

#define BRD_SCL_PIN				GPIO_PIN_6
#define BRD_SCL_GPIO_PORT		GPIOB
#define __BRD_SCL_GPIO_CLK()	__GPIOB_CLK_ENABLE()
#define BRD_SCL_AF          	GPIO_AF_I2C1

//NP2 debug printf uart definitions
#define BRD_DEBUG_UART				UART4_BASE
#define BRD_DEBUG_UART_BAUDRATE		115200
#define __BRD_DEBUG_UART_CLK()		__UART4_CLK_ENABLE()
#define BRD_DEBUG_UART_AF			GPIO_AF8_UART4
#define BRD_DEBUG_UART_TX_PIN		GPIO_PIN_0
#define BRD_DEBUG_UART_TX_GPIO_PORT	GPIOA
#define __BRD_DEBUG_UART_GPIO_CLK()		__GPIOA_CLK_ENABLE()

//NP2 SPI definitions
#define BRD_SPI                       SPI2_BASE
#define __BRD_SPI_CLK()               __SPI2_CLK_ENABLE()

#define BRD_SPI_SCK_PIN               GPIO_PIN_13                 /* PC.13 */
#define BRD_SPI_SCK_GPIO_PORT         GPIOB                       /* GPIOB */
#define __BRD_SPI_SCK_GPIO_CLK()      __GPIOB_CLK_ENABLE()
#define BRD_SPI_SCK_AF                GPIO_AF5_SPI2

#define BRD_SPI_MISO_PIN              GPIO_PIN_14                 /* PC.14 */
#define BRD_SPI_MISO_GPIO_PORT        GPIOB                       /* GPIOB */
#define __BRD_SPI_MISO_GPIO_CLK()     __GPIOB_CLK_ENABLE()
#define BRD_SPI_MISO_AF               GPIO_AF5_SPI2

#define BRD_SPI_MOSI_PIN              GPIO_PIN_15                 /* PC.15 */
#define BRD_SPI_MOSI_GPIO_PORT        GPIOB                       /* GPIOB */
#define __BRD_SPI_MOSI_GPIO_CLK()     __GPIOB_CLK_ENABLE()
#define BRD_SPI_MOSI_AF               GPIO_AF5_SPI2

#define BRD_SPI_CS_PIN                GPIO_PIN_10                 /* PB.10 */
#define BRD_SPI_CS_GPIO_PORT          GPIOB                       /* GPIOB */
#define __BRD_SPI_CS_GPIO_CLK()       __GPIOB_CLK_ENABLE()

//NP2 Micro SD Card definitions
#define BRD_MMC_SPI                       SPI3_BASE
#define __BRD_MMC_SPI_CLK                   __SPI3_CLK_ENABLE();

#define BRD_MMC_SPI_SCK_PIN               GPIO_PIN_10                 /* PC.10 */
#define BRD_MMC_SPI_SCK_GPIO_PORT         GPIOC                       
#define __BRD_MMC_SPI_SCK_GPIO_CLK()      __GPIOC_CLK_ENABLE()
#define BRD_MMC_SPI_SCK_AF                GPIO_AF_SPI3

#define BRD_MMC_SPI_MISO_PIN              GPIO_PIN_11                 /* PC.11 */
#define BRD_MMC_SPI_MISO_GPIO_PORT        GPIOC                       
#define BRD_MMC_SPI_MISO_GPIO_CLK()       __GPIOC_CLK_ENABLE()
#define BRD_MMC_SPI_MISO_AF               GPIO_AF_SPI3

#define BRD_MMC_SPI_MOSI_PIN              GPIO_PIN_12                 /* PC.12 */
#define BRD_MMC_SPI_MOSI_GPIO_PORT        GPIOC                      
#define __BRD_MMC_SPI_MOSI_GPIO_CLK()     __GPIOC_CLK_ENABLE()
#define BRD_MMC_SPI_MOSI_AF               GPIO_AF_SPI3

#define BRD_MMC_SPI_CS_PIN                GPIO_PIN_0                  /* PB.00 */
#define BRD_MMC_SPI_CS_GPIO_PORT          GPIOB                       
#define __BRD_MMC_SPI_CS_GPIO_CLK()       __GPIOB_CLK_ENABLE()

#define BRD_MMC_PWR_PIN                	  GPIO_PIN_1                  /* PB.01 */
#define BRD_MMC_PWR_GPIO_PORT          	  GPIOB                       
#define __BRD_MMC_PWR_GPIO_CLK()          __GPIOB_CLK_ENABLE()

#define BRD_MMC_SPI_INT1_PIN              GPIO_PIN_5                  /* PB.05 */
#define BRD_MMC_SPI_INT1_GPIO_PORT        GPIOB                       
#define __BRD_MMC_SPI_INT1_GPIO_CLK()     __GPIOB_CLK_ENABLE()
#define BRD_MMC_SPI_INT1_EXTI_IRQn        EXTI5_IRQn 

//NP2 ENC28J60 Ethernet definitions
#define BRD_ENC_SPI                       SPI1_BASE
#define __BRD_ENC_SPI_CLK()               __SPI1_CLK_ENABLE();

#define BRD_ENC_SPI_SCK_PIN               GPIO_PIN_5                 /* PC.05 */
#define BRD_ENC_SPI_SCK_GPIO_PORT         GPIOA                       
#define __BRD_ENC_SPI_SCK_GPIO_CLK()      __GPIOA_CLK_ENABLE()
#define BRD_ENC_SPI_SCK_AF                GPIO_AF5_SPI1

#define BRD_ENC_SPI_MISO_PIN              GPIO_PIN_4                 /* PB.04 */
#define BRD_ENC_SPI_MISO_GPIO_PORT        GPIOB                      
#define __BRD_ENC_SPI_MISO_GPIO_CLK()     __GPIOB_CLK_ENABLE()
#define BRD_ENC_SPI_MISO_AF               GPIO_AF5_SPI1

#define BRD_ENC_SPI_MOSI_PIN              GPIO_PIN_7                 /* PC.07 */
#define BRD_ENC_SPI_MOSI_GPIO_PORT        GPIOA                      
#define __BRD_ENC_SPI_MOSI_GPIO_CLK()     __GPIOA_CLK_ENABLE()
#define BRD_ENC_SPI_MOSI_AF               GPIO_AF5_SPI1

#define BRD_ENC_SPI_CS_PIN                GPIO_PIN_8                  /* PC.08 */
#define BRD_ENC_SPI_CS_GPIO_PORT          GPIOC                       
#define __BRD_ENC_SPI_CS_GPIO_CLK()       __GPIOC_CLK_ENABLE()

//Ethernet Power
#define BRD_ENC_PWR_PIN                	  GPIO_PIN_15                  /* PC.15 */
#define BRD_ENC_PWR_GPIO_PORT          	  GPIOC                      
#define __BRD_ENC_PWR_GPIO_CLK()          __GPIOC_CLK_ENABLE()

//Ethernet Reset
#define BRD_ENC_RST_PIN                	  GPIO_PIN_2                  /* PD.02 */
#define BRD_ENC_RST_GPIO_PORT          	  GPIOD                      
#define __BRD_ENC_RST_GPIO_CLK()          __GPIOD_CLK_ENABLE()

//Ethernet Clock input
#define BRD_ENC_CLK_PIN                	  GPIO_PIN_8                  /* PA.08 */
#define BRD_ENC_CLK_GPIO_PORT          	  GPIOA                      
#define __BRD_ENC_CLK_GPIO_CLK()          __GPIOA_CLK_ENABLE()
#define BRD_ENC_CLK_AF               	  GPIO_AF0_MCO

//Ethernet Interrupt
#define BRD_ENC_SPI_INT_PIN              GPIO_PIN_4                  /* PA.04 */
#define BRD_ENC_SPI_INT_GPIO_PORT        GPIOA                       
#define __BRD_ENC_SPI_INT_GPIO_CLK()     __GPIOA_CLK_ENABLE()
#define BRD_ENC_SPI_INT_EXTI_IRQn        EXTI4_IRQn 


/** @addtogroup STM32F4_NETDUINOPLUS2_LOW_LEVEL_LED
  * @{
  */
//#define LEDn                             1	//4
#define BRD_LED					10
#define BRD_LED_PIN                GPIO_PIN_10	//12
#define BRD_LED_GPIO_PORT          GPIOA		//D
#define __BRD_LED_GPIO_CLK()       __GPIOA_CLK_ENABLE()	//D  
   
/**
 * @brief push-button
 */
#define BRD_PB_PIN                	GPIO_PIN_11
#define BRD_PB_GPIO_PORT          	GPIOB
#define __BRD_PB_GPIO_CLK()         __GPIOB_CLK_ENABLE()
#define BRD_PB_EXTI_IRQ          	EXTI15_10_IRQn 


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

void HAL_IncTick(void);
uint32_t HAL_GetTick(void);

#ifdef ENABLE_DEBUG_UART
void BRD_debuguart_putc(unsigned char c);
unsigned char BRD_debuguart_getc();
#endif

  
#ifdef __cplusplus
}
#endif

#endif /* __BOARD_H */

