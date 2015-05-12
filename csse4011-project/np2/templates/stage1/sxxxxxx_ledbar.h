/**   
 ******************************************************************************   
 * @file    mylib/sxxxxxx_ledbar.h  
 * @author  MyName – MyStudent ID   
 * @date    03032015   
 * @brief   LED Light Bar peripheral driver   
 *	     REFERENCE: LEDLightBar_datasheet.pdf   
 *
 *			NOTE: REPLACE sxxxxxx with your student login.
 ******************************************************************************   
 *     EXTERNAL FUNCTIONS
 ******************************************************************************
 * sxxxxxx_ledbar_init() – intialise LED Light BAR
 * sxxxxxx_ledbar_set() – set LED Light BAR value
 ******************************************************************************   
 *     REVISION HISTORY
 ******************************************************************************
 * 1. 3/3/2015 - Created
 * 2. 10/3/2015 – Added functionality to set function.  
 */

#ifndef SXXXXXX_LEDBAR_H
#define SXXXXXX_LEDBAR_H

/* Includes ------------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

extern void sxxxxxx_ledbar_init(void);
extern void sxxxxxx_ledbar_set(unsigned short value);
#endif

