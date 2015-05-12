/**
  ******************************************************************************
  * @file    netconf.h
  * @author  MDS
  * @date    22-April-2014 
  * @brief   This file contains default MAC, IP, MASK and Gateway address definitions.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __NETCONF_H
#define __NETCONF_H

/* Includes ------------------------------------------------------------------*/
#include "netconfig.h"

#ifdef __cplusplus
 extern "C" {
#endif


/* MAC ADDRESS*/
#ifndef MAC_ADDR0
#define MAC_ADDR0   0x02
#endif
#ifndef MAC_ADDR1
#define MAC_ADDR1   0x67
#endif
#ifndef MAC_ADDR2
#define MAC_ADDR2   0x00
#endif
#ifndef MAC_ADDR3
#define MAC_ADDR3   0x4A
#endif
#ifndef MAC_ADDR4
#define MAC_ADDR4   0x86
#endif
#ifndef MAC_ADDR5
#define MAC_ADDR5   0x5C
#endif
 
/*Static IP ADDRESS*/
#ifndef	IP_ADDR0
#define IP_ADDR0   192
#endif
#ifndef	IP_ADDR1
#define IP_ADDR1   168
#endif
#ifndef	IP_ADDR2
#define IP_ADDR2   0
#endif
#ifndef	IP_ADDR3
#define IP_ADDR3   10
#endif

/*NETMASK*/
#ifndef NETMASK_ADDR0
#define NETMASK_ADDR0   255
#endif
#ifndef NETMASK_ADDR1
#define NETMASK_ADDR1   255
#endif
#ifndef NETMASK_ADDR2
#define NETMASK_ADDR2   255
#endif
#ifndef NETMASK_ADDR3
#define NETMASK_ADDR3   0
#endif

/*Gateway Address*/
#ifndef GW_ADDR0
#define GW_ADDR0   192
#endif
#ifndef GW_ADDR1
#define GW_ADDR1   168
#endif
#ifndef GW_ADDR2
#define GW_ADDR2   0
#endif
#ifndef GW_ADDR3
#define GW_ADDR3   1     
#endif

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void LwIP_Init(void);
void LwIP_DHCP_task(void * pvParameters);

#ifdef __cplusplus
}
#endif

#endif /* __NETCONF_H */


