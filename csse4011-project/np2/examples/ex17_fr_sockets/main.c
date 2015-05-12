/**
  ******************************************************************************
  * @file    ex17_fr_socket/main.c 
  * @author  MDS
  * @date    22-April-2014
  * @brief   FreeRTOS LWIP socket networking example. Binds and listens on both
  *			 UDP and TCP sockets.Will accept and return ASCII strings in capitals.
  *
  *			 LWIP socket API uses the same commands as standard socket interfaces.
  *			 i.e. connect, bind, accept, read, write, recv, recvfrom, send, etc.
  *
  *			 OSX\Linux Use netcat (nc) to access sockets:
  *										   e.g. For TCP: "nc 192.168.0.10 10"
  *											    For UDP: "nc -u 192.168.0.10 11"
  *			 Windows: Use Putty - set connection type to 'raw'.
  *			 Default IP Address: 192.168.0.10 (see netconfg.h)
  *
  *			 Open serial terminal (kermit) to view received bytes.
  *			 
  ******************************************************************************
  *  
  */ 

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "debug_printf.h"
#include "usbd_cdc_vcp.h"
#include <string.h>

#include "netconf.h"
#include "FreeRTOS.h"
#include "task.h"

#include "tcpip.h"
#include "lwip/opt.h"
#include "lwip/api.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define TCP_SOCKET_PORT	10
#define UDP_SOCKET_PORT	11

/* Task Priorities -----------------------------------------------------------*/
#define TCPTASK_PRIORITY   ( tskIDLE_PRIORITY + 2 )      
#define UDPTASK_PRIORITY    ( tskIDLE_PRIORITY + 1 )

/* Task Stack Allocations -----------------------------------------------------*/
#define TCPTASK_STACK_SIZE		( configMINIMAL_STACK_SIZE * 6 )
#define UDPTASK_STACK_SIZE		( configMINIMAL_STACK_SIZE * 6 )


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void Hardware_init();
void TCPTask(void * pvParameters);
void UDPTask(void * pvParameters);
void vApplicationIdleHook( void );	/* The idle hook is just used to stream data to the USB port.*/


/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void) {
  
	BRD_init();
	Hardware_init();
    
  	/* Initilaize the LwIP stack */
  	LwIP_Init();
  
	xTaskCreate( (void *) &UDPTask, (const signed char *) "UDP", UDPTASK_STACK_SIZE, NULL, UDPTASK_PRIORITY, NULL);
	xTaskCreate( (void *) &TCPTask, (const signed char *) "TCP", TCPTASK_STACK_SIZE, NULL, TCPTASK_PRIORITY, NULL);
  
  	/* Start scheduler */
  	vTaskStartScheduler();

	return 0;
}


/**
  * @brief  TCP Task. Bind to a TCP Socket. Reflect back captilised version of received string.
  * @param  None
  * @retval None
  */
void TCPTask(void * pvParameters) {

	int sock = 0;
	int tcpconn = 0;
	long size = 0;
  	int ret = 0;
	int i;
  	unsigned char recv_buffer[20];

	struct sockaddr_in address, remotehost;

 	/* create a TCP socket */
  	if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
    	debug_printf("can not create socket");
		VCP_txflush();
    	return;
  	}
  
  	/* bind to port 80 at any interface */
  	address.sin_family = AF_INET;
  	address.sin_port = htons(TCP_SOCKET_PORT);
  	address.sin_addr.s_addr = INADDR_ANY;

  	if (bind(sock, (const struct sockaddr *)(&address), sizeof (address)) < 0) {
    	debug_printf("can not bind socket");
		VCP_txflush();
    	return;
  	}
  
  	/* listen for incoming connections (TCP listen backlog = 5) */
  	listen(sock, 5);
  
  	size = sizeof(remotehost);

	memset(recv_buffer, 0, sizeof(recv_buffer));  

  	for (;;) {

		/* Wait for connection */
    	tcpconn = accept(sock, (struct sockaddr *)(&remotehost), (socklen_t *)&size);
    	
				
  		/* Read from Socket */		
		while ((ret = read(tcpconn, recv_buffer, sizeof(recv_buffer) )) > 0) { 

			debug_printf("TCP Socket: ");
			for (i = 0; i < ret; i++) {
			
				/* Convert to ASCII upper case */
				if (recv_buffer[i] > 96) {
					recv_buffer[i] -= 32;
				}

				debug_putc(recv_buffer[i]);		//put character into USB buffer			
			}

			/* Put new line and carrriage return characters into the USB buffer */
			debug_putc('\n');
			debug_putc('\r');
			vTaskDelay(5);
			debug_flush();						//flush USB buffer to transmit characters

			/* Write buffer back to socket */
			write(tcpconn, recv_buffer, ret);
		}

		vTaskDelay(100);
  	}
}


/**
  * @brief  UDP Task. Listen to a UDP Socket. Reflect back captilised version of received string.
  * @param  None
  * @retval None
  */
void UDPTask(void * pvParameters) {

	int udpsock = 0;
  	int ret = 0; 
	int len = 0;
	int i;
  	unsigned char recv_buffer[20];
	struct sockaddr_in address; 
	struct sockaddr remotehost;

 	/* create a UDP socket */
  	if ((udpsock = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    	//debug_printf("can not create UDP socket");
    	return;
  	}
  
  	/* bind to UDP port at any interface */
  	address.sin_family = AF_INET;
  	address.sin_port = htons(UDP_SOCKET_PORT);
  	address.sin_addr.s_addr = INADDR_ANY;

	if (bind(udpsock, (const struct sockaddr *)(&address), sizeof (address)) < 0) {
    	debug_printf("can not bind UPD socket");
    	return;
  	}

	len = (uint32_t) sizeof(remotehost);

  	for (;;) {
				
  		/* Read from Socket */
		while ((ret = recvfrom(udpsock, recv_buffer, sizeof(recv_buffer), 0, (struct sockaddr *)(&remotehost), (uint32_t *) &len)) > 0) { 

			debug_printf("UDP Socket: ");
			for (i = 0; i < ret; i++) {

				/* Convert to ASCII upper case */
				if (recv_buffer[i] > 96) {
					recv_buffer[i] -= 32;
				}
				debug_putc(recv_buffer[i]);
			}
			debug_putc('\n');
			debug_putc('\r');
			vTaskDelay(5);
	  		debug_flush();

			/* Write back to socket */
			sendto(udpsock, recv_buffer, ret, 0, (struct sockaddr *)(&remotehost), sizeof(remotehost));
		}

		vTaskDelay(100);
  	}
}

/**
  * @brief  Hardware Initialisation.
  * @param  None
  * @retval None
  */
void Hardware_init( void ) {
	portDISABLE_INTERRUPTS();

	BRD_LEDInit();		//Initialise Blue LED
	BRD_LEDOff();		//Turn off Blue LED

	portENABLE_INTERRUPTS();
}

/**
  * @brief  Idle Application Task (Disabled)
  * @param  None
  * @retval None
  */
void vApplicationIdleHook( void ) {
	static TickType_t xLastTx = 0;

	BRD_LEDOff();
	/* The idle hook simply prints the idle tick count */
	if ((xTaskGetTickCount() - xLastTx ) > (1000 / portTICK_RATE_MS)) {
		xLastTx = xTaskGetTickCount();
		debug_printf("IDLE Tick %d\n", xLastTx);		
	}
}

/**
  * @brief  vApplicationStackOverflowHook
  * @param  Task Handler and Task Name
  * @retval None
  */
void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName ) {
	/* This function will get called if a task overflows its stack.   If the
	parameters are corrupt then inspect pxCurrentTCB to find which was the
	offending task. */

	BRD_LEDOff();
	( void ) pxTask;
	( void ) pcTaskName;

	for( ;; );
}

