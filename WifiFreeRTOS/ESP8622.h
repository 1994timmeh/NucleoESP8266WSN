typedef struct{
  char message[50];
} UART_Message;

#define WIFI_CMD_RST        "AT+RST\r\n"
#define WIFI_LEN_RST 9

#define WIFI_CMD_MODE_STA   "AT+CWMODE=1\r\n"

#define WIFI_CMD_MODE_AP    "AT+CWMODE=2\r\n"

#define WIFI_CMD_MODE_BOTH  "AT+CWMODE=3\r\n"
#define WIFI_LEN_MODE_BOTH 13

#define WIFI_CMD_JOIN_TIMMY_HOME "AT+CWJAP=\"Hadwen AirPort\",\"5Awr2juW\"\r\n"
#define WIFI_LEN_JOIN_TIMMY_HOME 38

#define WIFI_CMD_QUIT_AP "AT+CWQAP\r\n"
#define WIFI_LEN_QUIT_AP 10

#define WIFI_CMD_SET_AP "AT+CWSAP=\"NODE0x01\",\"password\"\r\n"
#define WIFI_LEN_SET_AP 31

// Will have to stick with this format in the future to allow sprintf new data
#define WIFI_CMD_START_TCP "AT+CIPSTART=4,\"TCP\",\"%d.%d.%d.%d\",%d\r\n"

#define WIFI_CMD_SERVE "AT+CIPSERVER=1,8888\r\n"
#define WIFI_LEN_SERVE 21

#define WIFI_CMD_MUX_1 "AT+CIPMUX=1\r\n"
#define WIFI_LEN_MUX_1 13

#define WIFI_CMD_LIST_APS "AT+CWLAP\r\n"
#define WIFI_LEN_LIST_APS 10

#define WIFI_CMD_STATUS "AT+CIPSTATUS\r\n"
#define WIFI_LEN_STATUS 14

#define mainLED_PRIORITY					( tskIDLE_PRIORITY + 2 )
#define mainLED_TASK_STACK_SIZE		( configMINIMAL_STACK_SIZE * 2 )

#define SEC 0x7FFF00
#define task_loop for(;;)

//Tasks
void UART_Processor( void );

//Starting functions
void ESP8622_init( void );
void ESP8622_send_test( void );

//Library functions
void Wifi_reset( void );
void Wifi_join( void );
void Wifi_setmode( void );
void Wifi_listAPs( void );
void Wifi_status( void );

//Helpers
void waitForPassed();
void Delay(int x);

//Interupts
void USART1_IRQHandler(void);
