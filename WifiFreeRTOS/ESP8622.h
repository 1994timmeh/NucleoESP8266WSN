typedef struct{
  char message[50];
} UART_Message;

#define WIFI_CMD_RST        "AT+RST\r\n"
#define WIFI_LEN_RST 9

#define WIFI_CMD_MODE_STA   "AT+CWMODE=1\r\n"

#define WIFI_CMD_MODE_AP    "AT+CWMODE=2\r\n"

#define WIFI_CMD_MODE_BOTH  "AT+CWMODE=3\r\n"
#define WIFI_LEN_MODE_BOTH 13

#define WIFI_CMD_JOIN "AT+CWJAP=\"%s\",\"%s\"\r\n"

#define WIFI_CMD_JOIN_TIMMY_HOME "AT+CWJAP=\"Hadwen AirPort\",\"5Awr2juW\"\r\n"
#define WIFI_LEN_JOIN_TIMMY_HOME 38

#define WIFI_CMD_QUIT_AP "AT+CWQAP\r\n"
#define WIFI_LEN_QUIT_AP 10

#define WIFI_CMD_START_TCP "AT+CIPSTART=0,\"TCP\",\"%d.%d.%d.%d\",%d\r\n"

#define WIFI_CMD_SET_AP "AT+CWSAP=\"%s\",\"%s\",3,0\n\r"

#define WIFI_CMD_SERVE "AT+CIPSERVER=1,8888\r\n"
#define WIFI_LEN_SERVE 21

#define WIFI_CMD_MUX_1 "AT+CIPMUX=1\r\n"
#define WIFI_LEN_MUX_1 13

#define WIFI_CMD_LIST_APS "AT+CWLAP\r\n"
#define WIFI_LEN_LIST_APS 10

#define WIFI_CMD_STATUS "AT+CIPSTATUS\r\n"
#define WIFI_LEN_STATUS 14

#define WIFI_CMD_GET_IP "AT+CIFSR\r\n"
#define WIFI_LEN_GET_IP 10

#define WIFI_CMD_SEND_DATA "AT+CIPSEND=0,3\r\n"
#define WIFI_LEN_SEND_DATA 16

#define mainLED_PRIORITY					( tskIDLE_PRIORITY + 2 )
#define mainLED_TASK_STACK_SIZE		( configMINIMAL_STACK_SIZE * 2 )

#define SEC 0x7FFF00
#define task_loop for(;;)

// Datatypes
typedef struct Access_Point {} Access_Point;



// Tasks
void UART_Processor( void );

// Starting functions
void ESP8622_init( void );
void ESP8622_send_test( void );

// Library functions
void Wifi_reset( void );
void Wifi_join(char SSID[50], char password[50]);
void Wifi_setmode( void );
void Wifi_listAPs( void );
void Wifi_status( void );
void Wifi_getip( void );
void Wifi_senddata();

// Helpers
void waitForPassed(int timeout);
void waitForPrompt();
void Delay(int x);
float RSSItoDistance(int rssi);

// Interrupts
void UART1_IRQHandler(void);
