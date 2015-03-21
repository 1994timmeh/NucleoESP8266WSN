#define WIFI_CMD_RST        "AT+RST\r\n"
#define WIFI_LEN_RST 9

#define WIFI_CMD_MODE_STA   "AT+CWMODE=1\r\n"

#define WIFI_CMD_MODE_AP    "AT+CWMODE=2\r\n"

#define WIFI_CMD_MODE_BOTH  "AT+CWMODE=3\r\n"
#define WIFI_LEN_MODE_BOTH 13

#define WIFI_CMD_JOIN_TIMMY_HOME "AT+CWJAP=\"Hadwen AirPort\",\"5Awr2juW\"\r\n"
#define WIFI_LEN_JOIN_TIMMY_HOME 38

#define WIFI_CMD_LIST_APS "AT+CWLAP\r\n"
#define WIFI_LEN_LIST_APS 10

#define WIFI_CMD_STATUS "AT+CIPSTATUS\r\n"
#define WIFI_LEN_STATUS 14

#define WIFI_CMD_SET_AP "AT+CWSAP=\"NODE0x01\",\"password\"\r\n"
#define WIFI_LEN_SET_AP 31

#define SEC 0x7FFF00


void ESP8622_init( void );
void ESP8622_send_test( void );

void Wifi_reset( void );
void Wifi_join( void );
void Wifi_setmode( void );
void Wifi_listAPs( void );
void Wifi_status( void );
