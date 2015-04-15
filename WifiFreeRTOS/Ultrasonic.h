#define SEC 0x7FFF00
#include <stdint.h>

void Ultrasonic_init();
void Ultrasonic_start();
void handle_Ultrasonic_Data(uint8_t node, uint8_t* data_String);
uint16_t Ultrasonic_getdist();
uint16_t Ultrasonic_getwidth();
void uDelay(unsigned long nCount);


void tim3_irqhandler(void);
