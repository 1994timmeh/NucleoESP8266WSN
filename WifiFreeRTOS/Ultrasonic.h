#define SEC 0x7FFF00

void Ultrasonic_init();
void Ultrasonic_start();
int Ultrasonic_getdist();
void uDelay(unsigned long nCount);


void tim3_irqhandler(void);
