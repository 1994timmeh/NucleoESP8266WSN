#define SEC 0x7FFF00

void Ultrasonic_init();
void Ultrasonic_start();
void Delay(int time);

void tim3_irqhandler(void);
