#ifndef __DS18B20_
#define __DS18B20_
#include"gpio.h"

void Init_DS18B20(void);
uint8_t ReadOneChar(void);
void WriteOneChar(unsigned char dat);
double ReadTemperature(void);

#endif 








