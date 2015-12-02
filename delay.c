#include "STC15F2K60S2.h"
#include "IIC.h"
#include "delay.h"
#include "modbus.h"

void delay() 
{
 ;;
}
void delay_ms(unsigned int z)
{
	unsigned int x,y;
	for(x=z;x>0;x--)
		for(y=110;y>0;y--);
}