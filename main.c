//#include <REGX52.H>
#include "STC15F2K60S2.h"
#include "IIC.h"
#include "delay.h"
#include "modbus.h"
#include "E2PROM.h"

void main()
{
	i2cinit();
	delay_ms(20);

	write_add(0x01,0x20);
	delay_ms(20);
	write_add(0x03,110);
	delay_ms(20);

	Init_SerialPort(9600);
	Init_timer2();
	EA=1;                                   //开总中断
	Recenable=0;                            //485接收允许

	MyAddress=IapReadByte(0x1300);			//从EEPROM地址0x1300读取站号
	delay_ms(50);
//	SBUF=MyAddress;
//	delay_ms(50);

	while(1)
	{
		if(Exflag==1)
		{
			Exdata=read_add(0x01); //开始I2C上限中断数据
			delay_ms(20);
			if(Exdata==0x20)	   //如果接收到上限数据
			{
				led1=1;
				led2=1;			   //关继电器
				Exflag=0;		   //清零I2C通信标志位
				read_add(0x02);    //结束I2C通信
			}
		}
	}

}