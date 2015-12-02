#include "STC15F2K60S2.h"
#include "IIC.h"
#include "delay.h"
#include "modbus.h"
unsigned char Exflag;
unsigned char Exdata;

void i2cinit()//总线初始化
{
 SDA = 1;
 delay();
 SCL = 1;
 delay();
}
void start()//启动信号
{
 SDA = 1;
 SCL = 1;
 delay();
 SDA = 0;
 delay();
}
void stop()//停止信号
{
 SDA = 0;
 delay();
 SCL = 1;
 delay();
 SDA = 1;
 delay();
}
void respons()//应答信号
{
 unsigned char i = 0;
 SCL = 1;
 delay();
 while(SDA == 1 && i < 255)//等待应答，过一段时间不应答退出循环
  i++;
 SCL = 0;
 delay();
}
void writebyte(unsigned char date)//写一个字节
{
 unsigned char i,temp;
 temp = date;
 for(i = 0; i < 8; i++)
 {
  temp <<= 1;//temp左移一位后高位进CY
  SCL = 0;
  delay();
  SDA = CY;
  delay();
  SCL = 1;
  delay();
 } 
 SCL = 0;//应答信号中SCL = 1，所以这里要置0
 delay();
 SDA = 1;//用完要释放数据总线
 delay();
}
unsigned char readbyte()//读一个字节 
{
 unsigned char i,k;
 SCL = 0;
 delay();
 SDA = 1;
 for(i = 0; i < 8; i++)
 {
  SCL = 1; 
  delay();
  k = (k << 1) | SDA; //和最低位或，一位位送到K
  SCL = 0;
  delay();
 }
 delay();
 return k;
}
void write_add(unsigned char address,unsigned char date)//向控制板发送数据
{
 start();
 writebyte(0x01);//向地址为01的控制板发送数据
 respons();
 writebyte(address); //控制板内部暂存器地址
 respons();
 writebyte(date);
 respons();
 stop();
}
unsigned char read_add(unsigned char address)//读控制板内部寄存器数据
{
 unsigned char date;
 start();
 writebyte(0x01);// 地址01
 respons();
 writebyte(address); //控制板内部暂存器地址 
 respons();
 start();
 writebyte(0x02);//地址+1做校验
 respons();
 date = readbyte();	//开始读数据
 stop();			//读取完毕停止
 return date;
}