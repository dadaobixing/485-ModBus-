#ifndef _IIC_H_

#define _IIC_H_
 
sbit SDA=P1^2;                         
sbit SCL=P1^3;

extern unsigned char Exflag;
extern unsigned char Exdata;

extern void i2cinit();
extern void stop();
extern void respons();
extern void writebyte(unsigned char date);
extern unsigned char readbyte();
extern void write_add(unsigned char address,unsigned char date);
extern unsigned char read_add(unsigned char address);

 

#endif