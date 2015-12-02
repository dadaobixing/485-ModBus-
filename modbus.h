#ifndef _modbus_H_
#define _modbus_H_

sbit led1  =P1^1;//¿ØÖÆled
sbit led2  =P1^0;//¿ØÖÆled 
sbit Recenable=P3^2;
extern char MyAddress;
extern unsigned int CRC16(unsigned char *puchMsg, unsigned int usDataLen);
extern unsigned char getbit(unsigned int  address,unsigned char  function);
extern void SendData(unsigned char  *output,unsigned char  Outlength);
extern void Function12(unsigned char address,unsigned char len);
extern void Function3(unsigned char  address,unsigned char  len);
extern void SendError(char ErrorCode);
extern void Deal();
extern void Init_timer2();
extern void Init_SerialPort(unsigned int baudrate);

#endif