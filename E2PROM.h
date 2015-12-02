#ifndef _E2PROM_H_

#define _E2PROM_H_

extern void IapIdle();
extern unsigned char IapReadByte(unsigned int addr);
extern void IapProgramByte(unsigned int addr, unsigned char dat);
extern void IapEraseSector(unsigned int addr);

#endif