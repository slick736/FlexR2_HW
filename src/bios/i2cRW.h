#ifndef I2CRW_H
#define I2CRW_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

extern int initI2C(void);
extern int i2CReadFromMPU(unsigned char Address, unsigned char RegisterAddr, unsigned short RegisterLen, unsigned char *RegisterValue);
extern int i2CWriteToMPU(unsigned char Address, unsigned char RegisterAddr, unsigned short RegisterLen, unsigned char *RegisterValue);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif