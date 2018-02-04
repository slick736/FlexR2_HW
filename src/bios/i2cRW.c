#include "i2cRW.h"

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include "stdio.h"
#include "main.h"

#include "Board.h"
/* Board specific I2C addresses */
#define Board_TMP_ADDR              (0x40)

I2C_Handle      i2c;
I2C_Params      i2cParams;
I2C_Transaction i2cTransaction;

int i2CRead(uint16_t slaveAddress, uint8_t readCount, void *readBuf);
int i2CWrite(uint16_t slaveAddress, uint8_t writeCount, void *writeBuf);

//��ʼ��I2C�ӿڡ����û����һ�����������̸
int initI2C(void)
{
  I2C_init();
  I2C_Params_init(&i2cParams);
  //��ʱѡ��400kHz������ʽ�����п��ܻ�ѡ��100kHz����ʡ��
  i2cParams.bitRate = I2C_400kHz;
  //i2cParams.bitRate = I2C_100kHz;
  i2c = I2C_open(Board_I2C0, &i2cParams);
  if (i2c == NULL) {
    //��ʼ��ʧ��
    //System_printf("Error Initializing I2C\n");
    return 0;
  } else {
    //��ʼ���ɹ�
    //System_printf("I2C Initialized!\n");
    return 1;
  }
}

//I2C�Ķ�ȡ==============================================================================
//I2C���豸��ȡ���ݵ���������
int i2CReadFromMPU(unsigned char Address, unsigned char RegisterAddr, unsigned short RegisterLen, unsigned char *RegisterValue)
{
  if(i2c == NULL){
    //I2C�ӿځKδ��Ч��ʼ��
    return 2;
  }
  uint8_t slaveAddress = Address;
  //uint8_t tempAddr = RegisterAddr;
  uint8_t writeBuf[1] = { RegisterAddr };
  //i2cTransaction.slaveAddress = RegisterAddr;
  i2cTransaction.slaveAddress = slaveAddress; // <-- ����Ϊ�����ǣ�0xD0
  i2cTransaction.writeBuf = writeBuf; // <-- ����Ϊ�������е��ڲ��Ĵ�����0X44
  i2cTransaction.writeCount = 1;
  i2cTransaction.readBuf = RegisterValue;
  i2cTransaction.readCount = RegisterLen;
  I2C_transfer(i2c, &i2cTransaction);
  return 0;
}
//=================================================================================

//I2C��д��=========================================================================
//I2C���豸д�����ݵ���������
int i2CWriteToMPU(unsigned char Address, unsigned char RegisterAddr, unsigned short RegisterLen, unsigned char *RegisterValue)
{
  if(i2c == NULL){
    //I2C�ӿځKδ��Ч��ʼ��
    return 2;
  }
  uint8_t slaveAddress = Address;
  uint8_t writeLength = RegisterLen + 1;
  uint8_t writeBuf[8];
  uint8_t i;
  for(i = 0; i < writeLength; i++){
    if(i == 0){
      writeBuf[i] = RegisterAddr;
    }else{
      writeBuf[i] = RegisterValue[i - 1];
    }
  }
  
  i2cTransaction.slaveAddress = slaveAddress;
  i2cTransaction.writeBuf = writeBuf;
  i2cTransaction.writeCount = 1 + RegisterLen;
  i2cTransaction.readBuf = NULL;
  i2cTransaction.readCount = 0;
  I2C_transfer(i2c, &i2cTransaction);
  return 0;
}
//====================================================================================
