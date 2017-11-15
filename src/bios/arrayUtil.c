#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "arrayUtil.h"

//�����һ���ֱȽϣ�����1, ����2, ��ʼλ��, �Ƚϳ��ȣ�
uint8_t arrayCompare(uint8_t *array1, uint8_t *array2, uint8_t startPosition, uint8_t compLength){
  uint8_t i;
  for(i = startPosition; i < (startPosition + compLength); i++){
    if(array1[i] != array2[i]){
      return 0;
    }
  }
  return 1;
}

//������������������ֵ���������δ��ʼ��������ֵ����Ҫ����һ�������������³�ʼ��
uint8_t rawMPUData[13] = {0x03, 0x00,0xB4,0x00,0x00, 0x00,0x00,0x00,0x00, 0xFF,0x4C,0x00,0x00};
uint8_t mpuResultIsRaw(uint8_t *mpuResult){
  if(arrayCompare(mpuResult, rawMPUData, 1, 12) > 0){
    //��������������δ��ʼ��
    return 1;
  }
  //�����������������ѳ�ʼ��
  return 0;
}
