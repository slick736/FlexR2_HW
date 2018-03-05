#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "arrayUtil.h"

//数组的一部分比较（数组1, 数组2, 起始位置, 比较长度）
uint8_t arrayCompare(uint8_t *array1, uint8_t *array2, uint8_t startPosition, uint8_t compLength){
  uint8_t i;
  for(i = startPosition; i < (startPosition + compLength); i++){
    if(array1[i] != array2[i]){
      return 0;
    }
  }
  return 1;
}

//检查陀螺仪输出的特征值，如果符合未初始化的特征值，则要返回一个参数令其重新初始化
uint8_t rawMPUData[13] = {0x03, 0x00,0xB4,0x00,0x00, 0x00,0x00,0x00,0x00, 0xFF,0x4C,0x00,0x00};
uint8_t mpuResultIsRaw(uint8_t *mpuResult){
  if(arrayCompare(mpuResult, rawMPUData, 1, 12) > 0){
    //符合特征，表明未初始化
    return 1;
  }
  //不符合特征，表明已初始化
  return 0;
}
