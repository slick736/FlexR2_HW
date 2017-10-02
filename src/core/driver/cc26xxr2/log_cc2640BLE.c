/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
/*******************************************************************************
 * $Id: $
 *******************************************************************************/

/**
 *  @defgroup MSP430_System_Layer MSP430 System Layer
 *  @brief  MSP430 System Layer APIs.
 *          To interface with any platform, eMPL needs access to various
 *          system layer functions.
 *
 *  @{
 *      @file   log_msp430.c
 *      @brief  Logging facility for the TI MSP430.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>

#include "packet.h"
#include "log.h"

#define BUF_SIZE        (256)
#define PACKET_LENGTH   (23)

#define PACKET_DEBUG    (1)
#define PACKET_QUAT     (2)
#define PACKET_DATA     (3)

/**
 *  @brief      Prints a variable argument log message.
 *  USB output will be formatted as follows:\n
 *  packet[0]       = $\n
 *  packet[1]       = packet type (1: debug, 2: quat, 3: data)\n
 *  packet[2]       = \n for debug packets: log priority\n
 *                    for quaternion packets: unused\n
 *                    for data packets: packet content (accel, gyro, etc)\n
 *  packet[3-20]    = data\n
 *  packet[21]      = \\r\n
 *  packet[22]      = \\n
 *  @param[in]  priority    Log priority (based on Android).
 *  @param[in]  tag         File specific string.
 *  @param[in]  fmt         String of text with optional format tags.
 *
 *  @return     0 if successful.
 */
char bleOutData[PACKET_LENGTH];
void initBleOutData(void){
  uint8_t i;
  for(i = 0; i < PACKET_LENGTH; i++){
    bleOutData[i] = 0;
  }
}
void putBleOutData(char *datas){
  uint8_t i;
  for(i = 0; i < PACKET_LENGTH; i++){
    bleOutData[i] = datas[i];
  }
}
void getBleOutData(char *datas){
  uint8_t i;
  for(i = 0; i < PACKET_LENGTH; i++){
    datas[i] = bleOutData[i];
  }
}

int _MLPrintLog (int priority, const char* tag, const char* fmt, ...)
{
  /*
    
  */
    return 0;
}

void eMPL_send_quat(long *quat)
{
    char out[PACKET_LENGTH];
    if (!quat)
        return;
    memset(out, 0, PACKET_LENGTH);
     //数据类型代码0：四元数
     out[0] = 0;
     out[1] = (char)(quat[0] >> 24);
     out[2] = (char)(quat[0] >> 16);
     out[3] = (char)(quat[0] >> 8);
     out[4] = (char)quat[0];
     out[5] = (char)(quat[1] >> 24);
     out[6] = (char)(quat[1] >> 16);
     out[7] = (char)(quat[1] >> 8);
     out[8] = (char)quat[1];
     out[9] = (char)(quat[2] >> 24);
    out[10] = (char)(quat[2] >> 16);
    out[11] = (char)(quat[2] >> 8);
    out[12] = (char)quat[2];
    out[13] = (char)(quat[3] >> 24);
    out[14] = (char)(quat[3] >> 16);
    out[15] = (char)(quat[3] >> 8);
    out[16] = (char)quat[3];
    //将数据的各字节整合到蓝牙输出区
}

//数据输出（长整形）
void eMPL_send_data(unsigned char type, long *data)
{
    uint8_t i;
    char out[PACKET_LENGTH];
    if (!data)
        return;
    memset(out, 0, PACKET_LENGTH);
    switch (type) {
    case PACKET_DATA_ROT:
         out[1] = (char)(data[0] >> 24);
         out[2] = (char)(data[0] >> 16);
         out[3] = (char)(data[1] >> 24);
         out[4] = (char)(data[1] >> 16);
         out[5] = (char)(data[2] >> 24);
         out[6] = (char)(data[2] >> 16);
         out[7] = (char)(data[3] >> 24);
         out[8] = (char)(data[3] >> 16);
         out[9] = (char)(data[4] >> 24);
        out[10] = (char)(data[4] >> 16);
        out[11] = (char)(data[5] >> 24);
        out[12] = (char)(data[5] >> 16);
        out[13] = (char)(data[6] >> 24);
        out[14] = (char)(data[6] >> 16);
        out[15] = (char)(data[7] >> 24);
        out[16] = (char)(data[7] >> 16);
        out[17] = (char)(data[8] >> 24);
        out[18] = (char)(data[8] >> 16);
        break;
    case PACKET_DATA_QUAT:
        out[13] = (char)(data[3] >> 24);
        out[14] = (char)(data[3] >> 16);
        out[15] = (char)(data[3] >> 8);
        out[16] = (char)data[3];
    case PACKET_DATA_ACCEL:
    case PACKET_DATA_GYRO:
    case PACKET_DATA_COMPASS:
    case PACKET_DATA_EULER:
         out[1] = (char)(data[0] >> 24);
         out[2] = (char)(data[0] >> 16);
         out[3] = (char)(data[0] >> 8);
         out[4] = (char)data[0];
         out[5] = (char)(data[1] >> 24);
         out[6] = (char)(data[1] >> 16);
         out[7] = (char)(data[1] >> 8);
         out[8] = (char)data[1];
         out[9] = (char)(data[2] >> 24);
        out[10] = (char)(data[2] >> 16);
        out[11] = (char)(data[2] >> 8);
        out[12] = (char)data[2];
        break;
    case PACKET_DATA_HEADING:
         out[1] = (char)(data[0] >> 24);
         out[2] = (char)(data[0] >> 16);
         out[3] = (char)(data[0] >> 8);
         out[4] = (char)data[0];
        break;
    default:
        return;
    }
    switch (type) {
    case PACKET_DATA_ROT:
        out[0] = 4; //数据类型代码4：旋转矩阵
        break;
    case PACKET_DATA_QUAT:
        out[0] = 5; //数据类型代码5：四元数
        break;
    case PACKET_DATA_ACCEL:
        out[0] = 0; //数据类型代码0：加速度
        break;
    case PACKET_DATA_GYRO:
        out[0] = 1; //数据类型代码1：角速度
        break;
    case PACKET_DATA_COMPASS:
        out[0] = 2; //数据类型代码2：电子罗盘
        break;
    case PACKET_DATA_EULER:
        out[0] = 3; //数据类型代码3：欧拉角
        break;
    //case PACKET_DATA_HEADING:
        //out[0] = 8; //数据类型代码8：航向角
        //break;
    default:
        out[0] = -1; //数据类型代码-1(255)：操作错误
        return;
    }
    //将数据的各字节整合到蓝牙输出区
    for(i = 0; i < PACKET_LENGTH; i++){
      bleOutData[i] = out[i];
    }
}

//数据输出（浮点型转换成长整形）
void eMPL_send_float_data(unsigned char type, float *float_data)
{
  uint8_t i;
  char out[PACKET_LENGTH];
  long data[3] = { 0, 0, 0 };
  if (!float_data){
    return;
  }
  //浮点型转换成长整形
  for(i = 0; i < 3; i++){
    data[i] = floor(float_data[i] * 65536);
  }
  memset(out, 0, PACKET_LENGTH);
  switch (type) {
  case PACKET_DATA_LINEAR_ACCEL:
  case PACKET_GRAVITY_VECTOR:
     out[1] = (char)(data[0] >> 24);
     out[2] = (char)(data[0] >> 16);
     out[3] = (char)(data[0] >> 8);
     out[4] = (char)data[0];
     out[5] = (char)(data[1] >> 24);
     out[6] = (char)(data[1] >> 16);
     out[7] = (char)(data[1] >> 8);
     out[8] = (char)data[1];
     out[9] = (char)(data[2] >> 24);
    out[10] = (char)(data[2] >> 16);
    out[11] = (char)(data[2] >> 8);
    out[12] = (char)data[2];
    break;
  default:
    return;
  }
  switch (type) {
  case PACKET_DATA_LINEAR_ACCEL:
    out[0] = 6; //数据类型代码6：线性加速度
    break;
  case PACKET_GRAVITY_VECTOR:
    out[0] = 7; //数据类型代码7：重力矢量
    break;
  default:
    out[0] = -1; //数据类型代码-1(255)：操作错误
    return;
  }
  //将数据的各字节整合到蓝牙输出区
  for(i = 0; i < PACKET_LENGTH; i++){
    bleOutData[i] = out[i];
  }
}

/**
 * @}
**/


