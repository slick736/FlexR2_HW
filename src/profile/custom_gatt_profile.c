/******************************************************************************

 @file  simple_gatt_profile.c

 @brief This file contains the Simple GATT profile sample GATT service profile
        for use with the BLE sample application.

 Group: CMCU, SCS
 Target Device: CC2640R2

 ******************************************************************************
 
 Copyright (c) 2010-2017, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 Release Name: simplelink_cc2640r2_sdk_1_30_00_25
 Release Date: 2017-03-02 20:08:35
 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include "bcomdef.h"
#include "osal.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "gapbondmgr.h"
#include "i2cRW.h"
#include "custom_gatt_profile.h"
#include "simple_peripheral.h"
#include "ex_include_tirtos.h"
#include "scif.h"
#include "gpioCustom.h"
#include "main.h"

#include "icall_ble_api.h"
#include "st_util.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

//支持多少个通道？
//#define SERVAPP_NUM_ATTR_SUPPORTED        17 // <-- Original
#define SERVAPP_NUM_ATTR_SUPPORTED        14
//#define SERVAPP_NUM_ATTR_SUPPORTED        29 // <-- Max

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// Simple GATT Profile Service UUID: 0xFFE0
CONST uint8 customProfileServUUID[TI_UUID_SIZE] =
{ 
  TI_UUID(CUSTOMPROFILE_SERV_UUID)
};

// Characteristic 1 UUID: 0xFFF1
CONST uint8 customProfilechar1UUID[TI_UUID_SIZE] =
{ 
  TI_UUID(CUSTOMPROFILE_CHAR1_UUID)
};

// Characteristic 2 UUID: 0xFFF2
CONST uint8 customProfilechar2UUID[TI_UUID_SIZE] =
{ 
  TI_UUID(CUSTOMPROFILE_CHAR2_UUID)
};

// Characteristic 3 UUID: 0xFFF3
CONST uint8 customProfilechar3UUID[TI_UUID_SIZE] =
{ 
  TI_UUID(CUSTOMPROFILE_CHAR3_UUID)
};

// Characteristic 4 UUID: 0xFFF4
CONST uint8 customProfilechar4UUID[TI_UUID_SIZE] =
{ 
  TI_UUID(CUSTOMPROFILE_CHAR4_UUID)
};

/*********************************************************************
 * EXTERNAL VARIABLES
 */
//ADC相关变量
//extern uint8_t adcInitResult;
extern uint8_t adcConvertResult;
/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

static customProfileCBs_t *customProfile_AppCBs = NULL;

/*********************************************************************
 * Profile Attributes - variables
 */

// Simple Profile Service attribute
static CONST gattAttrType_t customProfileService = { TI_UUID_SIZE, customProfileServUUID };


// Simple Profile Characteristic 1 Properties
static uint8 customProfileChar1Props = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic 1 Value
static uint8 customProfileChar1[CUSTOMPROFILE_CHAR1_LEN] = { 4, 9 };

// Simple Profile Characteristic 1 User Description
//static uint8 simpleProfileChar1UserDesp[17] = "Characteristic 1";
static uint8 customProfileChar1UserDesp[17] = "WriteUsers Config";


// Simple Profile Characteristic 2 Properties
static uint8 customProfileChar2Props = GATT_PROP_READ;

// Characteristic 2 Value
static uint8 customProfileChar2[CUSTOMPROFILE_CHAR2_LEN] = { 0, 0, 0, 0, 0, 0, 0, 0 };

// Simple Profile Characteristic 2 User Description
//static uint8 simpleProfileChar2UserDesp[17] = "Characteristic 2";
static uint8 customProfileChar2UserDesp[17] = "Read Users Config";


// Simple Profile Characteristic 3 Properties
static uint8 customProfileChar3Props = GATT_PROP_WRITE;

// Characteristic 3 Value
static uint8 customProfileChar3 = 0;

// Simple Profile Characteristic 3 User Description
//static uint8 simpleProfileChar3UserDesp[17] = "Characteristic 3";
static uint8 customProfileChar3UserDesp[17] = "Char 3 - Write F";


// Simple Profile Characteristic 4 Properties
static uint8 customProfileChar4Props = GATT_PROP_NOTIFY;

// Characteristic 4 Value
static uint8 customProfileChar4 = 0;

// Characteristic 5 Value // <-- 实际上给了2通道
static uint8 customProfileChar5[CUSTOMPROFILE_CHAR5_LEN] = { 0, 0, 0, 0, 0, 0 };

// Simple Profile Characteristic 4 Configuration Each client has its own
// instantiation of the Client Characteristic Configuration. Reads of the
// Client Characteristic Configuration only shows the configuration for
// that client and writes only affect the configuration of that client.
static gattCharCfg_t *customProfileChar4Config;
                                        
// Simple Profile Characteristic 4 User Description
//static uint8 simpleProfileChar4UserDesp[17] = "Characteristic 4";
static uint8 customProfileChar4UserDesp[17] = "FlexRII v1.0.0.2";

//=================================================================================================

/*********************************************************************
 * Profile Attributes - Table
 */
//这里最关键：注册蓝牙通道的数量
static gattAttribute_t customProfileAttrTbl[SERVAPP_NUM_ATTR_SUPPORTED] = 
{
  // Simple Profile Service
  { 
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&customProfileService            /* pValue */
  },
    
    //====================================================================================
    // Characteristic 1 Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &customProfileChar1Props 
    },

      // Characteristic Value 1
      { 
        { TI_UUID_SIZE, customProfilechar1UUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        customProfileChar1 
      },

      // Characteristic 1 User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        customProfileChar1UserDesp 
      },      

    //====================================================================================
    // Characteristic 2 Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &customProfileChar2Props 
    },

      // Characteristic Value 2
      { 
        { TI_UUID_SIZE, customProfilechar2UUID },
        GATT_PERMIT_READ, 
        0, 
        customProfileChar2 
      },

      // Characteristic 2 User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        customProfileChar2UserDesp 
      },           
      
    //====================================================================================
    // Characteristic 3 Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &customProfileChar3Props 
    },

      // Characteristic Value 3
      { 
        { TI_UUID_SIZE, customProfilechar3UUID },
        GATT_PERMIT_WRITE, 
        0, 
        &customProfileChar3 
      },

      // Characteristic 3 User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        customProfileChar3UserDesp 
      },
    
    //====================================================================================
      
    // Characteristic 4 Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &customProfileChar4Props 
    },

      // Characteristic Value 4
      { 
        { TI_UUID_SIZE, customProfilechar4UUID },
        0, 
        0, 
        &customProfileChar4 
      },

      // Characteristic 4 configuration
      { 
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        (uint8 *)&customProfileChar4Config 
      },
      
      // Characteristic 4 User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        customProfileChar4UserDesp 
      }
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t customProfile_ReadAttrCB(uint16_t connHandle,
                                          gattAttribute_t *pAttr, 
                                          uint8_t *pValue, uint16_t *pLen,
                                          uint16_t offset, uint16_t maxLen,
                                          uint8_t method);
static bStatus_t customProfile_WriteAttrCB(uint16_t connHandle,
                                           gattAttribute_t *pAttr,
                                           uint8_t *pValue, uint16_t len,
                                           uint16_t offset, uint8_t method);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// Simple Profile Service Callbacks
// Note: When an operation on a characteristic requires authorization and 
// pfnAuthorizeAttrCB is not defined for that characteristic's service, the 
// Stack will report a status of ATT_ERR_UNLIKELY to the client.  When an 
// operation on a characteristic requires authorization the Stack will call 
// pfnAuthorizeAttrCB to check a client's authorization prior to calling
// pfnReadAttrCB or pfnWriteAttrCB, so no checks for authorization need to be 
// made within these functions.
CONST gattServiceCBs_t customProfileCBs =
{
  customProfile_ReadAttrCB,  // Read callback function pointer
  customProfile_WriteAttrCB, // Write callback function pointer
  NULL                       // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleProfile_AddService
 *
 * @brief   Initializes the Simple Profile service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  Success or Failure
 */
bStatus_t CustomProfile_AddService( uint32 services )
{
  uint8 status;

  // Allocate Client Characteristic Configuration table
  customProfileChar4Config = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) *
                                                            linkDBNumConns );
  if ( customProfileChar4Config == NULL )
  {     
    return ( bleMemAllocError );
  }
  
  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, customProfileChar4Config );
  
  if ( services & CUSTOMPROFILE_SERVICE )
  {
    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService( customProfileAttrTbl, 
                                          GATT_NUM_ATTRS( customProfileAttrTbl ),
                                          GATT_MAX_ENCRYPT_KEY_SIZE,
                                          &customProfileCBs );
  }
  else
  {
    status = SUCCESS;
  }

  return ( status );
}

/*********************************************************************
 * @fn      SimpleProfile_RegisterAppCBs
 *
 * @brief   Registers the application callback function. Only call 
 *          this function once.
 *
 * @param   callbacks - pointer to application callbacks.
 *
 * @return  SUCCESS or bleAlreadyInRequestedMode
 */
bStatus_t CustomProfile_RegisterAppCBs( customProfileCBs_t *appCallbacks )
{
  if ( appCallbacks )
  {
    customProfile_AppCBs = appCallbacks;
    
    return ( SUCCESS );
  }
  else
  {
    return ( bleAlreadyInRequestedMode );
  }
}

/*********************************************************************
 * @fn      SimpleProfile_SetParameter
 *
 * @brief   Set a Simple Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to write
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t CustomProfile_SetParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case CUSTOMPROFILE_CHAR1:
      if ( len == CUSTOMPROFILE_CHAR1_LEN ) 
      {
        VOID memcpy( customProfileChar1, value, CUSTOMPROFILE_CHAR1_LEN );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case CUSTOMPROFILE_CHAR2:
      if ( len == CUSTOMPROFILE_CHAR2_LEN ) 
      {
        VOID memcpy( customProfileChar2, value, CUSTOMPROFILE_CHAR2_LEN );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case CUSTOMPROFILE_CHAR3:
      if ( len == sizeof ( uint8 ) ) 
      {
        customProfileChar3 = *((uint8*)value);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case CUSTOMPROFILE_CHAR4:
      if ( len == sizeof ( uint8 ) ) 
      {
        customProfileChar4 = *((uint8*)value);
        
        // See if Notification has been enabled
        GATTServApp_ProcessCharCfg( customProfileChar4Config, &customProfileChar4, FALSE,
                                    customProfileAttrTbl, GATT_NUM_ATTRS( customProfileAttrTbl ),
                                    INVALID_TASK_ID, customProfile_ReadAttrCB );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;
      
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  
  return ( ret );
}

/*********************************************************************
 * @fn      SimpleProfile_GetParameter
 *
 * @brief   Get a Simple Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to put.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t CustomProfile_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case CUSTOMPROFILE_CHAR1:
      //*((uint8*)value) = customProfileChar1;
      VOID memcpy( value, customProfileChar1, CUSTOMPROFILE_CHAR1_LEN );
      break;

    case CUSTOMPROFILE_CHAR2:
      //*((uint8*)value) = customProfileChar2;
      //将5通道的数值传进value中
      //VOID memcpy( value, customProfileChar2, CUSTOMPROFILE_CHAR2_LEN );
      //将系统信息传进value中，目前包括电池电量和MPU、ADC是否开启
      break;      

    case CUSTOMPROFILE_CHAR3:
      *((uint8*)value) = customProfileChar3;
      break;  

    case CUSTOMPROFILE_CHAR4:
      *((uint8*)value) = customProfileChar4;
      break;
      
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  
  return ( ret );
}

/*********************************************************************
 * @fn          simpleProfile_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 * @param       method - type of read message
 *
 * @return      SUCCESS, blePending or Failure
 */
static uint16_t adcTempResult;
static uint16_t tempSystemOpt;
static bStatus_t customProfile_ReadAttrCB(uint16_t connHandle,
                                          gattAttribute_t *pAttr,
                                          uint8_t *pValue, uint16_t *pLen,
                                          uint16_t offset, uint16_t maxLen,
                                          uint8_t method)
{
  bStatus_t status = SUCCESS;
  
  // Make sure it's not a blob operation (no attributes in the profile are long)
  if ( offset > 0 )
  {
    return ( ATT_ERR_ATTR_NOT_LONG );
  }
  uint8_t uuidLen = pAttr->type.len;
  if (uuidLen == ATT_BT_UUID_SIZE || uuidLen == ATT_UUID_SIZE)
  {
    // 16-bit UUID or 128-bit UUID
    uint16 uuid = 0;
    if(uuidLen == ATT_BT_UUID_SIZE){
      // 16-bit UUID
      uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    }else{
      // 128-bit UUID
      uuid = BUILD_UINT16( pAttr->type.uuid[12], pAttr->type.uuid[13]);
    }
    //uint8_t i2cResult[2] = { 0, 0 };
    //uint8_t slaveAddress;
    //uint16_t adcTempResult;
    //uint16_t tempSystemOpt;
    switch ( uuid )
    {
      // No need for "GATT_SERVICE_UUID" or "GATT_CLIENT_CHAR_CFG_UUID" cases;
      // gattserverapp handles those reads

      // characteristics 1 and 2 have read permissions
      // characteritisc 3 does not have read permissions; therefore it is not
      //   included here
      // characteristic 4 does not have read permissions, but because it
      //   can be sent as a notification, it is included here
      case CUSTOMPROFILE_CHAR1_UUID:
        //先读取系统设定
        tempSystemOpt = getSystemOption();
        //0为高，1为低
        customProfileChar1[0] = (tempSystemOpt >> 8);
        customProfileChar1[1] = (tempSystemOpt % 256);
        //再将读取过来的数值发送出去
        *pLen = 2;
        VOID memcpy( pValue, pAttr->pValue, 2 );
        break;
      case CUSTOMPROFILE_CHAR6_UUID:
        //此处修改蓝牙输出长度？
        *pLen = 1; // <-- 1字节8位, 2字节16位
        pValue[0] = *pAttr->pValue;
        break;
      case CUSTOMPROFILE_CHAR4_UUID:
        //此处修改蓝牙输出长度？
        //*pLen = 2; // <-- 1字节8位, 2字节16位
        //pValue[0] = *pAttr->pValue;
        //pValue[1] = *pAttr->pValue;
        *pLen = 2;
        VOID memcpy( pValue, pAttr->pValue, 2 );
        break;
      case CUSTOMPROFILE_CHAR2_UUID:
        //读取这个地方的时候要更新一下I2C传来的数值
        /*
        slaveAddress = 0x69;
        //slaveAddress = (slaveAddress >> 1); // <-- CC2640的从机地址应该右偏一位！
        
        //X角速度
        i2CReadFromMPU(slaveAddress, 0x43, 2, i2cResult);
        customProfileChar2[0] = i2cResult[1];
        customProfileChar2[1] = i2cResult[0];
        //Y角速度
        i2CReadFromMPU(slaveAddress, 0x45, 2, i2cResult);
        customProfileChar2[2] = i2cResult[1];
        customProfileChar2[3] = i2cResult[0];
        //Z角速度
        i2CReadFromMPU(slaveAddress, 0x47, 2, i2cResult);
        customProfileChar2[4] = i2cResult[1];
        customProfileChar2[5] = i2cResult[0];
        */
        
        //前2字节：电源电压
        adcTempResult = getBatteryData();
        customProfileChar2[0] = (adcTempResult >> 8);
        customProfileChar2[1] = (adcTempResult % 256);
        
        //第3字节：连接状态
        // 0 - 正常工作中
        // 1 - 还能工作但电量不足了
        // 8 - 等待拍击以便进入正常工作状态
        // 9 - 因电量不足或其他故障而拒绝连接
        // 15 - 电量耗尽或其他故障而请求断开
        customProfileChar2[2] = (getWorkStatus() << 4);
        customProfileChar2[2] += VERSION_INFO;
        
        //第4字节：暂时显示为心跳机制计数
        customProfileChar2[3] = getHeartBeatCount();
        
        //后2字节：暂时未用
        //customProfileChar2[4] = 0;
        //customProfileChar2[5] = 0;
        //customProfileChar2[4] = (currentOutputVal >> 8);
        //customProfileChar2[5] = (currentOutputVal % 256);
        //*/
        
        //执行过读取动作后，心跳机制计数清零
        clearHeartBeatCount();
        
        *pLen = 4; // <-- 原6
        VOID memcpy(pValue, pAttr->pValue, 4);
        break;
        
      default:
        // Should never get here! (characteristics 3 and 4 do not have read permissions)
        *pLen = 0;
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else
  {
    // neither 16-bit UUID nor 128-bit UUID
    *pLen = 0;
    status = ATT_ERR_INVALID_HANDLE;
  }

  return ( status );
}

/*********************************************************************
 * @fn      simpleProfile_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @param   method - type of write message
 *
 * @return  SUCCESS, blePending or Failure
 */
static bStatus_t customProfile_WriteAttrCB(uint16_t connHandle,
                                           gattAttribute_t *pAttr,
                                           uint8_t *pValue, uint16_t len,
                                           uint16_t offset, uint8_t method)
{
  bStatus_t status = SUCCESS;
  uint8 notifyApp = 0xFF;
  uint16_t tempSystemOpt;
  
  uint8_t uuidLen = pAttr->type.len;
  if (uuidLen == ATT_BT_UUID_SIZE || uuidLen == ATT_UUID_SIZE)
  {
    // 16-bit UUID or 128-bit UUID
    uint16 uuid = 0;
    if(uuidLen == ATT_BT_UUID_SIZE){
      // 16-bit UUID
      uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    }else{
      // 128-bit UUID
      uuid = BUILD_UINT16( pAttr->type.uuid[12], pAttr->type.uuid[13]);
    }
    switch ( uuid )
    {
      case CUSTOMPROFILE_CHAR1_UUID:
        
        //Validate the value
        // Make sure it's not a blob oper
        if ( offset == 0 )
        {
          if ( len != CUSTOMPROFILE_CHAR1_LEN )
          {
            status = ATT_ERR_INVALID_VALUE_SIZE;
          }
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }
        
        //Write the value
        if ( status == SUCCESS )
        {
          uint16 *pCurValue = (uint16 *)pAttr->pValue;
          //0为高，1为低，不能反
          tempSystemOpt = pValue[0];
          tempSystemOpt <<= 8;
          tempSystemOpt += pValue[1];
          //写入系统设定
          setSyetemOption(tempSystemOpt);
          //最后给蓝牙模块传值
          *pCurValue = tempSystemOpt;
          notifyApp = CUSTOMPROFILE_CHAR1;
        }
             
        break;
      case CUSTOMPROFILE_CHAR3_UUID:

        //Validate the value
        // Make sure it's not a blob oper
        if ( offset == 0 )
        {
          if ( len != 1 )
          {
            status = ATT_ERR_INVALID_VALUE_SIZE;
          }
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }
        
        //Write the value
        if ( status == SUCCESS )
        {
          uint8 *pCurValue = (uint8 *)pAttr->pValue;
          *pCurValue = pValue[0];

          //往系统通道内的EEE3通道写入数据
          //systemOption = pValue[0];
          notifyApp = CUSTOMPROFILE_CHAR3;
          //修改系统设置（两位16进制数，后一位为陀螺仪工作状态，前一位为输出数据类型）
        }
             
        break;

      case GATT_CLIENT_CHAR_CFG_UUID:
        status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                                 offset, GATT_CLIENT_CFG_NOTIFY );
        break;
        
      default:
        // Should never get here! (characteristics 2 and 4 do not have write permissions)
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else
  {
    // neither 16-bit UUID nor 128-bit UUID
    status = ATT_ERR_INVALID_HANDLE;
  }

  // If a characteristic value changed then callback function to notify application of change
  if ( (notifyApp != 0xFF ) && customProfile_AppCBs && customProfile_AppCBs->pfnCustomProfileChange )
  {
    customProfile_AppCBs->pfnCustomProfileChange( notifyApp );  
  }
  
  return ( status );
}

/*********************************************************************
*********************************************************************/
