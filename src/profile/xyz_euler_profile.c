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
#include "packet.h"
#include "i2cRW.h"
#include "main.h"

#include "xyz_euler_profile.h"
#include "simple_peripheral.h"

#include "icall_ble_api.h"

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
CONST uint8 xyzEulerServUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(XYZ_EULER_SERV_UUID), HI_UINT16(XYZ_EULER_SERV_UUID)
};

// Characteristic 1 UUID: 0xFFF1
CONST uint8 xyzEulerchar1UUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(XYZ_EULER_CHAR1_UUID), HI_UINT16(XYZ_EULER_CHAR1_UUID)
};

// Characteristic 2 UUID: 0xFFF2
CONST uint8 xyzEulerchar2UUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(XYZ_EULER_CHAR2_UUID), HI_UINT16(XYZ_EULER_CHAR2_UUID)
};

// Characteristic 3 UUID: 0xFFF3
CONST uint8 xyzEulerchar3UUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(XYZ_EULER_CHAR3_UUID), HI_UINT16(XYZ_EULER_CHAR3_UUID)
};

// Characteristic 4 UUID: 0xFFF4
CONST uint8 xyzEulerchar4UUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(XYZ_EULER_CHAR4_UUID), HI_UINT16(XYZ_EULER_CHAR4_UUID)
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

uint8_t outputLength; // <-- 蓝牙数据输出长度
uint8_t outputOffset; // <-- 数据地址偏移量
static xyzEulerCBs_t *xyzEuler_AppCBs = NULL;

uint8_t outerMPUisRaw = 1;
uint8_t runWithOuterMPU = 0;
//void setRunWithOuterMPU(uint8_t runWith);

/*********************************************************************
 * Profile Attributes - variables
 */

// Simple Profile Service attribute
static CONST gattAttrType_t xyzEulerService = { ATT_BT_UUID_SIZE, xyzEulerServUUID };


// Simple Profile Characteristic 1 Properties
static uint8 xyzEulerChar1Props = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic 1 Value
static uint8 xyzEulerChar1 = 0;

// Simple Profile Characteristic 1 User Description
//static uint8 simpleProfileChar1UserDesp[17] = "Characteristic 1";
static uint8 xyzEulerChar1UserDesp[17] = "Char 1 - R/W Fun";


// Simple Profile Characteristic 2 Properties
static uint8 xyzEulerChar2Props = GATT_PROP_READ;

// Characteristic 2 Value
static uint8 xyzEulerChar2[XYZ_EULER_CHAR5_LEN] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

// Simple Profile Characteristic 2 User Description
//static uint8 simpleProfileChar2UserDesp[17] = "Characteristic 2";
static uint8 xyzEulerChar2UserDesp[17] = "Char 2 - Read On";


// Simple Profile Characteristic 3 Properties
static uint8 xyzEulerChar3Props = GATT_PROP_WRITE;

// Characteristic 3 Value
static uint8 xyzEulerChar3 = 0;

// Simple Profile Characteristic 3 User Description
//static uint8 simpleProfileChar3UserDesp[17] = "Characteristic 3";
static uint8 xyzEulerChar3UserDesp[17] = "Char 3 - Write F";


// Simple Profile Characteristic 4 Properties
static uint8 xyzEulerChar4Props = GATT_PROP_NOTIFY;

// Characteristic 4 Value
static uint8 xyzEulerChar4 = 0;

// Simple Profile Characteristic 4 Configuration Each client has its own
// instantiation of the Client Characteristic Configuration. Reads of the
// Client Characteristic Configuration only shows the configuration for
// that client and writes only affect the configuration of that client.
static gattCharCfg_t *xyzEulerChar4Config;
                                        
// Simple Profile Characteristic 4 User Description
//static uint8 simpleProfileChar4UserDesp[17] = "Characteristic 4";
static uint8 xyzEulerChar4UserDesp[17] = "Notify Motion";

//=================================================================================================

/*********************************************************************
 * Profile Attributes - Table
 */
//这里最关键：注册蓝牙通道的数量
static gattAttribute_t xyzEulerAttrTbl[SERVAPP_NUM_ATTR_SUPPORTED] = 
{
  // Simple Profile Service
  { 
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&xyzEulerService            /* pValue */
  },
    
    //====================================================================================
    // Characteristic 1 Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &xyzEulerChar1Props 
    },

      // Characteristic Value 1
      { 
        { ATT_BT_UUID_SIZE, xyzEulerchar1UUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        &xyzEulerChar1 
      },

      // Characteristic 1 User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        xyzEulerChar1UserDesp 
      },      

    //====================================================================================
    // Characteristic 2 Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &xyzEulerChar2Props 
    },

      // Characteristic Value 2
      { 
        { ATT_BT_UUID_SIZE, xyzEulerchar2UUID },
        GATT_PERMIT_READ, 
        0, 
        xyzEulerChar2 
      },

      // Characteristic 2 User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        xyzEulerChar2UserDesp 
      },           
      
    //====================================================================================
    // Characteristic 3 Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &xyzEulerChar3Props 
    },

      // Characteristic Value 3
      { 
        { ATT_BT_UUID_SIZE, xyzEulerchar3UUID },
        GATT_PERMIT_WRITE, 
        0, 
        &xyzEulerChar3 
      },

      // Characteristic 3 User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        xyzEulerChar3UserDesp 
      },
    
    //====================================================================================
      
    // Characteristic 4 Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &xyzEulerChar4Props 
    },

      // Characteristic Value 4
      { 
        { ATT_BT_UUID_SIZE, xyzEulerchar4UUID },
        0, 
        0, 
        &xyzEulerChar4 
      },

      // Characteristic 4 configuration
      { 
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        (uint8 *)&xyzEulerChar4Config 
      },
      
      // Characteristic 4 User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        xyzEulerChar4UserDesp 
      }
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t xyzEuler_ReadAttrCB(uint16_t connHandle,
                                          gattAttribute_t *pAttr, 
                                          uint8_t *pValue, uint16_t *pLen,
                                          uint16_t offset, uint16_t maxLen,
                                          uint8_t method);
static bStatus_t xyzEuler_WriteAttrCB(uint16_t connHandle,
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
CONST gattServiceCBs_t xyzEulerCBs =
{
  xyzEuler_ReadAttrCB,  // Read callback function pointer
  xyzEuler_WriteAttrCB, // Write callback function pointer
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
bStatus_t XYZEuler_AddService( uint32 services )
{
  uint8 status;

  // Allocate Client Characteristic Configuration table
  xyzEulerChar4Config = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) *
                                                            linkDBNumConns );
  if ( xyzEulerChar4Config == NULL )
  {     
    return ( bleMemAllocError );
  }
  
  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, xyzEulerChar4Config );
  
  if ( services & XYZ_EULER_SERVICE )
  {
    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService( xyzEulerAttrTbl, 
                                          GATT_NUM_ATTRS( xyzEulerAttrTbl ),
                                          GATT_MAX_ENCRYPT_KEY_SIZE,
                                          &xyzEulerCBs );
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
bStatus_t XYZEuler_RegisterAppCBs( xyzEulerCBs_t *appCallbacks )
{
  if ( appCallbacks )
  {
    xyzEuler_AppCBs = appCallbacks;
    
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
bStatus_t XYZEuler_SetParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case XYZ_EULER_CHAR1:
      if ( len == sizeof ( uint8 ) ) 
      {
        xyzEulerChar1 = *((uint8*)value);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case XYZ_EULER_CHAR2:
      if ( len == XYZ_EULER_CHAR5_LEN ) 
      {
        VOID memcpy( xyzEulerChar2, value, XYZ_EULER_CHAR5_LEN );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case XYZ_EULER_CHAR3:
      if ( len == sizeof ( uint8 ) ) 
      {
        xyzEulerChar3 = *((uint8*)value);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case XYZ_EULER_CHAR4:
      if ( len == sizeof ( uint8 ) ) 
      {
        xyzEulerChar4 = *((uint8*)value);
        
        // See if Notification has been enabled
        GATTServApp_ProcessCharCfg( xyzEulerChar4Config, &xyzEulerChar4, FALSE,
                                    xyzEulerAttrTbl, GATT_NUM_ATTRS( xyzEulerAttrTbl ),
                                    INVALID_TASK_ID, xyzEuler_ReadAttrCB );
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
bStatus_t XYZEuler_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case XYZ_EULER_CHAR1:
      xyzEulerChar1 = getMPLFreq();
      *((uint8*)value) = xyzEulerChar1;
      break;

    case XYZ_EULER_CHAR2:
      VOID memcpy( value, xyzEulerChar2, XYZ_EULER_CHAR5_LEN );
      break;

    case XYZ_EULER_CHAR3:
      *((uint8*)value) = xyzEulerChar3;
      break;  

    case XYZ_EULER_CHAR4:
      //*((uint8*)value) = xyzEulerChar4;
      VOID memcpy( value, xyzEulerChar2, XYZ_EULER_CHAR5_LEN );
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
static bStatus_t xyzEuler_ReadAttrCB(uint16_t connHandle,
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
  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    char mplResult[23] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    uint8_t i;
    switch ( uuid )
    {
      // No need for "GATT_SERVICE_UUID" or "GATT_CLIENT_CHAR_CFG_UUID" cases;
      // gattserverapp handles those reads

      // characteristics 1 and 2 have read permissions
      // characteritisc 3 does not have read permissions; therefore it is not
      //   included here
      // characteristic 4 does not have read permissions, but because it
      //   can be sent as a notification, it is included here
      case XYZ_EULER_CHAR1_UUID:
      case XYZ_EULER_CHAR6_UUID:
        //此处修改蓝牙输出长度？
        *pLen = 1; // <-- 1字节8位, 2字节16位
        pValue[0] = *pAttr->pValue;
        break;
        
      case XYZ_EULER_CHAR4_UUID:
        
        //这个地方最重要，是MPU的对外输出功能！===================================
        
        getBleOutData(mplResult);
        for(i = 0; i < XYZ_EULER_CHAR5_LEN; i++){
          xyzEulerChar2[i] = mplResult[i];
        }
        //最终输出
        *pLen = outputLength; // <-- 决定输出长度
        VOID memcpy(pValue - 2 - OUTPUT_BYTE_OFFSET, pAttr->pValue, XYZ_EULER_CHAR5_LEN); // <-- 决定输出地址
        
        break;
        
        //========================================================================
        
      case XYZ_EULER_CHAR2_UUID:
        //将MPL数据输出区的东西考过来
        getBleOutData(mplResult);
        for(i = 0; i < XYZ_EULER_CHAR5_LEN; i++){
          xyzEulerChar2[i] = mplResult[i];
        }
        
        //最终输出
        *pLen = outputLength; // <-- 决定输出长度
        //*pLen = XYZ_EULER_CHAR5_LEN;
        VOID memcpy(pValue - 4 - OUTPUT_BYTE_OFFSET, pAttr->pValue, XYZ_EULER_CHAR5_LEN); // <-- 决定输出地址
        //VOID memcpy(pValue - 2, pAttr->pValue, outputLength);
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
    // 128-bit UUID
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
static bStatus_t xyzEuler_WriteAttrCB(uint16_t connHandle,
                                           gattAttribute_t *pAttr,
                                           uint8_t *pValue, uint16_t len,
                                           uint16_t offset, uint8_t method)
{
  bStatus_t status = SUCCESS;
  uint8 notifyApp = 0xFF;
  
  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {
      case XYZ_EULER_CHAR1_UUID:
      case XYZ_EULER_CHAR3_UUID:

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
          uint8 valueMPL = (pValue[0] & 63); // <-- MPL功能：取该值的后六位
          uint8 valueBLE = (pValue[0] & 192); // <-- 特殊功能：取该值的前两位（不会反映在read功能中，注意）
          //*pCurValue = pValue[0];

          if( pAttr->pValue == &xyzEulerChar1 )
          {
            //UUID1多一个特有步骤：改写MPL采样频率、改变信号增益强度
            
            //步骤1：分析MPL采样频率
            if(valueMPL < MPL_FREQ_MIN){
              valueMPL = MPL_FREQ_MIN;
            }
            if(valueMPL > MPL_FREQ_MAX){
              valueMPL = MPL_FREQ_MAX;
            }
            *pCurValue = valueMPL;
            setMPLFreq(valueMPL);
            
            //步骤2：分析蓝牙增益位
            if(valueBLE & 64 > 0){
              //第6位置高：高增益接收
              changeRxLevel(HIGH_RX);
            }else{
              changeRxLevel(LOW_RX);
            }
            if(valueBLE & 128 > 0){
              //第7位置高：高功率发送
              changeTxLevel(HIGH_TX);
            }else{
              changeTxLevel(LOW_TX);
            }
            
            //步骤3：通知程序，这个地方有改变
            notifyApp = XYZ_EULER_CHAR1;
          }
          else
          {
            *pCurValue = pValue[0];
            notifyApp = XYZ_EULER_CHAR3;           
          }
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
    // 128-bit UUID
    status = ATT_ERR_INVALID_HANDLE;
  }

  // If a characteristic value changed then callback function to notify application of change
  if ( (notifyApp != 0xFF ) && xyzEuler_AppCBs && xyzEuler_AppCBs->pfnXYZEulerChange )
  {
    xyzEuler_AppCBs->pfnXYZEulerChange( notifyApp );  
  }
  
  return ( status );
}

//修改蓝牙通道输出数据的类型和长度(初始化时必须调用一次)
void changeOutputValueSort(uint8_t valueSort){
  switch(valueSort){
  case OUTPUT_QUAT:
    outputLength = 17;
    break;
  case OUTPUT_ACCEL:
  case OUTPUT_LIACCEL:
  case OUTPUT_GVECTOR:
  case OUTPUT_GYRO:
  case OUTPUT_EULER:
  case OUTPUT_COMPASS:
    outputLength = 13;
    break;
  case OUTPUT_ROT:
    outputLength = 19;
    break;
  case OUTPUT_HEADING:
    outputLength = 5;
    break;
  default:
    outputLength = 1;
    break;
  }
  outputOffset = XYZ_EULER_CHAR5_LEN - outputLength;
}

/*********************************************************************
*********************************************************************/
