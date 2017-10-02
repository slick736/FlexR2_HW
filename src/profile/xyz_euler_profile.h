/******************************************************************************

 @file  simple_gatt_profile.h

 @brief This file contains the Simple GATT profile definitions and prototypes
        prototypes.

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

#ifndef XYZEULERPROFILE_H
#define XYZEULERPROFILE_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */

//�������������
#define OUTPUT_ACCEL                      0
#define OUTPUT_GYRO                       1
#define OUTPUT_COMPASS                    2
#define OUTPUT_EULER                      3
#define OUTPUT_ROT                        4
#define OUTPUT_QUAT                       5
#define OUTPUT_LIACCEL                    6
#define OUTPUT_GVECTOR                    7
#define OUTPUT_HEADING                    8

// Profile Parameters
#define XYZ_EULER_CHAR1                   0  // RW uint8 - Profile Characteristic 1 value 
#define XYZ_EULER_CHAR2                   1  // RW uint8 - Profile Characteristic 2 value
#define XYZ_EULER_CHAR3                   2  // RW uint8 - Profile Characteristic 3 value
#define XYZ_EULER_CHAR4                   3  // RW uint8 - Profile Characteristic 4 value
#define XYZ_EULER_CHAR5                   4  // RW uint8 - Profile Characteristic 5 value
#define XYZ_EULER_CHAR6                   5  // RW uint8 - Profile Characteristic 6 value
  
// Simple Profile Service UUID
#define XYZ_EULER_SERV_UUID               0xCCC0
#define XYZ_EULER_SERVSLAVE_UUID          0xCC00
    
// Key Pressed UUID
#define XYZ_EULER_CHAR1_UUID            0xCCC1
#define XYZ_EULER_CHAR2_UUID            0xCCC2
#define XYZ_EULER_CHAR3_UUID            0xCCC3
#define XYZ_EULER_CHAR4_UUID            0xCCC4
#define XYZ_EULER_CHAR5_UUID            0xCCC5
#define XYZ_EULER_CHAR6_UUID            0xCCC6
  
// Simple Keys Profile Services bit fields
#define XYZ_EULER_SERVICE               0x00000001

// Length of Characteristic 5 in bytes
#define XYZ_EULER_CHAR5_LEN           23  

/*********************************************************************
 * TYPEDEFS
 */

  
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */

// Callback when a characteristic value has changed
typedef void (*xyzEulerChange_t)( uint8 paramID );

typedef struct
{
  xyzEulerChange_t        pfnXYZEulerChange;  // Called when characteristic value changes
} xyzEulerCBs_t;

    

/*********************************************************************
 * API FUNCTIONS 
 */


/*
 * SimpleProfile_AddService- Initializes the Simple GATT Profile service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 */

extern bStatus_t XYZEuler_AddService( uint32 services );

/*
 * SimpleProfile_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t XYZEuler_RegisterAppCBs( xyzEulerCBs_t *appCallbacks );

/*
 * SimpleProfile_SetParameter - Set a Simple GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 */
extern bStatus_t XYZEuler_SetParameter( uint8 param, uint8 len, void *value );
  
/*
 * SimpleProfile_GetParameter - Get a Simple GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 */
extern bStatus_t XYZEuler_GetParameter( uint8 param, void *value );

//����ӿڣ��ı�������ݳ���
extern void changeOutputValueSort(uint8_t valueSort);

//extern void setRunWithOuterMPU(uint8_t runWith);
extern uint8_t isRunningWithOuterMPU(void);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SIMPLEGATTPROFILE_H */
