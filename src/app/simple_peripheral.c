/******************************************************************************

 @file  simple_peripheral.c

 @brief This file contains the Simple BLE Peripheral sample application for use
        with the CC2650 Bluetooth Low Energy Protocol Stack.

 Group: CMCU, SCS
 Target Device: CC2640R2

 ******************************************************************************
 
 Copyright (c) 2013-2017, Texas Instruments Incorporated
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
 Release Date: 2017-03-02 20:08:31
 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>

#include "hci_tl.h"
#include "hci.h"
#include "gatt.h"
#include "linkdb.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "simple_gatt_profile.h"
#include "custom_gatt_profile.h"
#include "xyz_euler_profile.h"
#include "ll_common.h"
#include "delay.h"
#include "stdio.h"
#include "i2cRW.h"
#include "packet.h"
#include "ex_include_tirtos.h"
#include "scif.h"
#include "gpioCustom.h"
#include "peripheral.h"
#include "main.h"
#include "delay.h"

#include <ti/sysbios/knl/Swi.h>

//Semaphore_Handle sem;
Swi_Handle swi;

//系统设定
extern uint8_t enableReadMPL;
extern uint8_t enableReadADC;
uint8_t BLE_PAUSED;
void meDisableBLEConnect(void);
extern uint8_t lpaLampLoop = 0;
extern void changeRxLevel(uint8_t neoRxLevel);
extern void changeTxLevel(uint8_t neoTxLevel);

//是否允许不经过陀螺仪同意而自动连接
extern uint8_t enableAutoConnect = 0;

//是否允许该设备连接蓝牙
uint8_t enableConnectBLE;
uint8_t terminateConnectionWithDisabled; // <-- 终结连接的时候是否将enableConnectBLE也置0
//变量：是否为外部因素挂断
uint8_t disconnectByInterrupt = 0;

#if defined(FEATURE_OAD) || defined(IMAGE_INVALIDATE)
#include "oad_target.h"
#include "oad.h"
#endif //FEATURE_OAD || IMAGE_INVALIDATE

#include "peripheral.h"
#include "gapbondmgr.h"

#include "osal_snv.h"
#include "icall_ble_apimsg.h"

#include "util.h"

#ifdef USE_RCOSC
#include "rcosc_calibration.h"
#endif //USE_RCOSC

#ifdef USE_CORE_SDK
  #include <ti/display/Display.h>
#else // !USE_CORE_SDK
  #include <ti/mw/display/Display.h>
#endif // USE_CORE_SDK
#include "board_key.h"

#include "board.h"

#include "simple_peripheral.h"

#if defined( USE_FPGA ) || defined( DEBUG_SW_TRACE )
#include <driverlib/ioc.h>
#endif // USE_FPGA | DEBUG_SW_TRACE

#include "icall_ble_api.h"
#include <ti/drivers/ADC.h>
/*********************************************************************
 * CONSTANTS
 */

#define ENABLE_GYRO                           1

// Advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160
//#define DEFAULT_ADVERTISING_INTERVAL          4800 // <-- 原本是100ms发一次Advertise，现在改为3s一次

//开机时的维持时间
#define ADVERT_AT_POWERON                     4         // 4 seconds

//每次广播的维持时间（过了广播维持时间后就自动跌入0状态）
//#define ADVERT_DURATION                       10         // 10 seconds
#define ADVERT_DURATION                       1
#define ADVERT_LONG_DURATION                  10
//两次广播中间的歇息时间（过了歇息时间后自动升入1状态）
#define ADVERT_SLEEP_DURATION                 3            // 3 seconds

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely
//#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_LIMITED

#ifndef FEATURE_OAD
// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     80

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     800
#else //!FEATURE_OAD
// Minimum connection interval (units of 1.25ms, 8=10ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     8

// Maximum connection interval (units of 1.25ms, 8=10ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     8
#endif // FEATURE_OAD

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter
// update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000

// Whether to enable automatic parameter update request when a connection is
// formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         GAPROLE_LINK_PARAM_UPDATE_WAIT_REMOTE_PARAMS

// Connection Pause Peripheral time value (in seconds)
   //建立连接后要休息6秒？
#define DEFAULT_CONN_PAUSE_PERIPHERAL         6

// How often to perform periodic event (in msec)
#define SBP_PERIODIC_EVT_PERIOD               25 // 25, <-- Original 5000
//#define SBP_PERIODIC_EVT_PERIOD               500 // <-- 1秒1次ADC与MPU交替循环
#define SBP_INITIALIZE_EVT_PERIOD             200 // <-- 执行初始化时，每个工作步骤间隔200ms

// Application specific event ID for HCI Connection Event End Events
#define SBP_HCI_CONN_EVT_END_EVT              0x0001

// Type of Display to open
#if !defined(Display_DISABLE_ALL)
  #ifdef USE_CORE_SDK
    #if defined(BOARD_DISPLAY_USE_LCD) && (BOARD_DISPLAY_USE_LCD!=0)
      #define SBP_DISPLAY_TYPE Display_Type_LCD
    #elif defined (BOARD_DISPLAY_USE_UART) && (BOARD_DISPLAY_USE_UART!=0)
      #define SBP_DISPLAY_TYPE Display_Type_UART
    #else // !BOARD_DISPLAY_USE_LCD && !BOARD_DISPLAY_USE_UART
      #define SBP_DISPLAY_TYPE 0 // Option not supported
    #endif // BOARD_DISPLAY_USE_LCD && BOARD_DISPLAY_USE_UART
  #else // !USE_CORE_SDK
    #if !defined(BOARD_DISPLAY_EXCLUDE_LCD)
      #define SBP_DISPLAY_TYPE Display_Type_LCD
    #elif !defined (BOARD_DISPLAY_EXCLUDE_UART)
      #define SBP_DISPLAY_TYPE Display_Type_UART
    #else // BOARD_DISPLAY_EXCLUDE_LCD && BOARD_DISPLAY_EXCLUDE_UART
      #define SBP_DISPLAY_TYPE 0 // Option not supported
    #endif // !BOARD_DISPLAY_EXCLUDE_LCD || !BOARD_DISPLAY_EXCLUDE_UART
  #endif // USE_CORE_SDK
#else // BOARD_DISPLAY_USE_LCD && BOARD_DISPLAY_USE_UART
  #define SBP_DISPLAY_TYPE 0 // No Display
#endif // Display_DISABLE_ALL

#ifdef FEATURE_OAD
// The size of an OAD packet.
#define OAD_PACKET_SIZE                       ((OAD_BLOCK_SIZE) + 2)
#endif // FEATURE_OAD

// Task configuration
#define SBP_TASK_PRIORITY                     1

#ifndef SBP_TASK_STACK_SIZE
#define SBP_TASK_STACK_SIZE                   644
#endif

#define SBP_STATE_CHANGE_EVT                  0x0001
#define SBP_CHAR_CHANGE_EVT                   0x0002
#define SBP_CUSTOM_CHAR_CHANGE_EVT            0x0004
#define SBP_EULER_CHAR_CHANGE_EVT             0x0008

// Internal Events for RTOS application
#define SBP_ICALL_EVT                         ICALL_MSG_EVENT_ID // Event_Id_31
#define SBP_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30
#define SBP_PERIODIC_EVT                      Event_Id_00
//将“初始化”时钟事件提到最前
#define SBP_INITIALIZE_EVT                    Event_Id_02

#ifdef FEATURE_OAD
#define SBP_QUEUE_PING_EVT                    Event_Id_01

#define SBP_ALL_EVENTS                        (SBP_ICALL_EVT        | \
                                               SBP_QUEUE_EVT        | \
                                               SBP_PERIODIC_EVT     | \
                                               SBP_INITIALIZE_EVT   | \
                                               SBP_QUEUE_PING_EVT)
#else
#define SBP_ALL_EVENTS                        (SBP_ICALL_EVT        | \
                                               SBP_QUEUE_EVT        | \
                                               SBP_INITIALIZE_EVT   | \
                                               SBP_PERIODIC_EVT)
#endif /* FEATURE_OAD */

/*********************************************************************
 * TYPEDEFS
 */

// App event passed from profiles.
typedef struct
{
  appEvtHdr_t hdr;  // event header.
} sbpEvt_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Display Interface
Display_Handle dispHandle = NULL;

//标志位：是否该系统被预告蓝牙即将关闭
extern uint8_t bleIsToClose = 0;

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;

// Clock instances for internal periodic events.
static Clock_Struct periodicClock;
static Clock_Struct initializeClock;
static Clock_Struct advertiseClock;
static Clock_Struct advertiseShortClock;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

#if defined(FEATURE_OAD)
// Event data from OAD profile.
static Queue_Struct oadQ;
static Queue_Handle hOadQ;
#endif //FEATURE_OAD

// Task configuration
Task_Struct sbpTask;
Char sbpTaskStack[SBP_TASK_STACK_SIZE];

// Profile state and parameters
//static gaprole_States_t gapProfileState = GAPROLE_INIT;

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8_t scanRspData[] =
{
  // complete name
  0x09,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'F',
  'l',
  'e',
  'x',
  'R',
  ' ',
  'I',
  'I',

  // connection interval range
  0x05,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),   // 100ms
  HI_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
  LO_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),   // 1s
  HI_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),

  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8_t advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // service UUID, to notify central devices what services are included
  // in this peripheral
  // 提醒主机，都有什么Service可以连上
  
#if !defined(FEATURE_OAD) || defined(FEATURE_OAD_ONCHIP)
  0x07,   // length of this data
#else //OAD for external flash
  0x09,   // length of this data
#endif //FEATURE_OAD
  
  GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
#ifdef FEATURE_OAD
  LO_UINT16(OAD_SERVICE_UUID),
  HI_UINT16(OAD_SERVICE_UUID),
#endif //FEATURE_OAD

//给哪个UUID打广告？
#ifndef FEATURE_OAD_ONCHIP
  LO_UINT16(SIMPLEPROFILE_SERV_UUID),
  HI_UINT16(SIMPLEPROFILE_SERV_UUID),
  LO_UINT16(CUSTOMPROFILE_SERV_UUID),
  HI_UINT16(CUSTOMPROFILE_SERV_UUID),
  LO_UINT16(XYZ_EULER_SERV_UUID),
  HI_UINT16(XYZ_EULER_SERV_UUID)
#endif //FEATURE_OAD_ONCHIP
};

// GAP GATT Attributes
//static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "FlexR II";
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = DEVICE_NAME;

// Globals used for ATT Response retransmission
static gattMsgEvent_t *pAttRsp = NULL;
static uint8_t rspTxRetry = 0;

//Flag变量：蓝牙断开原因是人为还是Advertise时间到
//0-时间到 1-人为
uint8_t bleStopReason = 0;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void SimpleBLEPeripheral_init( void );
static void SimpleBLEPeripheral_taskFxn(UArg a0, UArg a1);

static uint8_t SimpleBLEPeripheral_processStackMsg(ICall_Hdr *pMsg);
static uint8_t SimpleBLEPeripheral_processGATTMsg(gattMsgEvent_t *pMsg);

static void SimpleBLEPeripheral_processAppMsg(sbpEvt_t *pMsg);
static void SimpleBLEPeripheral_processStateChangeEvt(gaprole_States_t newState);

static void SimpleBLEPeripheral_processCharValueChangeEvt(uint8_t paramID);
static void CustomPeripheral_processCharValueChangeEvt(uint8_t paramID);
static void EulerPeripheral_processCharValueChangeEvt(uint8_t paramID);

static void SimpleBLEPeripheral_performPeriodicTask(void);
static void SimpleBLEPeripheral_clockHandler(UArg arg);

static void SimpleBLEPeripheral_performInitializeTask(void);
static void SimpleBLEPeripheral_initializeHandler(UArg arg);

static void SimpleBLEPeripheral_advertiseHandler(UArg arg);

static void SimpleBLEPeripheral_sendAttRsp(void);
static void SimpleBLEPeripheral_freeAttRsp(uint8_t status);

static void SimpleBLEPeripheral_stateChangeCB(gaprole_States_t newState);

#ifndef FEATURE_OAD_ONCHIP
static void SimpleBLEPeripheral_charValueChangeCB(uint8_t paramID);
static void CustomPeripheral_charValueChangeCB(uint8_t paramID);
static void EulerPeripheral_charValueChangeCB(uint8_t paramID);
#endif //!FEATURE_OAD_ONCHIP

static void SimpleBLEPeripheral_enqueueMsg(uint8_t event, uint8_t state);

#ifdef FEATURE_OAD
void SimpleBLEPeripheral_processOadWriteCB(uint8_t event, uint16_t connHandle,
                                           uint8_t *pData);
#endif //FEATURE_OAD

/*********************************************************************
 * EXTERN FUNCTIONS
 */
extern void AssertHandler(uint8 assertCause, uint8 assertSubcause);

//ADC相关变量
//extern含义：这个变量是别处定义的，但我要在这儿用
extern uint8_t adcInitResult;
extern uint8_t adcConvertResult;
//extern ADC_Handle   adc;
//extern ADC_Params   params;
extern int_fast16_t res;
/* ADC conversion result variables */
uint16_t adcValue0Result;

//ADC积分相关算法
uint8_t adcStackIndi = 0;
uint8_t adcStackPointer = 0;
uint16_t adcSamples[ADC_SMOOTH_FORCE_MAX];

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t SimpleBLEPeripheral_gapRoleCBs =
{
  SimpleBLEPeripheral_stateChangeCB     // Profile State Change Callbacks
};

// GAP Bond Manager Callbacks
static gapBondCBs_t simpleBLEPeripheral_BondMgrCBs =
{
  NULL, // Passcode callback (not used by application)
  NULL  // Pairing / Bonding state Callback (not used by application)
};

// Simple GATT Profile Callbacks
#ifndef FEATURE_OAD_ONCHIP
// <-- 注意这里要将你所有的Service通道全都写上
static simpleProfileCBs_t SimpleBLEPeripheral_simpleProfileCBs =
{
  SimpleBLEPeripheral_charValueChangeCB // Characteristic value change callback
};
static customProfileCBs_t SimpleBLEPeripheral_customProfileCBs =
{
  CustomPeripheral_charValueChangeCB // Characteristic value change callback
};
static xyzEulerCBs_t SimpleBLEPeripheral_xyzEulerCBs =
{
  EulerPeripheral_charValueChangeCB // Characteristic value change callback
};
#endif //!FEATURE_OAD_ONCHIP

#ifdef FEATURE_OAD
static oadTargetCBs_t simpleBLEPeripheral_oadCBs =
{
  SimpleBLEPeripheral_processOadWriteCB // Write Callback.
};
#endif //FEATURE_OAD

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleBLEPeripheral_createTask
 *
 * @brief   Task creation function for the Simple BLE Peripheral.
 *
 * @param   None.
 *
 * @return  None.
 */
void SimpleBLEPeripheral_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = sbpTaskStack;
  taskParams.stackSize = SBP_TASK_STACK_SIZE;
  taskParams.priority = SBP_TASK_PRIORITY;

  Task_construct(&sbpTask, SimpleBLEPeripheral_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_init
 *
 * @brief   Called during initialization and contains application
 *          specific initialization (ie. hardware initialization/setup,
 *          table initialization, power up notification, etc), and
 *          profile initialization/setup.
 *
 * @param   None.
 *
 * @return  None.
 */

static void SimpleBLEPeripheral_init(void)
{
  // ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &syncEvent);

#ifdef USE_RCOSC
  RCOSC_enableCalibration();
#endif // USE_RCOSC

#if defined( USE_FPGA )
  // configure RF Core SMI Data Link
  IOCPortConfigureSet(IOID_12, IOC_PORT_RFC_GPO0, IOC_STD_OUTPUT);
  IOCPortConfigureSet(IOID_11, IOC_PORT_RFC_GPI0, IOC_STD_INPUT);

  // configure RF Core SMI Command Link
  IOCPortConfigureSet(IOID_10, IOC_IOCFG0_PORT_ID_RFC_SMI_CL_OUT, IOC_STD_OUTPUT);
  IOCPortConfigureSet(IOID_9, IOC_IOCFG0_PORT_ID_RFC_SMI_CL_IN, IOC_STD_INPUT);

  // configure RF Core tracer IO
  IOCPortConfigureSet(IOID_8, IOC_PORT_RFC_TRC, IOC_STD_OUTPUT);
#else // !USE_FPGA
  #if defined( DEBUG_SW_TRACE )
    // configure RF Core tracer IO
    IOCPortConfigureSet(IOID_8, IOC_PORT_RFC_TRC, IOC_STD_OUTPUT | IOC_CURRENT_4MA | IOC_SLEW_ENABLE);
  #endif // DEBUG_SW_TRACE
#endif // USE_FPGA

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);

  // Create one-shot clocks for internal periodic events.
  Util_constructClock(&periodicClock, SimpleBLEPeripheral_clockHandler,
                      SBP_PERIODIC_EVT_PERIOD, 0, false, SBP_PERIODIC_EVT);
  // Create one-shot clocks for initialization events.
  Util_constructClock(&initializeClock, SimpleBLEPeripheral_initializeHandler,
                      SBP_INITIALIZE_EVT_PERIOD, 0, false, SBP_INITIALIZE_EVT);
  // Create one-shot clocks for advertisement events.
  Util_constructClock(&advertiseClock, SimpleBLEPeripheral_advertiseHandler,
                      1000 * ADVERT_SLEEP_DURATION, 0, false, NULL);
  Util_constructClock(&advertiseShortClock, SimpleBLEPeripheral_advertiseHandler,
                      100, 0, false, NULL);

  //display功能不需要，这里屏蔽
  //dispHandle = Display_open(SBP_DISPLAY_TYPE, NULL);

  // Setup the GAP
  GAP_SetParamValue(TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL);

  // Setup the GAP Peripheral Role Profile
  {
    // For all hardware platforms, device starts advertising upon initialization
    uint8_t initialAdvertEnable = TRUE;

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16_t advertOffTime = 0;

    uint8_t enableUpdateRequest = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16_t desiredMinInterval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16_t desiredMaxInterval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16_t desiredSlaveLatency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16_t desiredConnTimeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    // Set the GAP Role Parameters
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                         &initialAdvertEnable);
    GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16_t),
                         &advertOffTime);

    GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData),
                         scanRspData);
    GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);

    GAPRole_SetParameter(GAPROLE_PARAM_UPDATE_ENABLE, sizeof(uint8_t),
                         &enableUpdateRequest);
    GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof(uint16_t),
                         &desiredMinInterval);
    GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, sizeof(uint16_t),
                         &desiredMaxInterval);
    GAPRole_SetParameter(GAPROLE_SLAVE_LATENCY, sizeof(uint16_t),
                         &desiredSlaveLatency);
    GAPRole_SetParameter(GAPROLE_TIMEOUT_MULTIPLIER, sizeof(uint16_t),
                         &desiredConnTimeout);
  }

  // Set the GAP Characteristics
  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

  // Set advertising interval
  {
    uint16_t advInt = DEFAULT_ADVERTISING_INTERVAL;
    uint16_t advDuration = ADVERT_AT_POWERON;
    
    GAP_SetParamValue(TGAP_LIM_ADV_TIMEOUT, advDuration);
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);
  }

  // Setup the GAP Bond Manager
  {
    uint32_t passkey = 0; // passkey "000000" <-- 注意这里是配对用的密码，目前取值000000
    uint8_t pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    uint8_t mitm = TRUE;
    uint8_t ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    uint8_t bonding = TRUE;

    GAPBondMgr_SetParameter(GAPBOND_DEFAULT_PASSCODE, sizeof(uint32_t),
                            &passkey);
    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
    GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
    GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
    GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);
  }

   // Initialize GATT attributes
  GGS_AddService(GATT_ALL_SERVICES);           // GAP
  GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT attributes
  DevInfo_AddService();                        // Device Information Service

#ifndef FEATURE_OAD_ONCHIP
  //在此处多加两三个Service
  //注意程序结构和Service的对应关系：profile内一组.c和.h两个文件对应一个Service
  //Service内部的Characteristic应该全部在AttrTable内注册过才能使用
  //XYZEuler_AddService(GATT_ALL_SERVICES); // Simple GATT Profile
  SimpleProfile_AddService(GATT_ALL_SERVICES); // Simple GATT Profile
  CustomProfile_AddService(GATT_ALL_SERVICES); // Simple GATT Profile
  XYZEuler_AddService(GATT_ALL_SERVICES); // Simple GATT Profile
#endif //!FEATURE_OAD_ONCHIP

#ifdef FEATURE_OAD
  VOID OAD_addService();                 // OAD Profile
  OAD_register((oadTargetCBs_t *)&simpleBLEPeripheral_oadCBs);
  hOadQ = Util_constructQueue(&oadQ);
#endif //FEATURE_OAD

#ifdef IMAGE_INVALIDATE
  Reset_addService();
#endif //IMAGE_INVALIDATE


#ifndef FEATURE_OAD_ONCHIP
  // 蓝牙初始化的时候写进什么值？
  {
    uint8_t charValue1 = 5;
    uint8_t charValue2 = adcInitResult; // <-- 原6; // <-- 要改为adcInitResult;
    uint8_t charValue3 = 7;
    uint8_t charValue4 = 8; //adcConvertResult; // <-- 原8; // <-- 要改为adcConvertResult;
    uint8_t charValue5[SIMPLEPROFILE_CHAR5_LEN] = { 1, 2, 3, 4, 5, 6 };
    uint8_t charValue6 = 9;
    uint8_t emgInitRate = DEFAULT_EMG_FREQ;
    uint8_t motionInitRate = DEFAULT_MOTION_FREQ;
    //I2C初始化
    initI2C();
    //MPL输出区初始化
    initBleOutData();
    //GPIO初始化
    PIN_Initialize();
    
    //EMG
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR1, sizeof(uint8_t), &emgInitRate);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR2, sizeof(uint8_t), &charValue2);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR3, sizeof(uint8_t), &charValue3);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, sizeof(uint8_t), &charValue4);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR5, SIMPLEPROFILE_CHAR5_LEN, charValue5);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR6, sizeof(uint16_t), &charValue6);
    
    //系统
    CustomProfile_SetParameter(CUSTOMPROFILE_CHAR1, sizeof(uint8_t), &charValue4);
    CustomProfile_SetParameter(CUSTOMPROFILE_CHAR2, sizeof(uint8_t), &charValue3);
    CustomProfile_SetParameter(CUSTOMPROFILE_CHAR3, sizeof(uint8_t), &charValue2);
    CustomProfile_SetParameter(CUSTOMPROFILE_CHAR4, sizeof(uint8_t), &charValue1);
    
    //Motion
    XYZEuler_SetParameter(XYZ_EULER_CHAR1, sizeof(uint8_t), &motionInitRate);
    XYZEuler_SetParameter(XYZ_EULER_CHAR2, sizeof(uint8_t), &charValue3);
    XYZEuler_SetParameter(XYZ_EULER_CHAR3, sizeof(uint8_t), &charValue2);
    XYZEuler_SetParameter(XYZ_EULER_CHAR4, sizeof(uint8_t), &charValue1);
    
    changeOutputValueSort(OUTPUT_EULER);
  }

  // Register callback with SimpleGATTprofile
  // <-- 这里Service通道要都写上，注意
  SimpleProfile_RegisterAppCBs(&SimpleBLEPeripheral_simpleProfileCBs);
  CustomProfile_RegisterAppCBs(&SimpleBLEPeripheral_customProfileCBs);
  XYZEuler_RegisterAppCBs(&SimpleBLEPeripheral_xyzEulerCBs);
#endif //!FEATURE_OAD_ONCHIP

  // Start the Device
  VOID GAPRole_StartDevice(&SimpleBLEPeripheral_gapRoleCBs);

  // Start Bond Manager
  VOID GAPBondMgr_Register(&simpleBLEPeripheral_BondMgrCBs);

  // Register with GAP for HCI/Host messages
  GAP_RegisterForMsgs(selfEntity);

  // Register for GATT local events and ATT Responses pending for transmission
  GATT_RegisterForMsgs(selfEntity);

  //This should be included only if 4.2 length extension feature is enable....
  //HCI_LE_ReadMaxDataLenCmd();

#if !defined (USE_LL_CONN_PARAM_UPDATE)
  // Get the currently set local supported LE features
  // The HCI will generate an HCI event that will get received in the main
  // loop
  HCI_LE_ReadLocalSupportedFeaturesCmd();
#endif // !defined (USE_LL_CONN_PARAM_UPDATE)
  
  //非蓝牙测试模式下，要事先调节好蓝牙发射功率和接收增益
  if(BLE_VARI_MODE == 0){
    changeTxLevel(HIGH_TX); // <-- 消去这一句就变成默认的0dBm发射强度
    changeRxLevel(HIGH_RX); // <-- 消去这一句就变成默认的普通Rx增益
  }
  
  //最后做好系统设定工作
  meDisableBLEConnect();
  terminateConnectionWithDisabled = 1;
}

//实时修改功率和增益
uint8_t rxLevel = 0;
extern void changeRxLevel(uint8_t neoRxLevel){
  rxLevel = neoRxLevel;
  if(rxLevel == LOW_RX){
    //设置为低增益接收
    HCI_EXT_SetRxGainCmd(HCI_EXT_RX_GAIN_STD);
  }else{
    //设置为高增益接收
    HCI_EXT_SetRxGainCmd(HCI_EXT_RX_GAIN_HIGH);
  }
}
uint8_t txLevel = 0;
extern void changeTxLevel(uint8_t neoTxLevel){
  txLevel = neoTxLevel;
  if(txLevel == LOW_TX){
    //设置为低功率发射
    HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_0_DBM);
  }else{
    //设置为高功率发射
    HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_5_DBM);
  }
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_taskFxn
 *
 * @brief   Application task entry point for the Simple BLE Peripheral.
 *
 * @param   a0, a1 - not used.
 *
 * @return  None.
 */
uint8_t mpuInitializeCount;
static void SimpleBLEPeripheral_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  SimpleBLEPeripheral_init();
  
  //预循环：这里做设备的初始化
  //这里是陀螺仪开机状态相关======================================================
  //==============================================================================
  mpuInitializeCount = (AXIS_9_FUSION_MODE ? 9 : 5);
  Util_startClock(&initializeClock);
  //==============================================================================
  //==============================================================================
  
  
  // Application main loop
  for (;;)
  {
    uint32_t events;

    // Waits for an event to be posted associated with the calling thread.
    // Note that an event associated with a thread is posted when a
    // message is queued to the message receive queue of the thread
    // Event_Pend: 阻断任务继续进行，直到有EVENT信号进来给打断
    events = Event_pend(syncEvent, Event_Id_NONE, SBP_ALL_EVENTS,
                        ICALL_TIMEOUT_FOREVER);

    if (events)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;

      if (ICall_fetchServiceMsg(&src, &dest,
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        uint8 safeToDealloc = TRUE;

        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          ICall_Stack_Event *pEvt = (ICall_Stack_Event *)pMsg;

          // Check for BLE stack events first
          if (pEvt->signature == 0xffff)
          {
            if (pEvt->event_flag & SBP_HCI_CONN_EVT_END_EVT)
            {
              // Try to retransmit pending ATT Response (if any)
              SimpleBLEPeripheral_sendAttRsp();
            }
          }
          else
          {
            // Process inter-task message
            safeToDealloc = SimpleBLEPeripheral_processStackMsg((ICall_Hdr *)pMsg);
          }
        }

        if (pMsg && safeToDealloc)
        {
          ICall_freeMsg(pMsg);
        }
      }
      
      //最优先的线程：MPU初始化
      if (events & SBP_INITIALIZE_EVT)
      {
        SimpleBLEPeripheral_performInitializeTask();
      }

      // If RTOS queue is not empty, process app message.
      if (events & SBP_QUEUE_EVT)
      {
        while (!Queue_empty(appMsgQueue))
        {
          sbpEvt_t *pMsg = (sbpEvt_t *)Util_dequeueMsg(appMsgQueue);
          if (pMsg)
          {
            // Process message.
            SimpleBLEPeripheral_processAppMsg(pMsg);

            // Free the space from the message.
            ICall_free(pMsg);
          }
        }
      }

      if (events & SBP_PERIODIC_EVT)
      {
        //等待计时完成一次后再生成一个SBP_PERIODIC_EVT事件
        Util_startClock(&periodicClock);
        // Perform periodic application task
        SimpleBLEPeripheral_performPeriodicTask();
      }
      
#ifdef FEATURE_OAD
      if (events & SBP_QUEUE_PING_EVT)
      {
        while (!Queue_empty(hOadQ))
        {
          oadTargetWrite_t *oadWriteEvt = Queue_get(hOadQ);

          // Identify new image.
          if (oadWriteEvt->event == OAD_WRITE_IDENTIFY_REQ)
          {
            OAD_imgIdentifyWrite(oadWriteEvt->connHandle, oadWriteEvt->pData);
          }
          // Write a next block request.
          else if (oadWriteEvt->event == OAD_WRITE_BLOCK_REQ)
          {
            OAD_imgBlockWrite(oadWriteEvt->connHandle, oadWriteEvt->pData);
          }

          // Free buffer.
          ICall_free(oadWriteEvt);
        }
      }
#endif //FEATURE_OAD
    }
  }
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t SimpleBLEPeripheral_processStackMsg(ICall_Hdr *pMsg)
{
  uint8_t safeToDealloc = TRUE;

  switch (pMsg->event)
  {
    case GATT_MSG_EVENT:
      // Process GATT message
      safeToDealloc = SimpleBLEPeripheral_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;

    case HCI_GAP_EVENT_EVENT:
      {

        // Process HCI message
        switch(pMsg->status)
        {
          case HCI_COMMAND_COMPLETE_EVENT_CODE:
            // Process HCI Command Complete Event
            {

#if !defined (USE_LL_CONN_PARAM_UPDATE)
              // This code will disable the use of the LL_CONNECTION_PARAM_REQ
              // control procedure (for connection parameter updates, the 
              // L2CAP Connection Parameter Update procedure will be used 
              // instead). To re-enable the LL_CONNECTION_PARAM_REQ control
              // procedures, define the symbol USE_LL_CONN_PARAM_UPDATE                
              
              // Parse Command Complete Event for opcode and status
              hciEvt_CmdComplete_t* command_complete = (hciEvt_CmdComplete_t*) pMsg;
              uint8_t   pktStatus = command_complete->pReturnParam[0]; 
              
              //find which command this command complete is for
              switch (command_complete->cmdOpcode)
              {
                case HCI_LE_READ_LOCAL_SUPPORTED_FEATURES:
                  {
                    if (pktStatus == SUCCESS)
                    {
                      uint8_t featSet[8];
                      
                      // get current feature set from received event (bits 1-9 of
                      // the returned data
                      memcpy( featSet, &command_complete->pReturnParam[1], 8 );                          
                      
                      // Clear bit 1 of byte 0 of feature set to disable LL
                      // Connection Parameter Updates
                      CLR_FEATURE_FLAG( featSet[0], LL_FEATURE_CONN_PARAMS_REQ );
                        
                      // Update controller with modified features
                      HCI_EXT_SetLocalSupportedFeaturesCmd( featSet );
                    }
                  }
                  break;
                
                default:
                  //do nothing
                  break;
              }   
#endif // !defined (USE_LL_CONN_PARAM_UPDATE)
              
            }
            break;

          case HCI_BLE_HARDWARE_ERROR_EVENT_CODE:
            AssertHandler(HAL_ASSERT_CAUSE_HARDWARE_ERROR,0);
            break;

          default:
            break;
        }
      }
      break;

      default:
        // do nothing
        break;

    }
  
  return (safeToDealloc);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t SimpleBLEPeripheral_processGATTMsg(gattMsgEvent_t *pMsg)
{
  // See if GATT server was unable to transmit an ATT response
  if (pMsg->hdr.status == blePending)
  {
    // No HCI buffer was available. Let's try to retransmit the response
    // on the next connection event.
    if (HCI_EXT_ConnEventNoticeCmd(pMsg->connHandle, selfEntity,
                                   SBP_HCI_CONN_EVT_END_EVT) == SUCCESS)
    {
      // First free any pending response
      SimpleBLEPeripheral_freeAttRsp(FAILURE);

      // Hold on to the response message for retransmission
      pAttRsp = pMsg;

      // Don't free the response message yet
      return (FALSE);
    }
  }
  else if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
  {
    // ATT request-response or indication-confirmation flow control is
    // violated. All subsequent ATT requests or indications will be dropped.
    // The app is informed in case it wants to drop the connection.

    // Display the opcode of the message that caused the violation.
  }
  else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
  {
    // MTU size updated
  }

  // Free message payload. Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);

  // It's safe to free the incoming message
  return (TRUE);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_sendAttRsp
 *
 * @brief   Send a pending ATT response message.
 *
 * @param   none
 *
 * @return  none
 */
static void SimpleBLEPeripheral_sendAttRsp(void)
{
  // See if there's a pending ATT Response to be transmitted
  if (pAttRsp != NULL)
  {
    uint8_t status;

    // Increment retransmission count
    rspTxRetry++;

    // Try to retransmit ATT response till either we're successful or
    // the ATT Client times out (after 30s) and drops the connection.
    status = GATT_SendRsp(pAttRsp->connHandle, pAttRsp->method, &(pAttRsp->msg));
    if ((status != blePending) && (status != MSG_BUFFER_NOT_AVAIL))
    {
      // Disable connection event end notice
      HCI_EXT_ConnEventNoticeCmd(pAttRsp->connHandle, selfEntity, 0);

      // We're done with the response message
      SimpleBLEPeripheral_freeAttRsp(status);
    }
    else
    {
      // Continue retrying
    }
  }
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_freeAttRsp
 *
 * @brief   Free ATT response message.
 *
 * @param   status - response transmit status
 *
 * @return  none
 */
static void SimpleBLEPeripheral_freeAttRsp(uint8_t status)
{
  // See if there's a pending ATT response message
  if (pAttRsp != NULL)
  {
    // See if the response was sent out successfully
    if (status == SUCCESS)
    {
    }
    else
    {
      // Free response payload
      GATT_bm_free(&pAttRsp->msg, pAttRsp->method);

    }

    // Free response message
    ICall_freeMsg(pAttRsp);

    // Reset our globals
    pAttRsp = NULL;
    rspTxRetry = 0;
  }
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_processAppMsg(sbpEvt_t *pMsg)
{
  switch (pMsg->hdr.event)
  {
    case SBP_STATE_CHANGE_EVT:
      SimpleBLEPeripheral_processStateChangeEvt((gaprole_States_t)pMsg->
                                                hdr.state);
      break;

    case SBP_CHAR_CHANGE_EVT:
      SimpleBLEPeripheral_processCharValueChangeEvt(pMsg->hdr.state);
      break;
    
    case SBP_CUSTOM_CHAR_CHANGE_EVT:
      CustomPeripheral_processCharValueChangeEvt(pMsg->hdr.state);
      break;
    
    case SBP_EULER_CHAR_CHANGE_EVT:
      EulerPeripheral_processCharValueChangeEvt(pMsg->hdr.state);
      break;
    
    default:
      // Do nothing.
      break;
  }
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_stateChangeCB
 *
 * @brief   Callback from GAP Role indicating a role state change.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_stateChangeCB(gaprole_States_t newState)
{
  SimpleBLEPeripheral_enqueueMsg(SBP_STATE_CHANGE_EVT, newState);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_processStateChangeEvt
 *
 * @brief   Process a pending GAP Role state change event.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
//禁止连接时的操作
void meDisableBLEConnect(void){
  if(terminateConnectionWithDisabled > 0){
    //挂断后，如果是内部原因挂断，则不允许自动连接
    if(disconnectByInterrupt > 0){
      //是外因挂断的（此时检查是否有蓝牙关闭预告）
      if(bleIsToClose <= 0){
        //没有预告，即不明原因的断开，并且工作状态已达到正2级，则尝试重连
        if(getSYSTEMWorkLevel() < SYSTEM_ENERGY_LEVEL2){
          enableAutoConnect = 0;
        }else{
          enableAutoConnect = CONNECT_WITHOUT_MPU + 1;
        }
      }else{
        //有预告，即明确规定被软件人为断开了，则不尝试重连
        enableAutoConnect = 0;
      }
      bleIsToClose = 0;
    }else{
      //内因挂断，不尝试重连
      enableAutoConnect = 0;
    }
    setSYSTEMWorkLevel(SYSTEM_ENERGY_LEVEL1);
    enableConnectBLE = 0;
    if(isCharging(0) <= 0){
      //如果不在充电，则关闭灯光
      pinDark(1);
    }
  }else{
    //挂断后，仍然可以自动连接
    setSYSTEMWorkLevel(SYSTEM_ENERGY_LEVEL1_LPA);
  }
  disconnectByInterrupt = 0;
}
//返回：还剩多少个LPA周期
uint8_t getLPALampLoop(void){
  return lpaLampLoop;
}
static void SimpleBLEPeripheral_processStateChangeEvt(gaprole_States_t newState)
{
#ifdef PLUS_BROADCASTER
  static bool firstConnFlag = false;
#endif // PLUS_BROADCASTER

  switch ( newState )
  {
    case GAPROLE_STARTED:
      {
        uint8_t ownAddress[B_ADDR_LEN];
        uint8_t systemId[DEVINFO_SYSTEM_ID_LEN];

        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = ownAddress[0];
        systemId[1] = ownAddress[1];
        systemId[2] = ownAddress[2];

        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = ownAddress[5];
        systemId[6] = ownAddress[4];
        systemId[5] = ownAddress[3];

        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);

        // Display device address
      }
      break;

    case GAPROLE_ADVERTISING:
      //meDisableBLEConnect();
      break;

#ifdef PLUS_BROADCASTER
    /* After a connection is dropped a device in PLUS_BROADCASTER will continue
     * sending non-connectable advertisements and shall sending this change of
     * state to the application.  These are then disabled here so that sending
     * connectable advertisements can resume.
     */
    case GAPROLE_ADVERTISING_NONCONN:
      {
        uint8_t advertEnabled = FALSE;

        // Disable non-connectable advertising.
        GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t),
                           &advertEnabled);

        advertEnabled = TRUE;

        // Enabled connectable advertising.
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                             &advertEnabled);

        // Reset flag for next connection.
        firstConnFlag = false;

        SimpleBLEPeripheral_freeAttRsp(bleNotConnected);
      }
      break;
#endif //PLUS_BROADCASTER

    case GAPROLE_CONNECTED:
      {
        if(lpaLampLoop <= 0){
          lpaLampLoop = PRECONNECT_RETRY_MAX;
        }
        bleStopReason = 1;
        linkDBInfo_t linkInfo;
        uint8_t numActive = 0;
        
        //打开时钟，等待计时结束，事件生成后执行perodicTask
        //注：只有蓝牙已连接状况下才执行
        Util_startClock(&periodicClock);
        //接受蓝牙请求，蓝牙连接，进入2级工作状态
        //注意3级工作状态是无法从这里直接进入的，必须先执行EEE3的系统设定
        if(enableAutoConnect > 0 || WORKLEVEL2_WITHOUT_MPU > 0){
          //允许不经陀螺仪直接接通：直接进入2状态
          enableAutoConnect = 0;
          setSYSTEMWorkLevel(SYSTEM_ENERGY_LEVEL2);
        }else{
          //否则进入准2状态
          setSYSTEMWorkLevel(SYSTEM_ENERGY_LEVEL2_LPA);
        }
        //一旦接通，就重置所有1级工况以下使用的变量
        lpaLampLoop = 0;
        terminateConnectionWithDisabled = 1;
        enableConnectBLE = 0;
        numActive = linkDB_NumActive();
        //重置灯光状况
        resetLampCount();
        //在2级以上工况被断开，则一定是外部因素挂断(没电或充电情况除外)
        disconnectByInterrupt = 1;
        
        // Use numActive to determine the connection handle of the last
        // connection
        if ( linkDB_GetInfo( numActive - 1, &linkInfo ) == SUCCESS )
        {
        }
        else
        {
          uint8_t peerAddress[B_ADDR_LEN];

          GAPRole_GetParameter(GAPROLE_CONN_BD_ADDR, peerAddress);

        }

        #ifdef PLUS_BROADCASTER
          // Only turn advertising on for this state when we first connect
          // otherwise, when we go from connected_advertising back to this state
          // we will be turning advertising back on.
          if (firstConnFlag == false)
          {
            uint8_t advertEnabled = FALSE; // Turn on Advertising

            // Disable connectable advertising.
            GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                                 &advertEnabled);

            // Set to true for non-connectabel advertising.
            advertEnabled = TRUE;

            // Enable non-connectable advertising.
            GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t),
                                 &advertEnabled);
            firstConnFlag = true;
          }
        #endif // PLUS_BROADCASTER
      }
      break;

    case GAPROLE_CONNECTED_ADV:
      break;

    case GAPROLE_WAITING:
    case GAPROLE_WAITING_AFTER_TIMEOUT:
      //这里是蓝牙断开时最先执行的部分
      Util_stopClock(&periodicClock);
      //这里，只要一停掉使用蓝牙的程序，或者人工挂断连接，蓝牙就断开
      SimpleBLEPeripheral_freeAttRsp(bleNotConnected);
      
      //系统判断：是连接被人工挂断，还是Advertise到时间了？
      if(bleStopReason > 0){
        //人为原因挂断，或者蓝牙信号不佳而断连：
        //令MPU回到1级工作状态，并打开状态归零倒计时
        meDisableBLEConnect();
      }
      break;
    case GAPROLE_ERROR:
      break;

    default:
      break;
  }

  // Update the state
  //gapProfileState = newState;
}

//最开始simpleProfile蓝牙通道的事件处理机制=====================================
//==============================================================================
#ifndef FEATURE_OAD_ONCHIP
/*********************************************************************
 * @fn      SimpleBLEPeripheral_charValueChangeCB
 *
 * @brief   Callback from Simple Profile indicating a characteristic
 *          value change.
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_charValueChangeCB(uint8_t paramID)
{
  SimpleBLEPeripheral_enqueueMsg(SBP_CHAR_CHANGE_EVT, paramID);
}
#endif //!FEATURE_OAD_ONCHIP

/*********************************************************************
 * @fn      SimpleBLEPeripheral_processCharValueChangeEvt
 *
 * @brief   Process a pending Simple Profile characteristic value change
 *          event.
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_processCharValueChangeEvt(uint8_t paramID)
{
#ifndef FEATURE_OAD_ONCHIP
  uint8_t newValue;

  switch(paramID)
  {
    case SIMPLEPROFILE_CHAR1:
      SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR1, &newValue);
      break;

    case SIMPLEPROFILE_CHAR3:
      SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR3, &newValue);
      break;

    default:
      // should not reach here!
      break;
  }
#endif //!FEATURE_OAD_ONCHIP
}
//==============================================================================
//==============================================================================

//customProfile蓝牙通道的事件处理机制===========================================
//==============================================================================
#ifndef FEATURE_OAD_ONCHIP
/*********************************************************************
 * @fn      SimpleBLEPeripheral_charValueChangeCB
 *
 * @brief   Callback from Simple Profile indicating a characteristic
 *          value change.
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  None.
 */
static void CustomPeripheral_charValueChangeCB(uint8_t paramID)
{
  SimpleBLEPeripheral_enqueueMsg(SBP_CUSTOM_CHAR_CHANGE_EVT, paramID);
}
#endif //!FEATURE_OAD_ONCHIP

/*********************************************************************
 * @fn      SimpleBLEPeripheral_processCharValueChangeEvt
 *
 * @brief   Process a pending Simple Profile characteristic value change
 *          event.
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  None.
 */
static void CustomPeripheral_processCharValueChangeEvt(uint8_t paramID)
{
#ifndef FEATURE_OAD_ONCHIP
  //
#endif //!FEATURE_OAD_ONCHIP
}

//欧拉角蓝牙通道的事件处理机制==================================================
//==============================================================================
#ifndef FEATURE_OAD_ONCHIP
/*********************************************************************
 * @fn      SimpleBLEPeripheral_charValueChangeCB
 *
 * @brief   Callback from Simple Profile indicating a characteristic
 *          value change.
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  None.
 */
static void EulerPeripheral_charValueChangeCB(uint8_t paramID)
{
  SimpleBLEPeripheral_enqueueMsg(SBP_EULER_CHAR_CHANGE_EVT, paramID);
}
#endif //!FEATURE_OAD_ONCHIP

/*********************************************************************
 * @fn      SimpleBLEPeripheral_processCharValueChangeEvt
 *
 * @brief   Process a pending Simple Profile characteristic value change
 *          event.
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  None.
 */
static void EulerPeripheral_processCharValueChangeEvt(uint8_t paramID)
{
#ifndef FEATURE_OAD_ONCHIP
  //
#endif //!FEATURE_OAD_ONCHIP
}
//==============================================================================
//==============================================================================

/*********************************************************************
 * @fn      SimpleBLEPeripheral_performPeriodicTask
 *
 * @brief   Perform a periodic application task. This function gets called
 *          every five seconds (SBP_PERIODIC_EVT_PERIOD). In this example,
 *          the value of the third characteristic in the SimpleGATTProfile
 *          service is retrieved from the profile, and then copied into the
 *          value of the the fourth characteristic.
 *
 * @param   None.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_performInitializeTask(void)
{
  uint8_t i;
  PIN_GLight();
  //这里处理MPU初始化线程
  if(AXIS_9_FUSION_MODE > 0){
    //9轴模式运行
      mpuInitializeCount -= 1;
      switch(mpuInitializeCount){
      case 8:
        initMPLService();
        //初始化EMG数组栈
        for(i = 0; i < ADC_SMOOTH_FORCE_MAX; i++){
          adcSamples[i] = 0;
        }
        adcStackPointer = 0;
        break;
      case 7:
        initMPLService_STEP1();
        break;
      case 6:
        initMPLService_STEP1B();
        break;
      case 5:
        initMPLService_STEP2();
        break;
      case 4:
        initMPLService_STEP3();
        break;
      case 3:
        initMPLService_STEP3B();
        break;
      case 2:
        initMPLService_STEP4();
        break;
      case 1:
        initMPLService_STEP5();
        break;
      default:
        break;
      }
  }else{
    //6轴模式运行
      mpuInitializeCount -= 1;
      switch(mpuInitializeCount){
      case 4:
        initMPLService();
        //初始化EMG数组栈
        for(i = 0; i < ADC_SMOOTH_FORCE_MAX; i++){
          adcSamples[i] = 0;
        }
        adcStackPointer = 0;
        break;
      case 3:
        initMPLService_STEP1_TEST();
        break;
      case 2:
        initMPLService_STEP4();
        break;
      case 1:
        initMPLService_STEP5();
        break;
      default:
        break;
      }
  }
  //需判别初始化阶段，若MPU初始化完毕，则不再生成initializeClock事件
  if(mpuInitializeCount > 0){
    Util_startClock(&initializeClock);
  }else{
    if(isCharging(1) > 0){
      //充电中
      if(isChargeCompleted() > 0){
        pinGLightConst();
      }else{
        pinRLightConst();
      }
    }else{
      pinDark(1);
    }
  }
}

extern void mplTaskAtMainLoop(void);
uint8_t mplIsInitialized = 0;
uint8_t adcServiceGo = 0;
uint8_t batterySampleCount = 0;
uint16_t adcTempResult = 0;
uint8_t bleCycleCount = 127;

extern uint8_t workStatus = 0; // <-- 自身工作状态

//最关键的部分：蓝牙在连通时循环执行的部分
static void SimpleBLEPeripheral_performPeriodicTask(void)
{
#ifndef FEATURE_OAD_ONCHIP
  uint8_t workLevel;
  uint8_t i;
  uint8_t adcTotalNum;
  uint8_t batteryIsLow;
  uint16_t workResult = 0;
  
  long adcSigma;
  workStatus = WORKING_STATUS_OK; // <-- 工作状态：0为正常
  
  //是否为低电压请求挂断模式
  if(REJECT_CONNECT_ON_LOWBATT > 0){
    //检查电源：如果电源电压降至关机电压以下，就请求App端断开连接
    if(getBatteryData() <= SHUTDOWN_LOW_VOLTAGE){
      workStatus = WORKING_STATUS_REQUEST_DISCONNECT;
    }
    //检查充电：如果放在充电器上，同样会请求App端断开连接
    if(isCharging(0) > 0){
      workStatus = WORKING_STATUS_REQUEST_DISCONNECT;
    }
  }
  
  //蓝牙循环计数，直到计数器满了才执行下一个计算
  if(DISABLE_MPL > 0){
    //屏蔽Motion的运算环节？
    adcServiceGo = 1;
  }
  
  //判定系统工况
  workLevel = getSYSTEMWorkLevel();
  if(workLevel < SYSTEM_ENERGY_LEVEL2){
    //系统工况不足2级，就只空发数据而不进行运算
    if(workStatus == WORKING_STATUS_OK){
      workStatus = WORKING_STATUS_WAIT_FOR_TAP;
    }else if(workStatus == WORKING_STATUS_REQUEST_DISCONNECT){
      workStatus = WORKING_STATUS_DECLINE_CONNECTION;
    }
    if((adcServiceGo % 2) != 0){
      //肌电空发数据
      if(isCharging(0) > 0){
        //充电中：持续使用充电灯光
        if(isChargeCompleted() > 0){
          pinGLightConst();
        }else{
          pinRLightConst();
        }
      }else{
        //不在充电中：电池足够就亮灯，否则不亮
        if(workStatus == WORKING_STATUS_WAIT_FOR_TAP){
          setLEDWorkBlink(workLevel);
        }else{
          pinDark(1);
        }
      }
      adcTempResult = 0;
      //将工作状态合并到adcTempResult的最高4位
      workResult = workStatus;
      adcTempResult += (workResult << 12);
      SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, sizeof(uint16_t), &adcTempResult);
      adcServiceGo += 1;
      if(adcServiceGo >= 4){
        adcServiceGo = 0;
      }
    }else{
      //陀螺仪空发数据
      XYZEuler_SetParameter(XYZ_EULER_CHAR4, sizeof(uint8_t), &adcServiceGo);
      adcServiceGo += 1;
    }
    //如果发现了连接信号，则在下回进入该方法时，切换到2级状态
    if(enableConnectBLE > 0){
      enableConnectBLE = 0;
      setSYSTEMWorkLevel(SYSTEM_ENERGY_LEVEL2);
    }
    return;
  }
  
  //肌电和陀螺仪交替处理(只有工作状态至少正2级以上才会进入这个方法)
  if((adcServiceGo % 2) != 0){
    //轮到肌电处理
    
    //判定灯光和电压状态
    batteryIsLow = setLEDWorkBlink(workLevel);
    if(workStatus == WORKING_STATUS_OK){
      workStatus = batteryIsLow;
    }
    switch(workLevel){
    case SYSTEM_ENERGY_LEVEL2:
      //不在3级水平的工况：肌电数值恒定为0，并且不执行EMG采集工作
      adcTempResult = 0;
      if(getEMGOpen() == 1){
        //系统设定被启动了
        setSYSTEMWorkLevel(SYSTEM_ENERGY_LEVEL3);
      }
      break;
    case SYSTEM_ENERGY_LEVEL3:
      //蓝牙持续循环的同时，执行EMG的读取分析工作
      adcValue0Result = getADCData();
      //ADC打开成功且转换成功：这里计算ADC平滑后的数值
      if(getEMGSmooth() > 0){
        adcTotalNum = getEmgSmoPowByCalc();
        adcSamples[adcStackPointer] = adcValue0Result;
        //数据是否在平滑栈中堆满？满了就返回头一个
        adcStackPointer += 1;
        if(adcStackPointer >= adcTotalNum){
          adcStackPointer = 0;
        }
        //取头几个平滑？
        adcSigma = 0;
        for(i = 0; i < adcTotalNum; i++){
          adcSigma += adcSamples[i];
        }
        adcTempResult = adcSigma / adcTotalNum;
      }else{
        //不启用平滑：直接赋值
        adcTempResult = adcValue0Result;
      }
      if(getEMGOpen() == 0){
        //系统设定被关闭了
        setSYSTEMWorkLevel(SYSTEM_ENERGY_LEVEL2);
      }
      break;
    default:
      //不正确的工况：肌电数值恒定为0，并且不执行EMG采集工作
      adcTempResult = 0;
      break;
    }
    //顺序轮换
    adcServiceGo += 1;
    if(adcServiceGo >= 4){
      adcServiceGo = 0;
    }
    //将工作状态合并到adcTempResult的最高4位
    workResult = workStatus;
    adcTempResult += (workResult << 12);
    //数据传输
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, sizeof(uint16_t), &adcTempResult);
  }else{
    //轮到陀螺仪处理
    if(getMotionOpen() == 1){
      //蓝牙持续循环的同时，执行dmp的读取分析工作
      mplTaskAtMainLoop();
    }
    //顺序轮换
    adcServiceGo += 1;
    //陀螺仪信号的处理（伪处理，必须是某个值变化了才通知用户）
    XYZEuler_SetParameter(XYZ_EULER_CHAR4, sizeof(uint8_t), &adcServiceGo);
  }
  
#endif //!FEATURE_OAD_ONCHIP
}

#ifdef FEATURE_OAD
/*********************************************************************
 * @fn      SimpleBLEPeripheral_processOadWriteCB
 *
 * @brief   Process a write request to the OAD profile.
 *
 * @param   event      - event type:
 *                       OAD_WRITE_IDENTIFY_REQ
 *                       OAD_WRITE_BLOCK_REQ
 * @param   connHandle - the connection Handle this request is from.
 * @param   pData      - pointer to data for processing and/or storing.
 *
 * @return  None.
 */
void SimpleBLEPeripheral_processOadWriteCB(uint8_t event, uint16_t connHandle,
                                           uint8_t *pData)
{
  oadTargetWrite_t *oadWriteEvt = ICall_malloc( sizeof(oadTargetWrite_t) + \
                                             sizeof(uint8_t) * OAD_PACKET_SIZE);

  if ( oadWriteEvt != NULL )
  {
    oadWriteEvt->event = event;
    oadWriteEvt->connHandle = connHandle;

    oadWriteEvt->pData = (uint8_t *)(&oadWriteEvt->pData + 1);
    memcpy(oadWriteEvt->pData, pData, OAD_PACKET_SIZE);

    Queue_put(hOadQ, (Queue_Elem *)oadWriteEvt);

    // Post the application's event.  For OAD, no event flag is used.
    Event_post(syncEvent, SBP_QUEUE_PING_EVT);
  }
  else
  {
    // Fail silently.
  }
}
#endif //FEATURE_OAD

/*********************************************************************
 * @fn      SimpleBLEPeripheral_clockHandler
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   arg - event type
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_clockHandler(UArg arg)
{
  // Wake up the application.
  // 这里是处理程序循环运行的关键，解锁Event_Pend
  Event_post(syncEvent, arg);
}
static void SimpleBLEPeripheral_initializeHandler(UArg arg)
{
  // 唤醒APP并做优先级最高的处理：MPU初始化
  Event_post(syncEvent, arg);
}
static void SimpleBLEPeripheral_advertiseHandler(UArg arg)
{
  //进入1级工况，继续下一轮的Advertising
  setSYSTEMWorkLevel(SYSTEM_ENERGY_LEVEL1);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_enqueueMsg(uint8_t event, uint8_t state)
{
  sbpEvt_t *pMsg;

  // Create dynamic pointer to message.
  if ((pMsg = ICall_malloc(sizeof(sbpEvt_t))))
  {
    pMsg->hdr.event = event;
    pMsg->hdr.state = state;

    // Enqueue the message.
    Util_enqueueMsg(appMsgQueue, syncEvent, (uint8*)pMsg);
  }
}

//0级工况下Advertising的启停
int dbmChangeFlag = 3;
int dbmHigh = 0;
int dbmChangeFlagMax = 3;
uint8_t advertEnabled = 0;

void stopAdvertising(void){
  if(CHARGE_LAMP > 0){
    //灯光响应充电模式
    if(CHARGE_BLINK_AT_CHARGE > 0){
      pinDark(1);
    }else{
      if(isCharging(1) > 0){
        if(isChargeCompleted() > 0){
          PIN_GLight();
        }else{
          PIN_RLight();
        }
      }else{
        pinDark(1);
      }
    }
  }else{
    //灯光不响应充电模式
    pinDark(1);
  }
  //摇动自连接重试次数减少1次
  if(enableConnectBLE > 0){
    lpaLampLoop = 0;
    enableConnectBLE -= 1;
  }
  //还剩多少个白灯周期
  if(lpaLampLoop > 0){
    lpaLampLoop -= 1;
  }
  
  //测试：修改蓝牙功率
  if(BLE_VARI_MODE > 0){
    if(dbmChangeFlag > 0){
      dbmChangeFlag -= 1;
      if(dbmChangeFlag <= 0){
        dbmChangeFlag = dbmChangeFlagMax;
        if(dbmHigh == 0){
          dbmHigh = 1;
          // 3次广播后，变为高功率
          HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_5_DBM);
        }else{
          dbmHigh = 0;
          // 3次广播后，变为低功率
          HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_0_DBM);
        }
      }
    }
  }
  
  //启动广播重启定时器（并判定重启哪种定时器，长定时还是短定时）
  if(enableAutoConnect > 0){
    enableAutoConnect -= 1;
    Util_startClock(&advertiseShortClock);
  }else{
    Util_startClock(&advertiseClock);
  }
  
  //进入0级状态
  setSYSTEMWorkLevel(SYSTEM_ENERGY_LEVEL0);
}

//开始广播：这个方法仅有setSYSTEMWorkLevel(SYSTEM_ENERGY_LEVEL1)时才调用
uint8_t hasSetDuration = 0;
void startAdvertising(void){
  bleCycleCount = 127;
  //测试性亮灯（白灯亮为低功率发射模式，红灯亮为高功率发射模式）
  if(LAMP_TEST_MODE > 0){
    pinDark(1);
    if(enableAutoConnect <= 0){
      PIN_GLight(); // <-- 白灯亮：表示本次连接必须靠陀螺仪唤醒
    }else{
      PIN_RLight(); // <-- 红灯亮：表示本次连接可以不经陀螺仪同意直接唤醒
    }
  }else{
    //充电时周期到达，亮灯
    if(isCharging(1) > 0){
      if(isChargeCompleted() > 0){
        PIN_GLight();
      }else{
        PIN_RLight();
      }
    }else{
      pinDark(1);
    }
  }
  
  //先进入1级状态再开始广播（使能）
  advertEnabled = TRUE;
  //限制广播时间
  uint16_t advertDuration = ADVERT_DURATION;
  if(hasSetDuration == 0){
    //1.50版的Stack，这个参数只能重复设置一次，否则会死机
    GAP_SetParamValue(TGAP_LIM_ADV_TIMEOUT, advertDuration);
    hasSetDuration = 1;
  }
  GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &advertEnabled);
}

/*********************************************************************
*********************************************************************/
