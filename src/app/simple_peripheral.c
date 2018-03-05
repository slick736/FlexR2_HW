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

//ϵͳ�趨
extern uint8_t enableReadMPL;
extern uint8_t enableReadADC;
uint8_t BLE_PAUSED;
void meDisableBLEConnect(void);
extern uint8_t lpaLampLoop = 0;
extern void changeRxLevel(uint8_t neoRxLevel);
extern void changeTxLevel(uint8_t neoTxLevel);

//�Ƿ���������������ͬ����Զ�����
extern uint8_t enableAutoConnect = 0;

//�Ƿ�������豸��������
uint8_t enableConnectBLE;
uint8_t terminateConnectionWithDisabled; // <-- �ս����ӵ�ʱ���Ƿ�enableConnectBLEҲ��0
//�������Ƿ�Ϊ�ⲿ���عҶ�
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
//#define DEFAULT_ADVERTISING_INTERVAL          4800 // <-- ԭ����100ms��һ��Advertise�����ڸ�Ϊ3sһ��

//����ʱ��ά��ʱ��
#define ADVERT_AT_POWERON                     4         // 4 seconds

//ÿ�ι㲥��ά��ʱ�䣨���˹㲥ά��ʱ�����Զ�����0״̬��
//#define ADVERT_DURATION                       10         // 10 seconds
#define ADVERT_DURATION                       1
#define ADVERT_LONG_DURATION                  10
//���ι㲥�м��ЪϢʱ�䣨����ЪϢʱ����Զ�����1״̬��
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
   //�������Ӻ�Ҫ��Ϣ6�룿
#define DEFAULT_CONN_PAUSE_PERIPHERAL         6

// How often to perform periodic event (in msec)
#define SBP_PERIODIC_EVT_PERIOD               25 // 25, <-- Original 5000
//#define SBP_PERIODIC_EVT_PERIOD               500 // <-- 1��1��ADC��MPU����ѭ��
#define SBP_INITIALIZE_EVT_PERIOD             200 // <-- ִ�г�ʼ��ʱ��ÿ������������200ms

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
//������ʼ����ʱ���¼��ᵽ��ǰ
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

//��־λ���Ƿ��ϵͳ��Ԥ�����������ر�
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
  // ��������������ʲôService��������
  
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

//���ĸ�UUID���棿
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

//Flag�����������Ͽ�ԭ������Ϊ����Advertiseʱ�䵽
//0-ʱ�䵽 1-��Ϊ
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

//ADC��ر���
//extern���壺��������Ǳ𴦶���ģ�����Ҫ�������
extern uint8_t adcInitResult;
extern uint8_t adcConvertResult;
//extern ADC_Handle   adc;
//extern ADC_Params   params;
extern int_fast16_t res;
/* ADC conversion result variables */
uint16_t adcValue0Result;

//ADC��������㷨
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
// <-- ע������Ҫ�������е�Serviceͨ��ȫ��д��
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

  //display���ܲ���Ҫ����������
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
    uint32_t passkey = 0; // passkey "000000" <-- ע������������õ����룬Ŀǰȡֵ000000
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
  //�ڴ˴����������Service
  //ע�����ṹ��Service�Ķ�Ӧ��ϵ��profile��һ��.c��.h�����ļ���Ӧһ��Service
  //Service�ڲ���CharacteristicӦ��ȫ����AttrTable��ע�������ʹ��
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
  // ������ʼ����ʱ��д��ʲôֵ��
  {
    uint8_t charValue1 = 5;
    uint8_t charValue2 = adcInitResult; // <-- ԭ6; // <-- Ҫ��ΪadcInitResult;
    uint8_t charValue3 = 7;
    uint8_t charValue4 = 8; //adcConvertResult; // <-- ԭ8; // <-- Ҫ��ΪadcConvertResult;
    uint8_t charValue5[SIMPLEPROFILE_CHAR5_LEN] = { 1, 2, 3, 4, 5, 6 };
    uint8_t charValue6 = 9;
    uint8_t emgInitRate = DEFAULT_EMG_FREQ;
    uint8_t motionInitRate = DEFAULT_MOTION_FREQ;
    //I2C��ʼ��
    initI2C();
    //MPL�������ʼ��
    initBleOutData();
    //GPIO��ʼ��
    PIN_Initialize();
    
    //EMG
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR1, sizeof(uint8_t), &emgInitRate);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR2, sizeof(uint8_t), &charValue2);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR3, sizeof(uint8_t), &charValue3);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, sizeof(uint8_t), &charValue4);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR5, SIMPLEPROFILE_CHAR5_LEN, charValue5);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR6, sizeof(uint16_t), &charValue6);
    
    //ϵͳ
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
  // <-- ����Serviceͨ��Ҫ��д�ϣ�ע��
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
  
  //����������ģʽ�£�Ҫ���ȵ��ں��������书�ʺͽ�������
  if(BLE_VARI_MODE == 0){
    changeTxLevel(HIGH_TX); // <-- ��ȥ��һ��ͱ��Ĭ�ϵ�0dBm����ǿ��
    changeRxLevel(HIGH_RX); // <-- ��ȥ��һ��ͱ��Ĭ�ϵ���ͨRx����
  }
  
  //�������ϵͳ�趨����
  meDisableBLEConnect();
  terminateConnectionWithDisabled = 1;
}

//ʵʱ�޸Ĺ��ʺ�����
uint8_t rxLevel = 0;
extern void changeRxLevel(uint8_t neoRxLevel){
  rxLevel = neoRxLevel;
  if(rxLevel == LOW_RX){
    //����Ϊ���������
    HCI_EXT_SetRxGainCmd(HCI_EXT_RX_GAIN_STD);
  }else{
    //����Ϊ���������
    HCI_EXT_SetRxGainCmd(HCI_EXT_RX_GAIN_HIGH);
  }
}
uint8_t txLevel = 0;
extern void changeTxLevel(uint8_t neoTxLevel){
  txLevel = neoTxLevel;
  if(txLevel == LOW_TX){
    //����Ϊ�͹��ʷ���
    HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_0_DBM);
  }else{
    //����Ϊ�߹��ʷ���
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
  
  //Ԥѭ�����������豸�ĳ�ʼ��
  //�����������ǿ���״̬���======================================================
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
    // Event_Pend: �������������У�ֱ����EVENT�źŽ��������
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
      
      //�����ȵ��̣߳�MPU��ʼ��
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
        //�ȴ���ʱ���һ�κ�������һ��SBP_PERIODIC_EVT�¼�
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
//��ֹ����ʱ�Ĳ���
void meDisableBLEConnect(void){
  if(terminateConnectionWithDisabled > 0){
    //�ҶϺ�������ڲ�ԭ��Ҷϣ��������Զ�����
    if(disconnectByInterrupt > 0){
      //������Ҷϵģ���ʱ����Ƿ��������ر�Ԥ�棩
      if(bleIsToClose <= 0){
        //û��Ԥ�棬������ԭ��ĶϿ������ҹ���״̬�Ѵﵽ��2������������
        if(getSYSTEMWorkLevel() < SYSTEM_ENERGY_LEVEL2){
          enableAutoConnect = 0;
        }else{
          enableAutoConnect = CONNECT_WITHOUT_MPU + 1;
        }
      }else{
        //��Ԥ�棬����ȷ�涨�������Ϊ�Ͽ��ˣ��򲻳�������
        enableAutoConnect = 0;
      }
      bleIsToClose = 0;
    }else{
      //����Ҷϣ�����������
      enableAutoConnect = 0;
    }
    setSYSTEMWorkLevel(SYSTEM_ENERGY_LEVEL1);
    enableConnectBLE = 0;
    if(isCharging(0) <= 0){
      //������ڳ�磬��رյƹ�
      pinDark(1);
    }
  }else{
    //�ҶϺ���Ȼ�����Զ�����
    setSYSTEMWorkLevel(SYSTEM_ENERGY_LEVEL1_LPA);
  }
  disconnectByInterrupt = 0;
}
//���أ���ʣ���ٸ�LPA����
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
        
        //��ʱ�ӣ��ȴ���ʱ�������¼����ɺ�ִ��perodicTask
        //ע��ֻ������������״���²�ִ��
        Util_startClock(&periodicClock);
        //�������������������ӣ�����2������״̬
        //ע��3������״̬���޷�������ֱ�ӽ���ģ�������ִ��EEE3��ϵͳ�趨
        if(enableAutoConnect > 0 || WORKLEVEL2_WITHOUT_MPU > 0){
          //������������ֱ�ӽ�ͨ��ֱ�ӽ���2״̬
          enableAutoConnect = 0;
          setSYSTEMWorkLevel(SYSTEM_ENERGY_LEVEL2);
        }else{
          //�������׼2״̬
          setSYSTEMWorkLevel(SYSTEM_ENERGY_LEVEL2_LPA);
        }
        //һ����ͨ������������1����������ʹ�õı���
        lpaLampLoop = 0;
        terminateConnectionWithDisabled = 1;
        enableConnectBLE = 0;
        numActive = linkDB_NumActive();
        //���õƹ�״��
        resetLampCount();
        //��2�����Ϲ������Ͽ�����һ�����ⲿ���عҶ�(û������������)
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
      //�����������Ͽ�ʱ����ִ�еĲ���
      Util_stopClock(&periodicClock);
      //���ֻҪһͣ��ʹ�������ĳ��򣬻����˹��Ҷ����ӣ������ͶϿ�
      SimpleBLEPeripheral_freeAttRsp(bleNotConnected);
      
      //ϵͳ�жϣ������ӱ��˹��Ҷϣ�����Advertise��ʱ���ˣ�
      if(bleStopReason > 0){
        //��Ϊԭ��Ҷϣ����������źŲ��Ѷ�������
        //��MPU�ص�1������״̬������״̬���㵹��ʱ
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

//�ʼsimpleProfile����ͨ�����¼��������=====================================
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

//customProfile����ͨ�����¼��������===========================================
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

//ŷ��������ͨ�����¼��������==================================================
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
  //���ﴦ��MPU��ʼ���߳�
  if(AXIS_9_FUSION_MODE > 0){
    //9��ģʽ����
      mpuInitializeCount -= 1;
      switch(mpuInitializeCount){
      case 8:
        initMPLService();
        //��ʼ��EMG����ջ
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
    //6��ģʽ����
      mpuInitializeCount -= 1;
      switch(mpuInitializeCount){
      case 4:
        initMPLService();
        //��ʼ��EMG����ջ
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
  //���б��ʼ���׶Σ���MPU��ʼ����ϣ���������initializeClock�¼�
  if(mpuInitializeCount > 0){
    Util_startClock(&initializeClock);
  }else{
    if(isCharging(1) > 0){
      //�����
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

extern uint8_t workStatus = 0; // <-- ������״̬

//��ؼ��Ĳ��֣���������ͨʱѭ��ִ�еĲ���
static void SimpleBLEPeripheral_performPeriodicTask(void)
{
#ifndef FEATURE_OAD_ONCHIP
  uint8_t workLevel;
  uint8_t i;
  uint8_t adcTotalNum;
  uint8_t batteryIsLow;
  uint16_t workResult = 0;
  
  long adcSigma;
  workStatus = WORKING_STATUS_OK; // <-- ����״̬��0Ϊ����
  
  //�Ƿ�Ϊ�͵�ѹ����Ҷ�ģʽ
  if(REJECT_CONNECT_ON_LOWBATT > 0){
    //����Դ�������Դ��ѹ�����ػ���ѹ���£�������App�˶Ͽ�����
    if(getBatteryData() <= SHUTDOWN_LOW_VOLTAGE){
      workStatus = WORKING_STATUS_REQUEST_DISCONNECT;
    }
    //����磺������ڳ�����ϣ�ͬ��������App�˶Ͽ�����
    if(isCharging(0) > 0){
      workStatus = WORKING_STATUS_REQUEST_DISCONNECT;
    }
  }
  
  //����ѭ��������ֱ�����������˲�ִ����һ������
  if(DISABLE_MPL > 0){
    //����Motion�����㻷�ڣ�
    adcServiceGo = 1;
  }
  
  //�ж�ϵͳ����
  workLevel = getSYSTEMWorkLevel();
  if(workLevel < SYSTEM_ENERGY_LEVEL2){
    //ϵͳ��������2������ֻ�շ����ݶ�����������
    if(workStatus == WORKING_STATUS_OK){
      workStatus = WORKING_STATUS_WAIT_FOR_TAP;
    }else if(workStatus == WORKING_STATUS_REQUEST_DISCONNECT){
      workStatus = WORKING_STATUS_DECLINE_CONNECTION;
    }
    if((adcServiceGo % 2) != 0){
      //����շ�����
      if(isCharging(0) > 0){
        //����У�����ʹ�ó��ƹ�
        if(isChargeCompleted() > 0){
          pinGLightConst();
        }else{
          pinRLightConst();
        }
      }else{
        //���ڳ���У�����㹻�����ƣ�������
        if(workStatus == WORKING_STATUS_WAIT_FOR_TAP){
          setLEDWorkBlink(workLevel);
        }else{
          pinDark(1);
        }
      }
      adcTempResult = 0;
      //������״̬�ϲ���adcTempResult�����4λ
      workResult = workStatus;
      adcTempResult += (workResult << 12);
      SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, sizeof(uint16_t), &adcTempResult);
      adcServiceGo += 1;
      if(adcServiceGo >= 4){
        adcServiceGo = 0;
      }
    }else{
      //�����ǿշ�����
      XYZEuler_SetParameter(XYZ_EULER_CHAR4, sizeof(uint8_t), &adcServiceGo);
      adcServiceGo += 1;
    }
    //��������������źţ������»ؽ���÷���ʱ���л���2��״̬
    if(enableConnectBLE > 0){
      enableConnectBLE = 0;
      setSYSTEMWorkLevel(SYSTEM_ENERGY_LEVEL2);
    }
    return;
  }
  
  //����������ǽ��洦��(ֻ�й���״̬������2�����ϲŻ�����������)
  if((adcServiceGo % 2) != 0){
    //�ֵ����紦��
    
    //�ж��ƹ�͵�ѹ״̬
    batteryIsLow = setLEDWorkBlink(workLevel);
    if(workStatus == WORKING_STATUS_OK){
      workStatus = batteryIsLow;
    }
    switch(workLevel){
    case SYSTEM_ENERGY_LEVEL2:
      //����3��ˮƽ�Ĺ�����������ֵ�㶨Ϊ0�����Ҳ�ִ��EMG�ɼ�����
      adcTempResult = 0;
      if(getEMGOpen() == 1){
        //ϵͳ�趨��������
        setSYSTEMWorkLevel(SYSTEM_ENERGY_LEVEL3);
      }
      break;
    case SYSTEM_ENERGY_LEVEL3:
      //��������ѭ����ͬʱ��ִ��EMG�Ķ�ȡ��������
      adcValue0Result = getADCData();
      //ADC�򿪳ɹ���ת���ɹ����������ADCƽ�������ֵ
      if(getEMGSmooth() > 0){
        adcTotalNum = getEmgSmoPowByCalc();
        adcSamples[adcStackPointer] = adcValue0Result;
        //�����Ƿ���ƽ��ջ�ж��������˾ͷ���ͷһ��
        adcStackPointer += 1;
        if(adcStackPointer >= adcTotalNum){
          adcStackPointer = 0;
        }
        //ȡͷ����ƽ����
        adcSigma = 0;
        for(i = 0; i < adcTotalNum; i++){
          adcSigma += adcSamples[i];
        }
        adcTempResult = adcSigma / adcTotalNum;
      }else{
        //������ƽ����ֱ�Ӹ�ֵ
        adcTempResult = adcValue0Result;
      }
      if(getEMGOpen() == 0){
        //ϵͳ�趨���ر���
        setSYSTEMWorkLevel(SYSTEM_ENERGY_LEVEL2);
      }
      break;
    default:
      //����ȷ�Ĺ�����������ֵ�㶨Ϊ0�����Ҳ�ִ��EMG�ɼ�����
      adcTempResult = 0;
      break;
    }
    //˳���ֻ�
    adcServiceGo += 1;
    if(adcServiceGo >= 4){
      adcServiceGo = 0;
    }
    //������״̬�ϲ���adcTempResult�����4λ
    workResult = workStatus;
    adcTempResult += (workResult << 12);
    //���ݴ���
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, sizeof(uint16_t), &adcTempResult);
  }else{
    //�ֵ������Ǵ���
    if(getMotionOpen() == 1){
      //��������ѭ����ͬʱ��ִ��dmp�Ķ�ȡ��������
      mplTaskAtMainLoop();
    }
    //˳���ֻ�
    adcServiceGo += 1;
    //�������źŵĴ���α����������ĳ��ֵ�仯�˲�֪ͨ�û���
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
  // �����Ǵ������ѭ�����еĹؼ�������Event_Pend
  Event_post(syncEvent, arg);
}
static void SimpleBLEPeripheral_initializeHandler(UArg arg)
{
  // ����APP�������ȼ���ߵĴ���MPU��ʼ��
  Event_post(syncEvent, arg);
}
static void SimpleBLEPeripheral_advertiseHandler(UArg arg)
{
  //����1��������������һ�ֵ�Advertising
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

//0��������Advertising����ͣ
int dbmChangeFlag = 3;
int dbmHigh = 0;
int dbmChangeFlagMax = 3;
uint8_t advertEnabled = 0;

void stopAdvertising(void){
  if(CHARGE_LAMP > 0){
    //�ƹ���Ӧ���ģʽ
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
    //�ƹⲻ��Ӧ���ģʽ
    pinDark(1);
  }
  //ҡ�����������Դ�������1��
  if(enableConnectBLE > 0){
    lpaLampLoop = 0;
    enableConnectBLE -= 1;
  }
  //��ʣ���ٸ��׵�����
  if(lpaLampLoop > 0){
    lpaLampLoop -= 1;
  }
  
  //���ԣ��޸���������
  if(BLE_VARI_MODE > 0){
    if(dbmChangeFlag > 0){
      dbmChangeFlag -= 1;
      if(dbmChangeFlag <= 0){
        dbmChangeFlag = dbmChangeFlagMax;
        if(dbmHigh == 0){
          dbmHigh = 1;
          // 3�ι㲥�󣬱�Ϊ�߹���
          HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_5_DBM);
        }else{
          dbmHigh = 0;
          // 3�ι㲥�󣬱�Ϊ�͹���
          HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_0_DBM);
        }
      }
    }
  }
  
  //�����㲥������ʱ�������ж��������ֶ�ʱ��������ʱ���Ƕ̶�ʱ��
  if(enableAutoConnect > 0){
    enableAutoConnect -= 1;
    Util_startClock(&advertiseShortClock);
  }else{
    Util_startClock(&advertiseClock);
  }
  
  //����0��״̬
  setSYSTEMWorkLevel(SYSTEM_ENERGY_LEVEL0);
}

//��ʼ�㲥�������������setSYSTEMWorkLevel(SYSTEM_ENERGY_LEVEL1)ʱ�ŵ���
uint8_t hasSetDuration = 0;
void startAdvertising(void){
  bleCycleCount = 127;
  //���������ƣ��׵���Ϊ�͹��ʷ���ģʽ�������Ϊ�߹��ʷ���ģʽ��
  if(LAMP_TEST_MODE > 0){
    pinDark(1);
    if(enableAutoConnect <= 0){
      PIN_GLight(); // <-- �׵�������ʾ�������ӱ��뿿�����ǻ���
    }else{
      PIN_RLight(); // <-- ���������ʾ�������ӿ��Բ���������ͬ��ֱ�ӻ���
    }
  }else{
    //���ʱ���ڵ������
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
  
  //�Ƚ���1��״̬�ٿ�ʼ�㲥��ʹ�ܣ�
  advertEnabled = TRUE;
  //���ƹ㲥ʱ��
  uint16_t advertDuration = ADVERT_DURATION;
  if(hasSetDuration == 0){
    //1.50���Stack���������ֻ���ظ�����һ�Σ����������
    GAP_SetParamValue(TGAP_LIM_ADV_TIMEOUT, advertDuration);
    hasSetDuration = 1;
  }
  GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &advertEnabled);
}

/*********************************************************************
*********************************************************************/
