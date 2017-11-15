/******************************************************************************

 @file  main.c

 @brief main entry of the BLE stack sample application.

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

/*******************************************************************************
 * INCLUDES
 */

#include <xdc/runtime/Error.h>

#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>

#include "icall.h"
#include "hal_assert.h"
#include "bcomdef.h"
#include "peripheral.h"
#include "simple_peripheral.h"
#include "main.h"
#include "util.h"
#include "arrayUtil.h"
#include "i2cRW.h"
#include "gpioCustom.h"
#include "xyz_euler_profile.h"
#include <driverlib/ioc.h>

/* Header files required to enable instruction fetch cache */
#include <inc/hw_memmap.h>
#include <driverlib/vims.h>

#ifndef USE_DEFAULT_USER_CFG

#include "ble_user_config.h"

// BLE user defined configuration
#ifdef ICALL_JT
icall_userCfg_t user0Cfg = BLE_USER_CFG;
#else  /* ! ICALL_JT */
bleUserCfg_t user0Cfg = BLE_USER_CFG;
#endif /* ICALL_JT */

#endif // USE_DEFAULT_USER_CFG

#ifdef USE_CORE_SDK
  #include <ti/display/Display.h>
#else // !USE_CORE_SDK
  #include <ti/mw/display/Display.h>
#endif // USE_CORE_SDK

#ifdef USE_FPGA
#include <inc/hw_prcm.h>
#endif // USE_FPGA

//这里尝试加入除蓝牙之外的其他功能，如I2C、ADC等
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
//#include <unistd.h> // <-- 这个库有问题（路径？）
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
//#include <ti/drivers/ADC.h>
#if defined(CC2650DK_7ID) || defined(CC1310DK_7XD)
#include <ti/drivers/PIN.h>
#endif

//global 公用变量
//系统设定值：是否打开MPL的dmpRead功能
uint8_t enableReadMPL = 0;
uint8_t enableReadADC = 0;
void mpuHibernate(void);
void adcOff(void);
void mpuWakeUp(void);
void adcOn(void);
void mpuIntoLPA(void);
void mpuIntoLPMIM(void);
void mpuOutOfLPMIM(void);

//预定义的总体设置接口
void setEMGFreq(uint8_t neoFreq);
uint8_t getEMGFreq(void);
//修改EMG频率后，需要一段调整时间
static Clock_Struct emgPauseClock;
void emgPauseToSetting(void){
  //EMG暂停，打开时钟（注：该情况只可能发生在2级工况以上，即蓝牙已接通）
  adcOff();
  Util_startClock(&emgPauseClock);
}
static void emgPause_clockHandler(UArg arg)
{
  // EMG暂停时间结束，重开EMG
  adcOn();
}

void setMPLFreq(uint8_t neoFreq);
uint8_t getMPLFreq(void);

void setSyetemOption(uint16_t sysOption);
uint16_t getSystemOption(void);

//预定义的个别设置接口
void setEMGOpen(uint8_t neoOpen);
uint8_t getEMGOpen(void);

void setEMGSmooth(uint8_t neoSmo);
uint8_t getEMGSmooth(void);

void setEmgSmoPow(uint8_t neoSPo);
uint8_t getEmgSmoPow(void);

uint8_t motionIsOpen;
void setMotionOpen(uint8_t neoMotion);
uint8_t getMotionOpen(void);

void setMPLDFormat(uint8_t neoFmt);
uint8_t getMPLDFormat(void);

void setMPUWakePower(uint8_t neoPow);
uint8_t getMPUWakePower(void);
uint16_t getMPUWakePowerByCalc(void);

void setHALReport(uint8_t dataSort);

uint8_t isStandbyEnabled = 0;

/* ADC sample count */
#define ADC_SAMPLE_COUNT  (10)

#define THREADSTACKSIZE   (768)

/* ADC conversion result variables */
uint16_t adcValue0;
uint16_t adcValue1[ADC_SAMPLE_COUNT];
//ADC转换及初始化结果
uint8_t adcInitResult = 0x02;
uint8_t adcConvertResult = 0xF0;
int_fast16_t res;
uint16_t rtcTickPeriod;

#include "ex_include_tirtos.h"
#include "scif.h"
#include "string.h"
//#include "auxOneShot.h"

#define BV(n)               (1 << (n))

// Display error message if the SCIF driver has been generated with incorrect operating system setting
#ifndef SCIF_OSAL_TIRTOS_H
    #error "SCIF driver has incorrect operating system configuration for this example. Please change to 'TI-RTOS' in the Sensor Controller Studio project panel and re-generate the driver."
#endif

// Display error message if the SCIF driver has been generated with incorrect target chip package
#ifndef SCIF_TARGET_CHIP_PACKAGE_QFN48_7X7_RGZ
    #error "SCIF driver has incorrect target chip package configuration for this example. Please change to 'QFN48 7x7 RGZ' in the Sensor Controller Studio project panel and re-generate the driver."
#endif

// Task data
Task_Struct myTask;
Char myTaskStack[1024];

// UART driver objects
UART_Handle uHandle;
UART_Params uParams;

// Line buffer for UART printing
char pLine[32];

void scTaskAlertCallback(void) {

} // taskAlertCallback

void scCtrlReadyCallback(void) {

} // ctrlReadyCallback

//ADC相关的数据采集函数========================================================
uint16_t getADCData(void){
  //ADC采集区的数据头尾
  uint16_t adcTempResult = 0;
  adcTempResult = scifTaskData.muscleBattery.output.muscleValue;
  return adcTempResult;
}
uint16_t smoothedEMGData = 0;
void setSmoothedEMGData(uint16_t smoothedData){
  smoothedEMGData = smoothedData;
}
uint16_t getSmoothedEMGData(void){
  return smoothedEMGData;
}
uint16_t getBatteryData(void){
  //ADC采集区的数据头尾
  uint16_t adcTempResult = 0;
  adcTempResult = scifTaskData.muscleBattery.output.batteryValue;
  return adcTempResult;
}

//从这里开始是MPL相关==========================================================
//=============================================================================

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "invensense.h"
#include "invensense_adv.h"
#include "eMPL_outputs.h"
#include "mltypes.h"
#include "mpu.h"
#include "log.h"
#include "packet.h"
#include "delay.h"

/* Private typedef -----------------------------------------------------------*/
/* Data read from MPL. */
#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)
#define PRINT_COMPASS   (0x08)
#define PRINT_EULER     (0x10)
#define PRINT_ROT_MAT   (0x20)
#define PRINT_HEADING   (0x40)
#define PRINT_PEDO      (0x80)
#define PRINT_LINEAR_ACCEL (0x100)
#define PRINT_GRAVITY_VECTOR (0x200)

volatile uint32_t hal_timestamp = 0;
#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)
#define COMPASS_ON      (0x04)

#define MOTION          (0)
#define NO_MOTION       (1)

#define FLASH_SIZE      (512)
#define FLASH_MEM_START ((void*)0x1800)

#define PEDO_READ_MS    (1000)
#define TEMP_READ_MS    (500)
#define COMPASS_READ_MS (100)
struct rx_s {
    unsigned char header[3];
    unsigned char cmd;
};
struct hal_s {
    unsigned char lp_accel_mode;
    unsigned char sensors;
    unsigned char dmp_on;
    unsigned char wait_for_tap;
    volatile unsigned char new_gyro; // <-- 这个变量特别关键！volatile意思是无论怎样都不会被编译器的优化行为省略
    unsigned char motion_int_mode;
    unsigned long no_dmp_hz;
    unsigned long next_pedo_ms;
    unsigned long next_temp_ms;
    unsigned long next_compass_ms;
    unsigned int report;
    unsigned short dmp_features;
    struct rx_s rx;
};
static struct hal_s hal = {0};

/* USB RX binary semaphore. Actually, it's just a flag. Not included in struct
 * because it's declared extern elsewhere.
 */
volatile unsigned char rx_new;

unsigned char *mpl_key = (unsigned char*)"eMPL 5.1";

/* Platform-specific information. Kinda like a boardfile. */
struct platform_data_s {
    signed char orientation[9];
};

/* The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from the
 * driver(s).
 * TODO: The following matrices refer to the configuration on internal test
 * boards at Invensense. If needed, please modify the matrices to match the
 * chip-to-body matrix for your particular set up.
 */
static struct platform_data_s gyro_pdata = {
    .orientation = { 1, 0, 0,
                     0, 1, 0,
                     0, 0, 1}
};

#if defined MPU9150 || defined MPU9250
static struct platform_data_s compass_pdata = {
    .orientation = { 0, 1, 0,
                     1, 0, 0,
                     0, 0, -1}
};
#define COMPASS_ENABLED 0
#elif defined AK8975_SECONDARY
static struct platform_data_s compass_pdata = {
    .orientation = {-1, 0, 0,
                     0, 1, 0,
                     0, 0,-1}
};
#define COMPASS_ENABLED 0
#elif defined AK8963_SECONDARY
static struct platform_data_s compass_pdata = {
    .orientation = {-1, 0, 0,
                     0,-1, 0,
                     0, 0, 1}
};
#define COMPASS_ENABLED 0
#endif

//注意这里，原STM32F4程序中的empl_send是从UART口送出的
//移植到CC2640R2F后要全部都从蓝牙输出
static void read_from_mpl(void)
{
  if(enableReadMPL == 0){
    //关闭了readMPL的功能
    return;
  }
    
    long msg, data[9];
    int8_t accuracy;
    unsigned long timestamp;
    float float_data[3] = {0};

    if (inv_get_sensor_type_quat(data, &accuracy, (inv_time_t*)&timestamp)) {
       /* Sends a quaternion packet to the PC. Since this is used by the Python
        * test app to visually represent a 3D quaternion, it's sent each time
        * the MPL has new data.
        */
        eMPL_send_quat(data);

        /* Specific data packets can be sent or suppressed using USB commands. */
      if (hal.report & PRINT_QUAT) {
            eMPL_send_data(PACKET_DATA_QUAT, data);
      }
    }

    if (hal.report & PRINT_ACCEL) {
      inv_get_sensor_type_accel(data, &accuracy, (inv_time_t*)&timestamp);
      //if (inv_get_sensor_type_accel(data, &accuracy, (inv_time_t*)&timestamp)){
            eMPL_send_data(PACKET_DATA_ACCEL, data);
      //}
    }
    if (hal.report & PRINT_GYRO) {
      inv_get_sensor_type_gyro(data, &accuracy, (inv_time_t*)&timestamp);
      //if (inv_get_sensor_type_gyro(data, &accuracy, (inv_time_t*)&timestamp)){
            eMPL_send_data(PACKET_DATA_GYRO, data);
      //}
    }
#ifdef COMPASS_ENABLED
    if (hal.report & PRINT_COMPASS) {
      inv_get_sensor_type_compass(data, &accuracy, (inv_time_t*)&timestamp);
      //if (inv_get_sensor_type_compass(data, &accuracy, (inv_time_t*)&timestamp)){
            eMPL_send_data(PACKET_DATA_COMPASS, data);
      //}
    }
#endif
    if (hal.report & PRINT_EULER) {
      inv_get_sensor_type_euler(data, &accuracy, (inv_time_t*)&timestamp);
      //if (inv_get_sensor_type_euler(data, &accuracy, (inv_time_t*)&timestamp)){
            eMPL_send_data(PACKET_DATA_EULER, data);
      //}
    }
    if (hal.report & PRINT_ROT_MAT) {
      inv_get_sensor_type_rot_mat(data, &accuracy, (inv_time_t*)&timestamp);
      //if (inv_get_sensor_type_rot_mat(data, &accuracy, (inv_time_t*)&timestamp)){
            eMPL_send_data(PACKET_DATA_ROT, data);
      //}
    }
    if (hal.report & PRINT_HEADING) {
      inv_get_sensor_type_heading(data, &accuracy, (inv_time_t*)&timestamp);
      //if (inv_get_sensor_type_heading(data, &accuracy, (inv_time_t*)&timestamp)){
            eMPL_send_data(PACKET_DATA_HEADING, data);
      //}
    }
    if (hal.report & PRINT_LINEAR_ACCEL) {
      inv_get_sensor_type_linear_acceleration(float_data, &accuracy, (inv_time_t*)&timestamp);
      eMPL_send_float_data(PACKET_DATA_LINEAR_ACCEL, float_data);
        //if (inv_get_sensor_type_linear_acceleration(float_data, &accuracy, (inv_time_t*)&timestamp)) {
        	//MPL_LOGI("Linear Accel: %7.5f %7.5f %7.5f\r\n", float_data[0], float_data[1], float_data[2]);                                        
         //}
    }
    if (hal.report & PRINT_GRAVITY_VECTOR) {
      inv_get_sensor_type_gravity(float_data, &accuracy, (inv_time_t*)&timestamp);
      eMPL_send_float_data(PACKET_GRAVITY_VECTOR, float_data);
      //if (inv_get_sensor_type_gravity(float_data, &accuracy, (inv_time_t*)&timestamp)){
            	//MPL_LOGI("Gravity Vector: %7.5f %7.5f %7.5f\r\n", float_data[0], float_data[1], float_data[2]);
      //}
    }
    if (hal.report & PRINT_PEDO) {
        unsigned long timestamp;
        get_tick_count(&timestamp);
        if (timestamp > hal.next_pedo_ms) {
            hal.next_pedo_ms = timestamp + PEDO_READ_MS;
            unsigned long step_count, walk_time;
            dmp_get_pedometer_step_count(&step_count);
            dmp_get_pedometer_walk_time(&walk_time);
            MPL_LOGI("Walked %ld steps over %ld milliseconds..\n", step_count,
            walk_time);
        }
    }

    /* Whenever the MPL detects a change in motion state, the application can
     * be notified. For this example, we use an LED to represent the current
     * motion state.
     */
    msg = inv_get_message_level_0(INV_MSG_MOTION_EVENT |
            INV_MSG_NO_MOTION_EVENT);
    if (msg) {
        if (msg & INV_MSG_MOTION_EVENT) {
            //MPL_LOGI("Motion!\n");
        } else if (msg & INV_MSG_NO_MOTION_EVENT) {
            //MPL_LOGI("No motion!\n");
        }
    }
}

#ifdef COMPASS_ENABLED
void send_status_compass() {
	long data[3] = { 0 };
	int8_t accuracy = { 0 };
	unsigned long timestamp;
	inv_get_compass_set(data, &accuracy, (inv_time_t*) &timestamp);
	//MPL_LOGI("Compass: %7.4f %7.4f %7.4f ", data[0]/65536.f, data[1]/65536.f, data[2]/65536.f);
	//MPL_LOGI("Accuracy= %d\r\n", accuracy);

}
#endif

static void tap_cb(unsigned char direction, unsigned char count)
{
    switch (direction) {
    case TAP_X_UP:
        //MPL_LOGI("Tap X+ ");
        break;
    case TAP_X_DOWN:
        //MPL_LOGI("Tap X- ");
        break;
    case TAP_Y_UP:
        //MPL_LOGI("Tap Y+ ");
        break;
    case TAP_Y_DOWN:
        //MPL_LOGI("Tap Y- ");
        break;
    case TAP_Z_UP:
        //MPL_LOGI("Tap Z+ ");
        break;
    case TAP_Z_DOWN:
        //MPL_LOGI("Tap Z- ");
        break;
    default:
        return;
    }
    //MPL_LOGI("x%d\n", count);
    return;
}

static void android_orient_cb(unsigned char orientation)
{
	switch (orientation) {
	case ANDROID_ORIENT_PORTRAIT:
        //MPL_LOGI("Portrait\n");
        break;
	case ANDROID_ORIENT_LANDSCAPE:
        //MPL_LOGI("Landscape\n");
        break;
	case ANDROID_ORIENT_REVERSE_PORTRAIT:
        //MPL_LOGI("Reverse Portrait\n");
        break;
	case ANDROID_ORIENT_REVERSE_LANDSCAPE:
        //MPL_LOGI("Reverse Landscape\n");
        break;
	default:
		return;
	}
}


static inline void run_self_test(void)
{
    int result;
    long gyro[3], accel[3];

#if defined (MPU6500) || defined (MPU9250)
    result = mpu_run_6500_self_test(gyro, accel, 0);
#elif defined (MPU6050) || defined (MPU9150)
    result = mpu_run_self_test(gyro, accel);
#endif
    if (result == 0x7) {
      /*
	MPL_LOGI("Passed!\n");
        MPL_LOGI("accel: %7.4f %7.4f %7.4f\n",
                    accel[0]/65536.f,
                    accel[1]/65536.f,
                    accel[2]/65536.f);
        MPL_LOGI("gyro: %7.4f %7.4f %7.4f\n",
                    gyro[0]/65536.f,
                    gyro[1]/65536.f,
                    gyro[2]/65536.f);
      */
        /* Test passed. We can trust the gyro data here, so now we need to update calibrated data*/

#ifdef USE_CAL_HW_REGISTERS
        /*
         * This portion of the code uses the HW offset registers that are in the MPUxxxx devices
         * instead of pushing the cal data to the MPL software library
         */
        unsigned char i = 0;

        for(i = 0; i < 3; i++) {
        	gyro[i] = (long)(gyro[i] * 32.8f); //convert to +-1000dps
        	accel[i] *= 2048.f; //convert to +-16G
        	accel[i] = accel[i] >> 16;
        	gyro[i] = (long)(gyro[i] >> 16);
        }

        mpu_set_gyro_bias_reg(gyro);

#if defined (MPU6500) || defined (MPU9250)
        mpu_set_accel_bias_6500_reg(accel);
#elif defined (MPU6050) || defined (MPU9150)
        mpu_set_accel_bias_6050_reg(accel);
#endif
#else
        /* Push the calibrated data to the MPL library.
         *
         * MPL expects biases in hardware units << 16, but self test returns
		 * biases in g's << 16.
		 */
    	unsigned short accel_sens;
    	float gyro_sens;

		mpu_get_accel_sens(&accel_sens);
		accel[0] *= accel_sens;
		accel[1] *= accel_sens;
		accel[2] *= accel_sens;
		inv_set_accel_bias(accel, 3);
		mpu_get_gyro_sens(&gyro_sens);
		gyro[0] = (long) (gyro[0] * gyro_sens);
		gyro[1] = (long) (gyro[1] * gyro_sens);
		gyro[2] = (long) (gyro[2] * gyro_sens);
		inv_set_gyro_bias(gyro, 3);
#endif
    }
    else {
      if (!(result & 0x1)){
                //MPL_LOGE("Gyro failed.\n");
      }
      if (!(result & 0x2)){
                //MPL_LOGE("Accel failed.\n");
      }
      if (!(result & 0x4)){
                //MPL_LOGE("Compass failed.\n");
      }
     }

}

//=============================================================================
//=============================================================================

/*******************************************************************************
 * MACROS
 */

/*******************************************************************************
 * CONSTANTS
 */

#if defined( USE_FPGA )
  #define RFC_MODE_BLE                 PRCM_RFCMODESEL_CURR_MODE1
  #define RFC_MODE_ANT                 PRCM_RFCMODESEL_CURR_MODE4
  #define RFC_MODE_EVERYTHING_BUT_ANT  PRCM_RFCMODESEL_CURR_MODE5
  #define RFC_MODE_EVERYTHING          PRCM_RFCMODESEL_CURR_MODE6
  //
  #define SET_RFC_BLE_MODE(mode) HWREG( PRCM_BASE + PRCM_O_RFCMODESEL ) = (mode)
#endif // USE_FPGA

/*******************************************************************************
 * TYPEDEFS
 */

/*******************************************************************************
 * LOCAL VARIABLES
 */

/*******************************************************************************
 * GLOBAL VARIABLES
 */

#ifdef CC1350_LAUNCHXL
#ifdef POWER_SAVING
// Power Notify Object for wake-up callbacks
Power_NotifyObj rFSwitchPowerNotifyObj;
static uint8_t rFSwitchNotifyCb(uint8_t eventType, uint32_t *eventArg,
                                uint32_t *clientArg);
#endif //POWER_SAVING

PIN_State  radCtrlState;
PIN_Config radCtrlCfg[] =
{
  Board_DIO1_RFSW   | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW  | PIN_PUSHPULL | PIN_DRVSTR_MAX, /* RF SW Switch defaults to 2.4GHz path*/
  Board_DIO30_SWPWR | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX, /* Power to the RF Switch */
  PIN_TERMINATE
};
PIN_Handle radCtrlHandle;
#endif //CC1350_LAUNCHXL

/*******************************************************************************
 * EXTERNS
 */

extern void AssertHandler(uint8 assertCause, uint8 assertSubcause);

extern Display_Handle dispHandle;

/*******************************************************************************
 * MPL 相关服务
 */

static uint8_t mplIsInited = 0;
unsigned char accel_fsr,  new_temp = 0;
unsigned short gyro_rate, gyro_fsr;
unsigned long timestamp;
unsigned char new_compass = 0;
unsigned short compass_fsr;
inv_error_t result;
struct int_param_s int_param;

int initMPLService(void){
  //初始化：占用1蓝牙周期
  result = mpu_init(&int_param);
  if (result) {
    //MPU初始化失败了
      //MPL_LOGE("Could not initialize gyro.\n");
    sprintf(pLine, "%d\r\n", result);
    mplIsInited = 2;
    return mplIsInited;
  }
  
  result = inv_init_mpl();
  if (result) {
    //MPL初始化失败了
      //MPL_LOGE("Could not initialize MPL.\n");
    sprintf(pLine, "%d\r\n", result);
    mplIsInited = 2;
    return mplIsInited;
  }
  return 1;
}

int initMPLService_STEP1_TEST(void){
  //此步骤占1个周期
  inv_enable_quaternion(); // <-- 6轴融合算法启动
  inv_enable_fast_nomot(); // <-- 陀螺仪实时自动校准启动
  return 1;
}

int initMPLService_STEP1(void){
  //此步骤占1个周期
  inv_enable_quaternion(); // <-- 6轴融合算法启动
  return 1;
}

int initMPLService_STEP1B(void){
  //此步骤占1个周期
  inv_enable_9x_sensor_fusion(); // <-- 9轴融合算法启动
  return 1;
}

int initMPLService_STEP2(void){
  //此步骤占1个周期
  inv_enable_fast_nomot(); // <-- 陀螺仪实时自动校准启动
  //inv_enable_gyro_tc(); // <-- 陀螺仪温度自动补偿启动
  return 1;
}

int initMPLService_STEP3(void){
  //此步骤占2个周期
#ifdef COMPASS_ENABLED
  //inv_enable_vector_compass_cal(); // <-- 磁场自校准启动
#endif
  return 1;
}

int initMPLService_STEP3B(void){
  //此步骤占2个周期
#ifdef COMPASS_ENABLED
  inv_enable_magnetic_disturbance(); // <-- 磁场扰乱检测启动
#endif
  return 1;
}

int initMPLService_STEP4(void){
  //此步骤占1个周期
  inv_enable_eMPL_outputs();
  result = inv_start_mpl();
  if (result == INV_ERROR_NOT_AUTHORIZED) {
    //未识别MPL
      //while (1) {
          //MPL_LOGE("Not authorized.\n");
      //}
    sprintf(pLine, "%d\r\n", result);
    mplIsInited = 2;
    return mplIsInited;
  }
  if (result) {
    //启动MPL失败
      //MPL_LOGE("Could not start the MPL.\n");
    sprintf(pLine, "%d\r\n", result);
    mplIsInited = 2;
    return mplIsInited;
  }
  mplIsInited = 1;
  return 1;
}

int initMPLService_STEP5(void){
  //收尾工作：占用1蓝牙周期
#ifdef COMPASS_ENABLED
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
#else
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
#endif
    mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_set_sample_rate(DEFAULT_MOTION_FREQ);
#ifdef COMPASS_ENABLED
    mpu_set_compass_sample_rate(1000 / COMPASS_READ_MS);
#endif
    mpu_get_sample_rate(&gyro_rate);
    mpu_get_gyro_fsr(&gyro_fsr);
    mpu_get_accel_fsr(&accel_fsr);
#ifdef COMPASS_ENABLED
    mpu_get_compass_fsr(&compass_fsr);
#endif
    inv_set_gyro_sample_rate(1000000L / gyro_rate);
    inv_set_accel_sample_rate(1000000L / gyro_rate);
#ifdef COMPASS_ENABLED
    inv_set_compass_sample_rate(COMPASS_READ_MS * 1000L);
#endif
    inv_set_gyro_orientation_and_scale(
            inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
            (long)gyro_fsr<<15);
    inv_set_accel_orientation_and_scale(
            inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
            (long)accel_fsr<<15);
#ifdef COMPASS_ENABLED
    inv_set_compass_orientation_and_scale(
            inv_orientation_matrix_to_scalar(compass_pdata.orientation),
            (long)compass_fsr<<15);
#endif
  
#ifdef COMPASS_ENABLED
    hal.sensors = ACCEL_ON | GYRO_ON | COMPASS_ON;
#else
    hal.sensors = ACCEL_ON | GYRO_ON;
#endif
    hal.dmp_on = 0;
    hal.report = 0;
    hal.rx.cmd = 0;
    hal.next_pedo_ms = 0;
    hal.next_compass_ms = 0;
    hal.next_temp_ms = 0;

  get_tick_count(&timestamp);
  
  dmp_load_motion_driver_firmware();
    dmp_set_orientation(
        inv_orientation_matrix_to_scalar(gyro_pdata.orientation));
    dmp_register_tap_cb(tap_cb);
    dmp_register_android_orient_cb(android_orient_cb);
    
    hal.dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
        DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
        DMP_FEATURE_GYRO_CAL;
    dmp_enable_feature(hal.dmp_features);
    dmp_set_fifo_rate(DEFAULT_MOTION_FREQ);
    mpu_set_dmp_state(1);
    hal.dmp_on = 1;
    
    //向外输出什么数据？
    setHALReport(DEFAULT_OUTPUT_FORMAT);
    
    //初开机的时候设定为1级功耗，将拍击连接标志重置，并等待初始化完毕
    setSYSTEMWorkLevel(SYSTEM_ENERGY_LEVEL1);
    enableConnectBLE = 0;
    
    return 1;
}

//设置：hal.report应该输出什么类型的数据
void setHALReport(uint8_t dataSort){
  hal.report = 0;
  switch(dataSort){
  case 0:
    //加速度
    hal.report = PRINT_ACCEL;
    break;
  case 1:
    //角速度
    hal.report = PRINT_GYRO;
    break;
  case 2:
    //磁场
    if(AXIS_9_FUSION_MODE){
      hal.report = PRINT_COMPASS;
    }else{
      //设置错误或未知情况下默认为输出欧拉角
      hal.report = PRINT_EULER;
      dataSort = 3;
    }
    break;
  case 3:
    //欧拉角
    hal.report = PRINT_EULER;
    break;
  case 4:
    //旋转矩阵
    hal.report = PRINT_ROT_MAT;
    break;
  case 5:
    //旋转四元数
    hal.report = PRINT_QUAT;
    break;
  case 6:
    //线性加速度
    hal.report = PRINT_LINEAR_ACCEL;
    break;
  case 7:
    //重力矢量
    hal.report = PRINT_GRAVITY_VECTOR;
    break;
  default:
    //欧拉角
    hal.report = PRINT_EULER;
    dataSort = 3;
    break;
  }
  changeOutputValueSort(dataSort);
}

//改变dmp/mpu运算速率
void setMotionRate(uint8_t motionRate){
  dmp_set_fifo_rate(motionRate);
  inv_set_quat_sample_rate(1000000L / motionRate);
  mpu_set_sample_rate(motionRate);
  inv_set_gyro_sample_rate(1000000L / motionRate);
  inv_set_accel_sample_rate(1000000L / motionRate);
}

//原本的gyro_setup是写在输入操作判定区内部的，现在移动到外面来
/* Handle sensor on/off combinations. */
static void setup_gyro(void)
{
    unsigned char mask = 0, lp_accel_was_on = 0;
    if (hal.sensors & ACCEL_ON)
        mask |= INV_XYZ_ACCEL;
    if (hal.sensors & GYRO_ON) {
        mask |= INV_XYZ_GYRO;
        lp_accel_was_on |= hal.lp_accel_mode;
    }
#ifdef COMPASS_ENABLED
    if (hal.sensors & COMPASS_ON) {
        mask |= INV_XYZ_COMPASS;
        lp_accel_was_on |= hal.lp_accel_mode;
    }
#endif
    mpu_set_sensors(mask);
    mpu_configure_fifo(mask);
    //在磁场计和角速计至少有一个被打开的状况下，要检查是不是从LPA模式脱出
    if (lp_accel_was_on) {
        unsigned short rate;
        hal.lp_accel_mode = 0;
        // Switching out of LP accel, notify MPL of new accel sampling rate.
        mpu_get_sample_rate(&rate);
        inv_set_accel_sample_rate(1000000L / rate);
    }
}

//中断保护。这部分内容原本必须在MPU向MCU发送中断信号时才执行
//意义：MPU必须先向MCU报告“准备好”才能作陀螺仪的融合运算
//目前在CC2640R2F中暂时屏蔽这个功能，在今后优化的时候加回来
void gyro_data_ready_cb(void)
{
    hal.new_gyro = 1;
}
                     
//传感器的启动和休眠
void mpuHibernate(void){
  //如果MPU处于低功耗加速计状态，就取消它
  hal.sensors &= ~ACCEL_ON;
  hal.sensors &= ~GYRO_ON;
  hal.sensors &= ~COMPASS_ON;
  setup_gyro();
  inv_accel_was_turned_off();
  inv_gyro_was_turned_off();
  inv_compass_was_turned_off();
  hal.dmp_on = 0;
  hal.lp_accel_mode = 0;
  mpu_set_dmp_state(0);
  enableReadMPL = 0;
}
void adcOff(void){
  if(enableReadADC == 0){
    return;
  }
  scifStopTasksNbl(BV(SCIF_MUSCLE_BATTERY_TASK_ID));
  scifStopRtcTicks();
  enableReadADC = 0;
}
void mpuWakeUp(void){
  if(enableReadMPL > 0){
    return;
  }
  mpuOutOfLPMIM();
  hal.sensors |= ACCEL_ON;
  hal.sensors |= GYRO_ON;
  hal.sensors |= COMPASS_ON;
  //之前设定了GYRO_ON，此语句自动取消LPA模式
  setup_gyro();
  hal.dmp_on = 1;
  mpu_set_dmp_state(1);
  enableReadMPL = 1;
}
void adcOn(void){
  if(enableReadADC > 0){
    return;
  }
  scifResetTaskStructs(BV(SCIF_MUSCLE_BATTERY_TASK_ID), BV(SCIF_STRUCT_CFG));
  scifStartRtcTicksNow(rtcTickPeriod);
  scifStartTasksNbl(BV(SCIF_MUSCLE_BATTERY_TASK_ID));
  enableReadADC = 1;
}
void mpuIntoLPA(void){
  if(hal.dmp_on){
    return;
  }
  if(hal.lp_accel_mode > 0){
    //已经处于低功耗加速计模式时不再重复设定
    return;
  }
  //mpu从休眠模式进入低功耗加速计状态
  //否则必须先休眠再低功耗
  mpu_lp_accel_mode(20);
  mpu_set_int_latched(0);
  hal.sensors &= ~(GYRO_ON|COMPASS_ON);
  hal.sensors |= ACCEL_ON;
  hal.lp_accel_mode = 1;
  mpuIntoLPMIM();
}
void mpuIntoLPMIM(void){
  if(hal.motion_int_mode > 0){
    //已经处于低功耗加速计模式时不再重复设定
    return;
  }
  // Enable motion interrupt
  hal.motion_int_mode = 1;
  mpu_lp_motion_interrupt(getMPUWakePowerByCalc(), 5, 20);
  // Notify the MPL that contiguity was broken
  inv_accel_was_turned_off();
  inv_gyro_was_turned_off();
  inv_compass_was_turned_off();
  inv_quaternion_sensor_was_turned_off();
}
void mpuOutOfLPMIM(void){
  if(hal.motion_int_mode == 0){
    //已经脱离低功耗加速计模式时不再重复设定
    return;
  }
  // Restore the previous sensor configuration
  mpu_lp_motion_interrupt(0, 0, 0);
  hal.motion_int_mode = 0;
}

//定义MPU的几种工作状态
uint8_t systemWorkLevel = 127; // <-- 刚刚开机时不属于以下任何一种情况
void setSYSTEMWorkLevel(uint8_t workLevel){
  if(systemWorkLevel > SYSTEM_ENERGY_LEVEL1 && systemWorkLevel == workLevel){
    //如果设置参数等于当前工作等级，就什么也不做
    return;
  }
  systemWorkLevel = workLevel;
  //改变LED工作状态（如果LPA周期未结束就不变灯）
  if(getLPALampLoop() <= 0){
    setLEDWorkBlink(workLevel);
  }
  switch(workLevel){
  case SYSTEM_ENERGY_LEVEL0:
    //0级工作状态
    motionIsOpen = 0;
    //mpuHibernate();
    //mpuIntoLPA();
    adcOff();
    emgPortOpen(0);
    //0级工作状态时暂停MCU（注意0级工作状态只能由1级切换到，或者开机时切换到）
    if(ENABLE_AUTO_SLEEP > 0){
      enableStandby(1);
      PowerCC26XX_standbyPolicy();
    }
    break;
  case SYSTEM_ENERGY_LEVEL1:
    Util_stopClock(&emgPauseClock);
    //pinDark(1);
    if(ENABLE_AUTO_SLEEP > 0){
      enableStandby(1);
    }
    //进入短广播还是长广播
    //if(longAdvertiseFlag <= 0){
      startAdvertising(); // <-- 该方法只有这个函数才调用
    //}else{
      //startLongAdvertising(); // <-- 该方法只有这个函数才调用
    //}
    motionIsOpen = 0;
    if(ENABLE_AUTO_SLEEP == 0){
      mpuOutOfLPMIM();
      mpuHibernate();
    }else{
      if(hal.motion_int_mode == 0){
        mpuHibernate();
        mpuIntoLPA();
      }
    }
    setEMGOpen(0);
    adcOn(); // <-- 原Off
    emgPortOpen(0);
    break;
  case SYSTEM_ENERGY_LEVEL1_LPA:
    //1级待连接工作状态
    if(LAMP_TEST_MODE > 0){
      pinDark(1);
      PIN_GLight();
    }
    Util_stopClock(&emgPauseClock);
    motionIsOpen = 0;
    enableStandby(1);
    if(ENABLE_AUTO_SLEEP == 0){
      mpuHibernate();
      mpuIntoLPA();
    }
    adcOff();
    emgPortOpen(0);
    break;
  case SYSTEM_ENERGY_LEVEL2:
    //2级工作状态
    if(LAMP_TEST_MODE > 0){
      PIN_GLight();
      PIN_RLight();
    }
    motionIsOpen = 0;
    enableStandby(0);
    mpuOutOfLPMIM();
    mpuHibernate();
    adcOn();
    emgPortOpen(0);
    break;
  case SYSTEM_ENERGY_LEVEL3:
    //3级工作状态
    //根据动作种类判定Motion模块还是EMG模块打开
    //目前的2、3级工况判定依据是：EMG或Motion至少有一个模块被打开时为3级工况，否则为2级
    //setMotionOpen(1);
    emgPortOpen(1);
    adcOn();
    break;
  default:
    break;
  }
}
uint8_t getSYSTEMWorkLevel(void){
  return systemWorkLevel;
}
//即时判断系统应该如何亮灯
uint8_t ledWorkLevel = 127;
void setLEDWorkBlink(uint8_t workLevel){
  if(workLevel <= SYSTEM_ENERGY_LEVEL1){
    //关机、待机
    ledWorkLevel = 0;
    if(isCharging() <= 0){
      //不在充电状态就灭灯
      pinDark(1);
    }
  }else if(workLevel >= SYSTEM_ENERGY_LEVEL2){
    //正常工作
    ledWorkLevel = 2;
    if(isBatteryLow() > 0){
      pinRLightBlink();
    }else{
      pinGLightBlink();
    }
  }else{
    //1级半特殊处理
    ledWorkLevel = 1;
    pinGLightConst();
  }
}

//注意，这个方法目前写在蓝牙的主循环内，因此MPL的实际采样率不会大于
//蓝牙事件的循环速率
void mplTaskAtMainLoop(void){
  
  if(enableReadMPL == 0){
    //关闭了readMPL的功能
    return;
  }
  
  //测试阶段暂时屏蔽中断线的功能，正式产品中要加回来
  gyro_data_ready_cb();
  
  unsigned long sensor_timestamp;
    int new_data = 0;
    //if (USART_GetITStatus(USART2, USART_IT_RXNE)) {
        /* A byte has been received via USART. See handle_input for a list of
         * valid commands.
         */
        //USART_ClearITPendingBit(USART2, USART_IT_RXNE);
        //handle_input();
    //}
    get_tick_count(&timestamp);

#ifdef COMPASS_ENABLED
        /* We're not using a data ready interrupt for the compass, so we'll
         * make our compass reads timer-based instead.
         */
        if ((timestamp > hal.next_compass_ms) && !hal.lp_accel_mode &&
            hal.new_gyro && (hal.sensors & COMPASS_ON)) {
            hal.next_compass_ms = timestamp + COMPASS_READ_MS;
            new_compass = 1;
        }
#endif
        /* Temperature data doesn't need to be read with every gyro sample.
         * Let's make them timer-based like the compass reads.
         */
        if (timestamp > hal.next_temp_ms) {
            hal.next_temp_ms = timestamp + TEMP_READ_MS;
            new_temp = 1;
        }

    if (!hal.sensors || !hal.new_gyro) {
        //continue;
      //不在循环体内就不能使用continue打断
      return;
    }    

        if (hal.new_gyro && hal.lp_accel_mode) {
            short accel_short[3];
            long accel[3];
            mpu_get_accel_reg(accel_short, &sensor_timestamp);
            accel[0] = (long)accel_short[0];
            accel[1] = (long)accel_short[1];
            accel[2] = (long)accel_short[2];
            inv_build_accel(accel, 0, sensor_timestamp);
            new_data = 1;
            hal.new_gyro = 0;
        } else if (hal.new_gyro && hal.dmp_on) {
            short gyro[3], accel_short[3], sensors;
            unsigned char more;
            long accel[3], quat[4], temperature;
            /* This function gets new data from the FIFO when the DMP is in
             * use. The FIFO can contain any combination of gyro, accel,
             * quaternion, and gesture data. The sensors parameter tells the
             * caller which data fields were actually populated with new data.
             * For example, if sensors == (INV_XYZ_GYRO | INV_WXYZ_QUAT), then
             * the FIFO isn't being filled with accel data.
             * The driver parses the gesture data to determine if a gesture
             * event has occurred; on an event, the application will be notified
             * via a callback (assuming that a callback function was properly
             * registered). The more parameter is non-zero if there are
             * leftover packets in the FIFO.
             */
            dmp_read_fifo(gyro, accel_short, quat, &sensor_timestamp, &sensors, &more);
            if (!more)
                hal.new_gyro = 0;
            if (sensors & INV_XYZ_GYRO) {
                /* Push the new data to the MPL. */
                inv_build_gyro(gyro, sensor_timestamp);
                new_data = 1;
                if (new_temp) {
                    new_temp = 0;
                    /* Temperature only used for gyro temp comp. */
                    mpu_get_temperature(&temperature, &sensor_timestamp);
                    inv_build_temp(temperature, sensor_timestamp);
                }
            }
            if (sensors & INV_XYZ_ACCEL) {
                accel[0] = (long)accel_short[0];
                accel[1] = (long)accel_short[1];
                accel[2] = (long)accel_short[2];
                inv_build_accel(accel, 0, sensor_timestamp);
                new_data = 1;
            }
            if (sensors & INV_WXYZ_QUAT) {
                inv_build_quat(quat, 0, sensor_timestamp);
                new_data = 1;
            }
        } else if (hal.new_gyro) {
            short gyro[3], accel_short[3];
            unsigned char sensors, more;
            long accel[3], temperature;
            /* This function gets new data from the FIFO. The FIFO can contain
             * gyro, accel, both, or neither. The sensors parameter tells the
             * caller which data fields were actually populated with new data.
             * For example, if sensors == INV_XYZ_GYRO, then the FIFO isn't
             * being filled with accel data. The more parameter is non-zero if
             * there are leftover packets in the FIFO. The HAL can use this
             * information to increase the frequency at which this function is
             * called.
             */
            hal.new_gyro = 0;
            mpu_read_fifo(gyro, accel_short, &sensor_timestamp,
                &sensors, &more);
            if (more)
                hal.new_gyro = 1;
            if (sensors & INV_XYZ_GYRO) {
                /* Push the new data to the MPL. */
                inv_build_gyro(gyro, sensor_timestamp);
                new_data = 1;
                if (new_temp) {
                    new_temp = 0;
                    /* Temperature only used for gyro temp comp. */
                    mpu_get_temperature(&temperature, &sensor_timestamp);
                    inv_build_temp(temperature, sensor_timestamp);
                }
            }
            if (sensors & INV_XYZ_ACCEL) {
                accel[0] = (long)accel_short[0];
                accel[1] = (long)accel_short[1];
                accel[2] = (long)accel_short[2];
                inv_build_accel(accel, 0, sensor_timestamp);
                new_data = 1;
            }
        }
#ifdef COMPASS_ENABLED
        if (new_compass) {
            short compass_short[3];
            long compass[3];
            new_compass = 0;
            /* For any MPU device with an AKM on the auxiliary I2C bus, the raw
             * magnetometer registers are copied to special gyro registers.
             */
            if (!mpu_get_compass_reg(compass_short, &sensor_timestamp)) {
                compass[0] = (long)compass_short[0];
                compass[1] = (long)compass_short[1];
                compass[2] = (long)compass_short[2];
                /* NOTE: If using a third-party compass calibration library,
                 * pass in the compass data in uT * 2^16 and set the second
                 * parameter to INV_CALIBRATED | acc, where acc is the
                 * accuracy from 0 to 3.
                 */
                inv_build_compass(compass, 0, sensor_timestamp);
            }
            new_data = 1;
        }
#endif
        if (new_data) {
            inv_execute_on_data();
            /* This function reads bias-compensated sensor data and sensor
             * fusion outputs from the MPL. The outputs are formatted as seen
             * in eMPL_outputs.c. This function only needs to be called at the
             * rate requested by the host.
             */
            read_from_mpl();
        }
}

/*******************************************************************************
 * @fn          Main
 *
 * @brief       Application Main
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
int main()
{
  Power_init();
  
#if defined( USE_FPGA )
  HWREG(PRCM_BASE + PRCM_O_PDCTL0) &= ~PRCM_PDCTL0_RFC_ON;
  HWREG(PRCM_BASE + PRCM_O_PDCTL1) &= ~PRCM_PDCTL1_RFC_ON;
#endif // USE_FPGA

  /* Register Application callback to trap asserts raised in the Stack */
  RegisterAssertCback(AssertHandler);

  PIN_init(BoardGpioInitTable);

#ifdef CC1350_LAUNCHXL
  // Enable 2.4GHz Radio
  radCtrlHandle = PIN_open(&radCtrlState, radCtrlCfg);

#ifdef POWER_SAVING
  Power_registerNotify(&rFSwitchPowerNotifyObj,
                       PowerCC26XX_ENTERING_STANDBY | PowerCC26XX_AWAKE_STANDBY,
                       (Power_NotifyFxn) rFSwitchNotifyCb, NULL);
#endif //POWER_SAVING
#endif //CC1350_LAUNCHXL

#if defined( USE_FPGA )
  // set RFC mode to support BLE
  // Note: This must be done before the RF Core is released from reset!
  SET_RFC_BLE_MODE(RFC_MODE_BLE);
#endif // USE_FPGA

#ifdef CACHE_AS_RAM
  // retain cache during standby
  Power_setConstraint(PowerCC26XX_SB_VIMS_CACHE_RETAIN);
  Power_setConstraint(PowerCC26XX_NEED_FLASH_IN_IDLE);
#else
  // Enable iCache prefetching
  VIMSConfigure(VIMS_BASE, TRUE, TRUE);
  // Enable cache
  VIMSModeSet(VIMS_BASE, VIMS_MODE_ENABLED);
#endif //CACHE_AS_RAM

//#if !defined( POWER_SAVING ) || defined( USE_FPGA )
  /* Set constraints for Standby, powerdown and idle mode */
  // PowerCC26XX_SB_DISALLOW may be redundant
  Power_setConstraint(PowerCC26XX_SB_DISALLOW);
  Power_setConstraint(PowerCC26XX_IDLE_PD_DISALLOW);
  isStandbyEnabled = 0;
//#endif // POWER_SAVING | USE_FPGA

#ifdef ICALL_JT
  /* Update User Configuration of the stack */
  user0Cfg.appServiceInfo->timerTickPeriod = Clock_tickPeriod;
  user0Cfg.appServiceInfo->timerMaxMillisecond  = ICall_getMaxMSecs();
#endif  /* ICALL_JT */
  
  //这里是ADC相关=================================================================
  //==============================================================================
  scifOsalInit();
  scifOsalRegisterCtrlReadyCallback(scCtrlReadyCallback);
  scifOsalRegisterTaskAlertCallback(scTaskAlertCallback);
  scifInit(&scifDriverSetup);
  rtcTickPeriod = 0x00010000 / DEFAULT_EMG_FREQ;
  scifStartRtcTicksNow(rtcTickPeriod); // <-- 原10
  Util_constructClock(&emgPauseClock, emgPause_clockHandler,
                      EMG_RESET_INTERVAL, 0, false, NULL);
  //==============================================================================
  //==============================================================================
  
  //将RF Core的蓝牙功能移植到DIO14、DIO13口控制===================================
  // Map RFC_GPO0 to DIO13 
  //IOCPortConfigureSet(IOID_13, IOC_PORT_RFC_GPO0, IOC_IOMODE_NORMAL); 
  // Map RFC_GPO1 to DIO14 
  //IOCPortConfigureSet(IOID_14, IOC_PORT_RFC_GPO1, IOC_IOMODE_NORMAL);
  //==============================================================================
    
  /* Initialize ICall module */
  ICall_init();

  /* Start tasks of external images - Priority 5 */
  ICall_createRemoteTasks();

  /* Kick off profile - Priority 3 */
  GAPRole_createTask();

  //启动ADC（此步骤需判断是否允许传感器在开机时自启动）
  scifStartTasksNbl(BV(SCIF_MUSCLE_BATTERY_TASK_ID));
  
  //启动蓝牙
  SimpleBLEPeripheral_createTask();

  /* enable interrupts and start SYS/BIOS */
  BIOS_start();

  return 0;
}


/*******************************************************************************
 * @fn          AssertHandler
 *
 * @brief       This is the Application's callback handler for asserts raised
 *              in the stack.  When EXT_HAL_ASSERT is defined in the Stack
 *              project this function will be called when an assert is raised,
 *              and can be used to observe or trap a violation from expected
 *              behavior.
 *
 *              As an example, for Heap allocation failures the Stack will raise
 *              HAL_ASSERT_CAUSE_OUT_OF_MEMORY as the assertCause and
 *              HAL_ASSERT_SUBCAUSE_NONE as the assertSubcause.  An application
 *              developer could trap any malloc failure on the stack by calling
 *              HAL_ASSERT_SPINLOCK under the matching case.
 *
 *              An application developer is encouraged to extend this function
 *              for use by their own application.  To do this, add hal_assert.c
 *              to your project workspace, the path to hal_assert.h (this can
 *              be found on the stack side). Asserts are raised by including
 *              hal_assert.h and using macro HAL_ASSERT(cause) to raise an
 *              assert with argument assertCause.  the assertSubcause may be
 *              optionally set by macro HAL_ASSERT_SET_SUBCAUSE(subCause) prior
 *              to asserting the cause it describes. More information is
 *              available in hal_assert.h.
 *
 * input parameters
 *
 * @param       assertCause    - Assert cause as defined in hal_assert.h.
 * @param       assertSubcause - Optional assert subcause (see hal_assert.h).
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void AssertHandler(uint8 assertCause, uint8 assertSubcause)
{

  // check the assert cause
  switch (assertCause)
  {
    case HAL_ASSERT_CAUSE_OUT_OF_MEMORY:
      break;

    case HAL_ASSERT_CAUSE_INTERNAL_ERROR:
      // check the subcause
      if (assertSubcause == HAL_ASSERT_SUBCAUSE_FW_INERNAL_ERROR)
      {
      }
      else
      {
      }
      break;

    case HAL_ASSERT_CAUSE_ICALL_ABORT:
      HAL_ASSERT_SPINLOCK;
      break;

    case HAL_ASSERT_CAUSE_ICALL_TIMEOUT:
      HAL_ASSERT_SPINLOCK;
      break;

    case HAL_ASSERT_CAUSE_WRONG_API_CALL:
      HAL_ASSERT_SPINLOCK;
      break;

  default:
      HAL_ASSERT_SPINLOCK;
  }

  return;
}


/*******************************************************************************
 * @fn          smallErrorHook
 *
 * @brief       Error handler to be hooked into TI-RTOS.
 *
 * input parameters
 *
 * @param       eb - Pointer to Error Block.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void smallErrorHook(Error_Block *eb)
{
  for (;;);
}

#if defined (CC1350_LAUNCHXL) && defined (POWER_SAVING)
/*******************************************************************************
 * @fn          rFSwitchNotifyCb
 *
 * @brief       Power driver callback to toggle RF switch on Power state
 *              transitions.
 *
 * input parameters
 *
 * @param   eventType - The state change.
 * @param   eventArg  - Not used.
 * @param   clientArg - Not used.
 *
 * @return  Power_NOTIFYDONE to indicate success.
 */
static uint8_t rFSwitchNotifyCb(uint8_t eventType, uint32_t *eventArg,
                                uint32_t *clientArg)
{
  if (eventType == PowerCC26XX_ENTERING_STANDBY)
  {
    // Power down RF Switch
    PIN_setOutputValue(radCtrlHandle, Board_DIO30_SWPWR, 0);
  }
  else if (eventType == PowerCC26XX_AWAKE_STANDBY)
  {
    // Power up RF Switch
    PIN_setOutputValue(radCtrlHandle, Board_DIO30_SWPWR, 1);
  }

  // Notification handled successfully
  return Power_NOTIFYDONE;
}
#endif //CC1350_LAUNCHXL || POWER_SAVING

//Power Policy Change
void enableStandby(uint8_t standbyEnabled){
  if(standbyEnabled > 0){
    //允许进入待机状态
    if(isStandbyEnabled > 0){
      return;
    }
    isStandbyEnabled = 1;
    Power_releaseConstraint(PowerCC26XX_SB_DISALLOW);
  }else{
    //禁止进入待机状态
    if(isStandbyEnabled == 0){
      return;
    }
    isStandbyEnabled = 0;
    Power_setConstraint(PowerCC26XX_SB_DISALLOW);
  }
}

//更改ADC频率-接口(CCC1)
uint8_t emgFreq = DEFAULT_EMG_FREQ;
void setEMGFreq(uint8_t neoFreq){
  emgFreq = neoFreq;
  //不必担心除以0的问题。emgFreq这个参数在该方法被调用时已经事先被处理好
  rtcTickPeriod = 0x00010000 / emgFreq;
  //设定完频率后，关闭EMG再重开
  emgPauseToSetting();
}
uint8_t getEMGFreq(void){
  return emgFreq;
}

//更改MPL频率-接口(FFF1)
uint8_t mplFreq = DEFAULT_MOTION_FREQ;
void setMPLFreq(uint8_t neoFreq){
  mplFreq = neoFreq;
  //不必担心除以0的问题。mplFreq这个参数在该方法被调用时已经事先被处理好
  setMotionRate(mplFreq);
}
uint8_t getMPLFreq(void){
  return mplFreq;
}

//一次性更改余下所有的系统设定(EEE1)

//个别系统设定的写入和读取========================================================

//是否打开EMG-接口
uint8_t emgIsOpen = 0;
void setEMGOpen(uint8_t neoOpen){
  if(emgIsOpen != neoOpen){
    if(neoOpen > 0){
      //打开EMG
      //打开EMG对应的三极管GPIO
    }else{
      //关闭EMG
      //关闭EMG对应的三极管GPIO
    }
  }
  emgIsOpen = neoOpen;
}
uint8_t getEMGOpen(void){
  return emgIsOpen;
}

//肌电是否平滑处理-接口
uint8_t emgSmooth = 0;
void setEMGSmooth(uint8_t neoSmo){
  emgSmooth = neoSmo;
}
uint8_t getEMGSmooth(void){
  return emgSmooth;
}

//肌电平滑处理力度-接口
uint8_t emgSmoPow = 0;
void setEmgSmoPow(uint8_t neoSPo){
  emgSmoPow = neoSPo;
}
uint8_t getEmgSmoPow(void){
  //直接获取平滑力度的原数值，这是为了直接读取系统设定
  return emgSmoPow;
}
uint8_t getEmgSmoPowByCalc(void){
  //获取平滑力度换算后的数值，这是为了程序中作计算使用
  uint8_t emgSmoCalc = emgSmoPow * 2 + 4;
  return emgSmoCalc;
}

//是否打开MOTION-接口
uint8_t motionIsOpen = 0;
void setMotionOpen(uint8_t neoMotion){
  if(motionIsOpen != neoMotion){
    if(neoMotion > 0){
      //打开Motion
      mpuWakeUp();
    }else{
      //关闭Motion
      mpuHibernate();
    }
  }
  motionIsOpen = neoMotion;
}
uint8_t getMotionOpen(void){
  return motionIsOpen;
}

//MPL输出何种格式数据-接口
uint8_t mplDataFormat = DEFAULT_OUTPUT_FORMAT; //初始状态下为欧拉角
void setMPLDFormat(uint8_t neoFmt){
  mplDataFormat = neoFmt;
  setHALReport(mplDataFormat);
}
uint8_t getMPLDFormat(void){
  return mplDataFormat;
}

//唤醒MPU的力度设定
uint8_t mpuWakePower = MPU_WAKE_POWER_DEFAULT; //初始状态下为1.0g唤醒
void setMPUWakePower(uint8_t neoPow){
  mpuWakePower = neoPow;
}
uint8_t getMPUWakePower(void){
  return mpuWakePower;
}
uint16_t getMPUWakePowerByCalc(void){
  uint16_t mpuWakePowerCalc = 4;
  mpuWakePowerCalc = mpuWakePowerCalc * mpuWakePower;
  return mpuWakePowerCalc;
}

//================================================================================

//写入系统设定====================================================================
void setSyetemOption(uint16_t sysOption){
  //MPU唤醒力度多大？
  uint8_t _mpuWakePower = sysOption >> 8;
  setMPUWakePower(_mpuWakePower);
  uint8_t _systemOptions = sysOption % 256;
  
  //MPU打开？
  uint8_t _mpuIsOpen = _systemOptions >> 7;
  setMotionOpen(_mpuIsOpen);
  _systemOptions = sysOption % 128;
  
  //MPL数据格式？
  uint8_t _mplDataFormat = _systemOptions >> 4;
  setMPLDFormat(_mplDataFormat);
  _systemOptions = sysOption % 16;
  
  //EMG平滑力度？
  uint8_t _emgSmoPow = _systemOptions >> 2;
  setEmgSmoPow(_emgSmoPow);
  _systemOptions = sysOption % 4;
  
  //EMG是否平滑？
  uint8_t _emgSmooth = _systemOptions >> 1;
  setEMGSmooth(_emgSmooth);
  _systemOptions = sysOption % 2;
  
  //EMG是否打开？
  setEMGOpen(_systemOptions);
}
//================================================================================

//获取总的系统设定================================================================
uint16_t getSystemOption(void){
  uint16_t allOptions = 0;
  
  //MPU唤醒力度多大？
  allOptions += mpuWakePower;
  
  //MPU打开？
  allOptions <<= 1;
  allOptions += motionIsOpen;
  
  //MPL数据格式？
  allOptions <<= 3;
  allOptions += mplDataFormat;
  
  //EMG平滑力度？
  allOptions <<= 2;
  allOptions += emgSmoPow;
  
  //EMG是否平滑？
  allOptions <<= 1;
  allOptions += emgSmooth;
  
  //EMG是否打开？
  allOptions <<= 1;
  allOptions += emgIsOpen;
  
  return allOptions;
}
//================================================================================

/*******************************************************************************
 */
