#include <Board.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/GPIO.h>
#include <ti/sysbios/knl/Clock.h>
#include "util.h"
#include "gpioCustom.h"
#include "simple_peripheral.h"
#include "main.h"
#include "i2cRW.h"

PIN_State ledPinState;
PIN_State buttonPinState;

PIN_Handle hLedPin;
PIN_Handle buttonPinHandle;

//哪个管脚响应2级工况唤醒（成品应为DIO15）
#define MPU_DIO         Board_DIO15
//#define MPU_DIO         Board_PIN_BUTTON1

#define CHARGE_DIO      Board_DIO22
#define EMG_DIO         Board_DIO0

static Clock_Struct blinkClock; // <-- 频闪时钟
static void clockBlinkJudge(UArg arg);
uint8_t blickProcessing; // <-- 是否正在闪烁中，0为熄灭或常亮，1为闪烁点亮阶段，2为闪烁熄灭阶段
uint8_t lightIndex; // <-- 哪个灯亮（或闪烁），0为熄灭，1为红灯，2为白灯
void pinDark(uint8_t withClockStop);

uint32_t currentOutputVal;
uint8_t runWithOuterADC = 0;

// 控制亮灯频率的变量
uint8_t blinkTimerCount = 1;

void buttonCallbackFxn(PIN_Handle handle, PIN_Id pinId);

void PIN_Initialize(void){
  //对GPIO进行初始化操作，包括控制LED开闭的时钟
  blickProcessing = 0;
  lightIndex = 0;
  Util_constructClock(&blinkClock, clockBlinkJudge,
                      BATTERY_BLINK_INTERVAL, 0, false, NULL);
  
  //参数意义：哪个管脚，是否允许读写，初始化是高/低电平状态，推挽输出还是开漏输出
  PIN_Config pLedPinTable[] = {
    Board_PIN_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MIN,
    Board_PIN_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MIN,
    EMG_DIO    | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MIN,
    
    PIN_TERMINATE
  };
  //最后一个参数疑似为按钮的下降沿触发中断
  PIN_Config buttonPinTable[] = {
    //Board_PIN_BUTTON0 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
    //Board_PIN_BUTTON1 | PIN_INPUT_EN | PIN_NOPULL | PIN_IRQ_NEGEDGE,
    CHARGE_DIO | PIN_INPUT_EN | PIN_NOPULL | PIN_IRQ_BOTHEDGES,
    MPU_DIO | PIN_INPUT_EN | PIN_NOPULL | PIN_IRQ_NEGEDGE,
    PIN_TERMINATE
  };
  
  hLedPin = PIN_open(&ledPinState, pLedPinTable);
  buttonPinHandle = PIN_open(&buttonPinState, buttonPinTable);
  PIN_registerIntCb(buttonPinHandle, &buttonCallbackFxn);
  
  //允许唤醒
  //PIN_setConfig(hLedPin, PINCC26XX_BM_WAKEUP, Board_BUTTON0 | PINCC26XX_WAKEUP_NEGEDGE);
  //PIN_setConfig(hLedPin, PINCC26XX_BM_WAKEUP, Board_BUTTON1 | PINCC26XX_WAKEUP_NEGEDGE);
}

void PIN_GLight(void){
  PIN_RDark();
  PIN_setOutputValue(hLedPin, Board_PIN_LED0, 1);
}
void PIN_GDark(void){
  PIN_setOutputValue(hLedPin, Board_PIN_LED0, 0);
}
void PIN_RLight(void){
  PIN_GDark();
  PIN_setOutputValue(hLedPin, Board_PIN_LED1, 1);
}
void PIN_RDark(void){
  PIN_setOutputValue(hLedPin, Board_PIN_LED1, 0);
}

//指令：白灯常亮
void pinGLightConst(void){
  pinDark(1);
  lightIndex = 2;
  PIN_GLight();
}
//指令：红灯常亮
void pinRLightConst(void){
  pinDark(1);
  lightIndex = 1;
  PIN_RLight();
}
//指令：白灯低频闪
void pinGLightBlink(void){
  if(blickProcessing <= 0){
    //没在闪，开启
    Util_startClock(&blinkClock);
    blickProcessing = 1;
    
  }else if(lightIndex == 1){
    //在闪，是否切换
    pinDark(0);
  }
  lightIndex = 2;
  if(blickProcessing < 2){
    PIN_GLight();
  }
}
//指令：红灯低频闪
void pinRLightBlink(void){
  if(blickProcessing <= 0){
    //没在闪，开启
    Util_startClock(&blinkClock);
    blickProcessing = 1;
    
  }else if(lightIndex == 2){
    //在闪，是否切换
    pinDark(0);
  }
  lightIndex = 1;
  if(blickProcessing < 2){
    PIN_RLight();
  }
}
//指令：灭灯（参数：是否同时停止闪光定时器 0-不停 1-停止）
void pinDark(uint8_t withClockStop){
  if(withClockStop > 0){
    Util_stopClock(&blinkClock);
    blickProcessing = 0;
    lightIndex = 0;
  }
  PIN_RDark();
  PIN_GDark();
}

//指令：2级工况刚刚进入时，Reset灯光计数器（刚刚进入2级工况时要点亮2秒）
extern void resetLampCount(void){
  blinkTimerCount = BATTERY_BLINK_RESET;
  blickProcessing = 1;
  //进入闪烁点亮阶段
  if(lightIndex == 1){
    PIN_RLight();
  }else{
    PIN_GLight();
  }
  Util_startClock(&blinkClock);
}
//指令：自动的闪光判定
static void clockBlinkJudge(UArg arg){
  if(blickProcessing == 1){
    //先判断闪光计数是否到达
    if(blinkTimerCount > 0){
      blinkTimerCount -= 1;
      if(blinkTimerCount > 0){
        Util_startClock(&blinkClock);
        return;
      }
    }
    blickProcessing = 2;
    //进入闪烁熄灭阶段
    if(lightIndex == 1){
      blinkTimerCount = BATTERY_LOWBATT_OFF;
    }else{
      blinkTimerCount = BATTERY_BLINK_OFF;
    }
    pinDark(0);
  }else{
    //先判断闪光计数是否到达
    if(blinkTimerCount > 0){
      blinkTimerCount -= 1;
      if(blinkTimerCount > 0){
        Util_startClock(&blinkClock);
        return;
      }
    }
    blickProcessing = 1;
    //进入闪烁点亮阶段
    if(lightIndex == 1){
      blinkTimerCount = BATTERY_LOWBATT_ON;
      PIN_RLight();
    }else{
      blinkTimerCount = BATTERY_BLINK_ON;
      PIN_GLight();
    }
  }
  Util_startClock(&blinkClock);
}
//判别是否正处于充电状态
uint8_t dioCharging = 0;
uint8_t isCharging(void){
  if(CHARGE_STATUS_KEEP > 0){
    return (CHARGE_STATUS_KEEP - 1);
  }
  if(CHARGE_BLINK_TEST){
    return 1;
  }
  if(CHARGE_LAMP){
    return dioCharging;
  }else{
    return 0;
  }
}
uint16_t batteryData;
//充电时判定充电完毕
uint8_t isChargeCompleted(void){
  //根据DIO-27脚数值判定充电完毕
  batteryData = getBatteryData();
  if(batteryData >= CHARGE_OK_VOLTAGE){
    return 1;
  }else{
    return 0;
  }
}
//非充电时判定电量低下
uint8_t isBatteryLow(void){
  if(isCharging() <= 0){
    //根据DIO-27脚数值判定电量低下
    batteryData = getBatteryData();
    if(batteryData <= BATTERY_LOW_VOLTAGE){
      return 1;
    }else{
      return 0;
    }
  }else{
    //充电状态就返回电量足够
    return 0;
  }
}

//控制ADC通路
uint8_t gpioIsOn = 0;
void emgPortOpen(uint8_t gOpen){
  gpioIsOn = gOpen;
  if(gpioIsOn > 0){
    //执行开操作
    PIN_setOutputValue(hLedPin, EMG_DIO, 1);
  }else{
    //执行关操作
    PIN_setOutputValue(hLedPin, EMG_DIO, 0);
  }
}

/*
 *  ======== buttonCallbackFxn ========
 *  Callback function for the GPIO interrupt.
 */
void buttonCallbackFxn(PIN_Handle handle, PIN_Id pinId){
  //uint32_t currVal = 0;
  //GPIO 中断处理（优先处理DIO22：处理是否正在充电的管脚）
  if(pinId == CHARGE_DIO){
    if(PIN_getInputValue(pinId)){
      //电平高，变成在充电状态
      dioCharging = 1;
    }else{
      //电平低，变成非充电状态
      dioCharging = 0;
    }
  }
  if(!PIN_getInputValue(pinId)){
    /* Toggle LED based on the button pressed */
    switch (pinId) {
    case MPU_DIO:
    //case Board_PIN_BUTTON1:
      //接收到了中断信号（暂时先用红板右侧按钮进行测试）
      //不同工况下，接收到的中断信号意义不同
      switch(getSYSTEMWorkLevel()){
      case SYSTEM_ENERGY_LEVEL0:
      case SYSTEM_ENERGY_LEVEL1:
      case SYSTEM_ENERGY_LEVEL1_LPA:
        //工况1以下：收到敲击信号，同意连接请求
        enableConnectBLE = CONNECT_RETRY_MAX;
        break;
      default:
        //2级以上工况：收到中断信号均为DMP一次计算完成
        break;
      }
      break;
    default:
      // Do nothing
      break;
    }
  }
}
