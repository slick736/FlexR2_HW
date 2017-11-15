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

//�ĸ��ܽ���Ӧ2���������ѣ���ƷӦΪDIO15��
#define MPU_DIO         Board_DIO15
//#define MPU_DIO         Board_PIN_BUTTON1

#define CHARGE_DIO      Board_DIO22
#define EMG_DIO         Board_DIO0

static Clock_Struct blinkClock; // <-- Ƶ��ʱ��
static void clockBlinkJudge(UArg arg);
uint8_t blickProcessing; // <-- �Ƿ�������˸�У�0ΪϨ�������1Ϊ��˸�����׶Σ�2Ϊ��˸Ϩ��׶�
uint8_t lightIndex; // <-- �ĸ�����������˸����0ΪϨ��1Ϊ��ƣ�2Ϊ�׵�
void pinDark(uint8_t withClockStop);

uint32_t currentOutputVal;
uint8_t runWithOuterADC = 0;

// ��������Ƶ�ʵı���
uint8_t blinkTimerCount = 1;

void buttonCallbackFxn(PIN_Handle handle, PIN_Id pinId);

void PIN_Initialize(void){
  //��GPIO���г�ʼ����������������LED���յ�ʱ��
  blickProcessing = 0;
  lightIndex = 0;
  Util_constructClock(&blinkClock, clockBlinkJudge,
                      BATTERY_BLINK_INTERVAL, 0, false, NULL);
  
  //�������壺�ĸ��ܽţ��Ƿ������д����ʼ���Ǹ�/�͵�ƽ״̬������������ǿ�©���
  PIN_Config pLedPinTable[] = {
    Board_PIN_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MIN,
    Board_PIN_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MIN,
    EMG_DIO    | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MIN,
    
    PIN_TERMINATE
  };
  //���һ����������Ϊ��ť���½��ش����ж�
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
  
  //������
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

//ָ��׵Ƴ���
void pinGLightConst(void){
  pinDark(1);
  lightIndex = 2;
  PIN_GLight();
}
//ָ���Ƴ���
void pinRLightConst(void){
  pinDark(1);
  lightIndex = 1;
  PIN_RLight();
}
//ָ��׵Ƶ�Ƶ��
void pinGLightBlink(void){
  if(blickProcessing <= 0){
    //û����������
    Util_startClock(&blinkClock);
    blickProcessing = 1;
    
  }else if(lightIndex == 1){
    //�������Ƿ��л�
    pinDark(0);
  }
  lightIndex = 2;
  if(blickProcessing < 2){
    PIN_GLight();
  }
}
//ָ���Ƶ�Ƶ��
void pinRLightBlink(void){
  if(blickProcessing <= 0){
    //û����������
    Util_startClock(&blinkClock);
    blickProcessing = 1;
    
  }else if(lightIndex == 2){
    //�������Ƿ��л�
    pinDark(0);
  }
  lightIndex = 1;
  if(blickProcessing < 2){
    PIN_RLight();
  }
}
//ָ���ƣ��������Ƿ�ͬʱֹͣ���ⶨʱ�� 0-��ͣ 1-ֹͣ��
void pinDark(uint8_t withClockStop){
  if(withClockStop > 0){
    Util_stopClock(&blinkClock);
    blickProcessing = 0;
    lightIndex = 0;
  }
  PIN_RDark();
  PIN_GDark();
}

//ָ�2�������ոս���ʱ��Reset�ƹ���������ոս���2������ʱҪ����2�룩
extern void resetLampCount(void){
  blinkTimerCount = BATTERY_BLINK_RESET;
  blickProcessing = 1;
  //������˸�����׶�
  if(lightIndex == 1){
    PIN_RLight();
  }else{
    PIN_GLight();
  }
  Util_startClock(&blinkClock);
}
//ָ��Զ��������ж�
static void clockBlinkJudge(UArg arg){
  if(blickProcessing == 1){
    //���ж���������Ƿ񵽴�
    if(blinkTimerCount > 0){
      blinkTimerCount -= 1;
      if(blinkTimerCount > 0){
        Util_startClock(&blinkClock);
        return;
      }
    }
    blickProcessing = 2;
    //������˸Ϩ��׶�
    if(lightIndex == 1){
      blinkTimerCount = BATTERY_LOWBATT_OFF;
    }else{
      blinkTimerCount = BATTERY_BLINK_OFF;
    }
    pinDark(0);
  }else{
    //���ж���������Ƿ񵽴�
    if(blinkTimerCount > 0){
      blinkTimerCount -= 1;
      if(blinkTimerCount > 0){
        Util_startClock(&blinkClock);
        return;
      }
    }
    blickProcessing = 1;
    //������˸�����׶�
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
//�б��Ƿ������ڳ��״̬
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
//���ʱ�ж�������
uint8_t isChargeCompleted(void){
  //����DIO-27����ֵ�ж�������
  batteryData = getBatteryData();
  if(batteryData >= CHARGE_OK_VOLTAGE){
    return 1;
  }else{
    return 0;
  }
}
//�ǳ��ʱ�ж���������
uint8_t isBatteryLow(void){
  if(isCharging() <= 0){
    //����DIO-27����ֵ�ж���������
    batteryData = getBatteryData();
    if(batteryData <= BATTERY_LOW_VOLTAGE){
      return 1;
    }else{
      return 0;
    }
  }else{
    //���״̬�ͷ��ص����㹻
    return 0;
  }
}

//����ADCͨ·
uint8_t gpioIsOn = 0;
void emgPortOpen(uint8_t gOpen){
  gpioIsOn = gOpen;
  if(gpioIsOn > 0){
    //ִ�п�����
    PIN_setOutputValue(hLedPin, EMG_DIO, 1);
  }else{
    //ִ�йز���
    PIN_setOutputValue(hLedPin, EMG_DIO, 0);
  }
}

/*
 *  ======== buttonCallbackFxn ========
 *  Callback function for the GPIO interrupt.
 */
void buttonCallbackFxn(PIN_Handle handle, PIN_Id pinId){
  //uint32_t currVal = 0;
  //GPIO �жϴ������ȴ���DIO22�������Ƿ����ڳ��Ĺܽţ�
  if(pinId == CHARGE_DIO){
    if(PIN_getInputValue(pinId)){
      //��ƽ�ߣ�����ڳ��״̬
      dioCharging = 1;
    }else{
      //��ƽ�ͣ���ɷǳ��״̬
      dioCharging = 0;
    }
  }
  if(!PIN_getInputValue(pinId)){
    /* Toggle LED based on the button pressed */
    switch (pinId) {
    case MPU_DIO:
    //case Board_PIN_BUTTON1:
      //���յ����ж��źţ���ʱ���ú���Ҳఴť���в��ԣ�
      //��ͬ�����£����յ����ж��ź����岻ͬ
      switch(getSYSTEMWorkLevel()){
      case SYSTEM_ENERGY_LEVEL0:
      case SYSTEM_ENERGY_LEVEL1:
      case SYSTEM_ENERGY_LEVEL1_LPA:
        //����1���£��յ��û��źţ�ͬ����������
        enableConnectBLE = CONNECT_RETRY_MAX;
        break;
      default:
        //2�����Ϲ������յ��ж��źž�ΪDMPһ�μ������
        break;
      }
      break;
    default:
      // Do nothing
      break;
    }
  }
}
