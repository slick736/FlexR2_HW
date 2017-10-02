#define SYSTEM_ENERGY_LEVEL0         0
#define SYSTEM_ENERGY_LEVEL1         10
#define SYSTEM_ENERGY_LEVEL1_LPA     15
#define SYSTEM_ENERGY_LEVEL2         20
#define SYSTEM_ENERGY_LEVEL3         30

#define LOW_RX                  0
#define HIGH_RX                 1
#define LOW_TX                  0
#define HIGH_TX                 1

#define EMG_OFF                 0
#define EMG_ON                  1

#define ADC_SMOOTH_FORCE_MAX    10

#define EMG_FREQ_MIN            1
#define EMG_FREQ_MAX            60

#define MOTION_OFF              0
#define MOTION_ON               0x80

#define MPU_WAKE_POWER_DEFAULT  250

#define MPL_FREQ_MIN            10
#define MPL_FREQ_MAX            50

#define ENABLE_AUTO_SLEEP       1 // <-- 是否允许以Power_releaseConstraint方式

#define OUTPUT_BYTE_OFFSET      2 // <--蓝牙数据输出地址偏移量

// 9-Axis Fusion Mode (0-False 1-True)
#define AXIS_9_FUSION_MODE      1

//开机时默认数值的设置
#define DEFAULT_EMG_FREQ        20
#define DEFAULT_MOTION_FREQ     20
#define DEFAULT_OUTPUT_FORMAT   3
#define EMG_RESET_INTERVAL      500

//每多少个蓝牙周期才进行一次运算和采样（成品应为0）
#define BLE_ACTIVE_INTERVAL     0

//是否屏蔽掉Motion运算环节（成品应为0）
#define DISABLE_MPL             0

//用于电源指示的管脚及常数
#define BATTERY_BLINK_INTERVAL      100  // <-- 电源指示灯闪光间隔（毫秒）
#define BATTERY_BLINK_ON            10
#define BATTERY_BLINK_OFF           20
#define BATTERY_BLINK_RESET         20
#define BATTERY_LOWBATT_ON          1
#define BATTERY_LOWBATT_OFF         5
#define CHARGE_OK_VOLTAGE           3876 // <-- 充电完成门限值（4.07V）
#define BATTERY_LOW_VOLTAGE         3495 // <-- 电量低下门限值（3.67V）
#define SHUTDOWN_LOW_VOLTAGE        3428 // <-- 禁止开机门限值（3.60V）
#define BATTERY_SAMPLE_INTERVAL     20   // <-- 每过多少个EMG周期自动采样一次电量

//连接相关
#define CONNECT_RETRY_MAX           2 // <-- 拍击后重试多少个0<->1循环周期
#define PRECONNECT_RETRY_MAX        5 // <-- 拍击前让LED点亮多少个0<->1循环周期
#define REJECT_CONNECT_ON_LOWBATT   1 // <-- 是否在电池没电时自动挂断并拒绝再次连接（成品应为1）

//测试用灯光（成品应为0）
#define LAMP_TEST_MODE              0

//测试：蓝牙实时修改发射功率（成品应为0）
#define BLE_VARI_MODE               0

//测试：是否响应充电接口的灯光行为改变（成品应为1）
#define CHARGE_LAMP                 1
//测试：充电时，红灯白灯是否应该闪烁，0为长亮（成品应为0）
#define CHARGE_BLINK_AT_CHARGE      0
//测试：持续使用充电模式灯光（成品应为0）（优先级高于CHARGE_LAMP）
#define CHARGE_BLINK_TEST           0
//测试：是否永久允许不经陀螺仪同意而直接连通（成品应为0）
#define WORKLEVEL2_WITHOUT_MPU      0

//被外因挂断后，允许尝试不经陀螺仪同意而自动连接的次数（成品应为10）
#define CONNECT_WITHOUT_MPU         10

//设备名称
#define DEVICE_NAME                 "FlexR II"

extern void setSYSTEMWorkLevel(uint8_t workLevel);
extern uint8_t getSYSTEMWorkLevel(void);
extern void enableStandby(uint8_t standbyEnabled);
extern void setLEDWorkBlink(uint8_t workLevel);

extern int initMPLService(void);
extern int initMPLService_STEP1(void);
extern int initMPLService_STEP1B(void);
extern int initMPLService_STEP1_TEST(void);
extern int initMPLService_STEP2(void);
extern int initMPLService_STEP3(void);
extern int initMPLService_STEP3B(void);
extern int initMPLService_STEP4(void);
extern int initMPLService_STEP5(void);

//通过蓝牙操作的系统设置接口
extern void setEMGFreq(uint8_t neoFreq);
extern uint8_t getEMGFreq(void);
extern void emgPauseToSetting(void);

extern void setMPLFreq(uint8_t neoFreq);
extern uint8_t getMPLFreq(void);

extern void setSyetemOption(uint16_t sysOption);
extern uint16_t getSystemOption(void);

//设置与获取个别系统设定的接口（蓝牙不操作此处，但程序内部会用到）
extern void setEMGOpen(uint8_t neoOpen);
extern uint8_t getEMGOpen(void);

extern void setEMGSmooth(uint8_t neoSmo);
extern uint8_t getEMGSmooth(void);
extern uint8_t getEmgSmoPowByCalc(void);

extern void setEmgSmoPow(uint8_t neoSPo);
extern uint8_t getEmgSmoPow(void);

extern void setMotionOpen(uint8_t neoMotion);
extern uint8_t getMotionOpen(void);

extern void setMPLDFormat(uint8_t neoFmt);
extern uint8_t getMPLDFormat(void);

extern void setMPUWakePower(uint8_t neoPow);
extern uint8_t getMPUWakePower(void);
extern uint16_t getMPUWakePowerByCalc(void);

//获取及设置AD转换的数据
extern uint16_t getBatteryData(void);
extern uint16_t getADCData(void);
extern void setSmoothedEMGData(uint16_t smoothedData);
extern uint16_t getSmoothedEMGData(void);
