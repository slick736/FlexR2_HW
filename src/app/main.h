#define SYSTEM_ENERGY_LEVEL0                 0
#define SYSTEM_ENERGY_LEVEL1                 10
#define SYSTEM_ENERGY_LEVEL1_LPA             15
#define SYSTEM_ENERGY_LEVEL2_LPA             17
#define SYSTEM_ENERGY_LEVEL2                 20
#define SYSTEM_ENERGY_LEVEL3_EMG             30
#define SYSTEM_ENERGY_LEVEL3_MOTION          32

#define WORKING_STATUS_OK                    0
#define WORKING_STATUS_LOW_BATTERY           1
#define WORKING_STATUS_WAIT_FOR_TAP          8
#define WORKING_STATUS_DECLINE_CONNECTION    9
#define WORKING_STATUS_REQUEST_DISCONNECT    15

#define HEALEREMG_SERIAL_NUMBER    0x46, 0x52, 0x31, 0x38, 0x31, 0x32, 0x30, 0x32, 0x30, 0x31, 0x37, 0x30, 0x30, 0x30, 0x32, 0x32, 0x32

#define LOW_RX                  0
#define HIGH_RX                 1
#define LOW_TX                  0
#define HIGH_TX                 1

#define EMG_OFF                 0
#define EMG_ON                  1

#define ADC_SMOOTH_FORCE_MAX    10

//#define EMG_FREQ_MIN            1
//#define EMG_FREQ_MAX            60

#define MOTION_OFF              0
#define MOTION_ON               0x80

#define MPU_WAKE_POWER_DEFAULT  250

//#define MPL_FREQ_MIN            10
//#define MPL_FREQ_MAX            50

#define ENABLE_AUTO_SLEEP       1 // <-- 是否允许以Power_releaseConstraint方式

#define OUTPUT_BYTE_OFFSET      2 // <--蓝牙数据输出地址偏移量

// 9-Axis Fusion Mode (0-False 1-True)
#define AXIS_9_FUSION_MODE      0
#define AXIS_3_FUSION_MODE      1

//开机时默认数值的设置
#define DEFAULT_EMG_FREQ        1024 // <-- 原 20
#define DEFAULT_MOTION_FREQ     200  // <-- 原 20
#define DEFAULT_OUTPUT_FORMAT   5   // <-- 原 3，欧拉角
#define EMG_RESET_INTERVAL      500

#define EMG_SAMPLE_CYCLE_COUNT     40 // <-- 多少个EMG采样周期过后，才能发送一次数据

#define MOTION_INTERVAL            5  // <-- 间隔多少毫秒，取一次Motion数据（与MotionFreq对应）
#define MOTION_SAMPLE_CYCLE_COUNT  6  // <-- 多少个Motion采样周期过后，才能发送一次数据
#define MOTION_DATA_LENGTH         20 // <-- 每个Motion最大多少字节
#define MOTION_SAMPLE_WITH_BLE     0  // <-- 蓝牙发送循环之前是否同时伴随采样(0-不伴随 1-伴随)

#define IDLE_INTERVAL              50 // <-- 间隔多少毫秒执行一次Idle循环

#define BLE_BYTES_MAX              240 // <-- 每次蓝牙发射最大多少字节

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
//#define BATTERY_LOW_VOLTAGE         3637 // <-- 电量低下门限值（3.82V）
//#define SHUTDOWN_LOW_VOLTAGE        3590 // <-- 禁止开机门限值（3.77V）
#define BATTERY_SAMPLE_INTERVAL     20   // <-- 每过多少个EMG周期自动采样一次电量

//连接相关
#define CONNECT_RETRY_MAX           2 // <-- 拍击后重试多少个0<->1循环周期
#define PRECONNECT_RETRY_MAX        5 // <-- 拍击前让LED点亮多少个0<->1循环周期
#define REJECT_CONNECT_ON_LOWBATT   0 // <-- 是否在电池没电时请求挂断并拒绝再次连接（成品应为1）

#define ENABLE_HEARTBEAT            0 // <-- 启用心跳机制防死机（成品应为1）
#define ENABLE_AUTO_DISCONNECT_BY_HEARTBEAT 0 // <-- 心跳机制启用时允许硬件自主断开（成品应为1）
#define HEARTBEAT_MAX_INTERVAL      200 // <-- 经过多少个EMG周期没有从EEE2通道读取数据就认为断连
                                        // 20 = 1秒，成品应为100

#define VERSION_INFO                1 // <-- 版本信息（本产品为1）

#define FPS_STATUS_MODE             0 // <-- 改用FPS统计模式，取代之前的工作状态和电压输出（成品应为0）

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
//测试：永远保持充电/不充电状态，0-不设定 1-永远不充电 2-永远充电（成品应为0）
#define CHARGE_STATUS_KEEP          0

//测试：是否使用DIO-6（白灯）来测试采样周期（成品应为0）
#define DIO_6_SAMPLE_RATE           0
//测试：EMG在持续采样时，电位反复变化还是维持高电平（0-维持高电平 1-反复变化）
//若上一个数值为0，则该数值没意义
#define DIO_6_SAMPLE_VARI           0

//测试：是否让EMG输出假数据，假数据的变化幅度（成品应为0）
#define MIRAGE_EMG_MODE             0 // <-- 测试时设为 50

//被外因挂断后，允许尝试不经陀螺仪同意而自动连接的次数（成品应为10）
#define CONNECT_WITHOUT_MPU         10

//设备名称
#define DEVICE_NAME                 "FlexR II"

extern void setSYSTEMWorkLevel(uint8_t workLevel);
extern uint8_t getSYSTEMWorkLevel(void);
//extern void enableStandby(uint8_t standbyEnabled);
extern uint8_t setLEDWorkBlink(uint8_t workLevel);

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
//extern void setEMGOpen(uint8_t neoOpen);
extern uint8_t getEMGOpen(void);

//extern void setEMGSmooth(uint8_t neoSmo);
extern uint8_t getEMGSmooth(void);
extern uint8_t getEmgSmoPowByCalc(void);

//extern void setEmgSmoPow(uint8_t neoSPo);
extern uint8_t getEmgSmoPow(void);

//extern void setMotionOpen(uint8_t neoMotion);
extern uint8_t getMotionOpen(void);

//extern void setMPLDFormat(uint8_t neoFmt);
extern uint8_t getMPLDFormat(void);

//extern void setMPUWakePower(uint8_t neoPow);
extern uint8_t getMPUWakePower(void);
//extern uint16_t getMPUWakePowerByCalc(void);

//获取及设置AD转换的数据
extern uint16_t getBatteryData(void);
extern uint16_t getADCData(void);
extern void setSmoothedEMGData(uint16_t smoothedData);
extern uint16_t getSmoothedEMGData(void);
