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

#define ENABLE_AUTO_SLEEP       1 // <-- �Ƿ�������Power_releaseConstraint��ʽ

#define OUTPUT_BYTE_OFFSET      2 // <--�������������ַƫ����

// 9-Axis Fusion Mode (0-False 1-True)
#define AXIS_9_FUSION_MODE      1

//����ʱĬ����ֵ������
#define DEFAULT_EMG_FREQ        20
#define DEFAULT_MOTION_FREQ     20
#define DEFAULT_OUTPUT_FORMAT   3
#define EMG_RESET_INTERVAL      500

//ÿ���ٸ��������ڲŽ���һ������Ͳ�������ƷӦΪ0��
#define BLE_ACTIVE_INTERVAL     0

//�Ƿ����ε�Motion���㻷�ڣ���ƷӦΪ0��
#define DISABLE_MPL             0

//���ڵ�Դָʾ�Ĺܽż�����
#define BATTERY_BLINK_INTERVAL      100  // <-- ��Դָʾ�������������룩
#define BATTERY_BLINK_ON            10
#define BATTERY_BLINK_OFF           20
#define BATTERY_BLINK_RESET         20
#define BATTERY_LOWBATT_ON          1
#define BATTERY_LOWBATT_OFF         5
#define CHARGE_OK_VOLTAGE           3876 // <-- ����������ֵ��4.07V��
#define BATTERY_LOW_VOLTAGE         3495 // <-- ������������ֵ��3.67V��
#define SHUTDOWN_LOW_VOLTAGE        3428 // <-- ��ֹ��������ֵ��3.60V��
#define BATTERY_SAMPLE_INTERVAL     20   // <-- ÿ�����ٸ�EMG�����Զ�����һ�ε���

//�������
#define CONNECT_RETRY_MAX           2 // <-- �Ļ������Զ��ٸ�0<->1ѭ������
#define PRECONNECT_RETRY_MAX        5 // <-- �Ļ�ǰ��LED�������ٸ�0<->1ѭ������
#define REJECT_CONNECT_ON_LOWBATT   1 // <-- �Ƿ��ڵ��û��ʱ�Զ��Ҷϲ��ܾ��ٴ����ӣ���ƷӦΪ1��

//�����õƹ⣨��ƷӦΪ0��
#define LAMP_TEST_MODE              0

//���ԣ�����ʵʱ�޸ķ��书�ʣ���ƷӦΪ0��
#define BLE_VARI_MODE               0

//���ԣ��Ƿ���Ӧ���ӿڵĵƹ���Ϊ�ı䣨��ƷӦΪ1��
#define CHARGE_LAMP                 1
//���ԣ����ʱ����ư׵��Ƿ�Ӧ����˸��0Ϊ��������ƷӦΪ0��
#define CHARGE_BLINK_AT_CHARGE      0
//���ԣ�����ʹ�ó��ģʽ�ƹ⣨��ƷӦΪ0�������ȼ�����CHARGE_LAMP��
#define CHARGE_BLINK_TEST           0
//���ԣ��Ƿ�����������������ͬ���ֱ����ͨ����ƷӦΪ0��
#define WORKLEVEL2_WITHOUT_MPU      0

//������ҶϺ������Բ���������ͬ����Զ����ӵĴ�������ƷӦΪ10��
#define CONNECT_WITHOUT_MPU         10

//�豸����
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

//ͨ������������ϵͳ���ýӿ�
extern void setEMGFreq(uint8_t neoFreq);
extern uint8_t getEMGFreq(void);
extern void emgPauseToSetting(void);

extern void setMPLFreq(uint8_t neoFreq);
extern uint8_t getMPLFreq(void);

extern void setSyetemOption(uint16_t sysOption);
extern uint16_t getSystemOption(void);

//�������ȡ����ϵͳ�趨�Ľӿڣ������������˴����������ڲ����õ���
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

//��ȡ������ADת��������
extern uint16_t getBatteryData(void);
extern uint16_t getADCData(void);
extern void setSmoothedEMGData(uint16_t smoothedData);
extern uint16_t getSmoothedEMGData(void);
