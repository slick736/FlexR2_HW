extern void PIN_Initialize(void);
extern uint32_t currentOutputVal;

extern void emgPortOpen(uint8_t gOpen);

//ʵ���Կ���ƣ���ʽ��Ʒ��û����Щ�ӿ�

extern void PIN_GLight(void);
extern void PIN_RLight(void);
extern void PIN_GDark(void);
extern void PIN_RDark(void);

extern void pinGLightConst(void);
extern void pinRLightConst(void);
extern void pinGLightBlink(void);
extern void pinRLightBlink(void);
extern void resetLampCount(void);
extern void pinDark(uint8_t withClockStop);
extern uint8_t isChargeCompleted(void);
extern uint8_t isBatteryLow(void);
extern uint8_t isCharging(void);
