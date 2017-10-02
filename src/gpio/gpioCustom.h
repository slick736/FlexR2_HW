extern void PIN_Initialize(void);
extern uint32_t currentOutputVal;

extern void emgPortOpen(uint8_t gOpen);

//实验性开灭灯，正式产品中没有这些接口

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
