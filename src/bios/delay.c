#include "delay.h"
#include "ex_include_tirtos.h"

//ʱ���ӳ٣���λ���룩
//ע�⣺�������һ��Ҫ��һ����ȷ������Task�̶߳��Ѿ������á�
//��λ���ϵ��ã�������ܻᵼ��IdleTask����ͣ���Ӷ�����MCU����
void mdelay(unsigned long milliseconds)
{
  //5000000 = 5s
  Task_sleep(milliseconds * 1000 / Clock_tickPeriod);
}

//��ȡʱ�������ַ��ʽ���룩������Ϊ��ϵͳ�ӿ�����ʼһֱ������һ�������˶��ٺ���
int get_tick_count(unsigned long *count)
{
  //ע�⣺��ȡϵͳ�ĺ�������Ҫ��ClockTick����100����ΪCC2640R2F��10΢��һ��Tick��
  count[0] = Clock_getTicks() / 100;
  return 0;
}

/*
int get_tick_count(unsigned long *count)
{
        count[0] = g_ul_ms_ticks;
	return 0;
}
*/