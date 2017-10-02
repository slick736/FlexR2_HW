#include "delay.h"
#include "ex_include_tirtos.h"

//时间延迟（单位毫秒）
//注意：这个方法一定要在一个“确保所有Task线程都已经建立好”
//的位置上调用，否则可能会导致IdleTask被截停，从而引起MCU死机
void mdelay(unsigned long milliseconds)
{
  //5000000 = 5s
  Task_sleep(milliseconds * 1000 / Clock_tickPeriod);
}

//获取时间戳（地址方式输入）。意义为本系统从开机开始一直到现在一共运行了多少毫秒
int get_tick_count(unsigned long *count)
{
  //注意：获取系统的毫秒数，要让ClockTick除以100（因为CC2640R2F是10微秒一个Tick）
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