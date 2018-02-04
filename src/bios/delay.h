#ifndef DELAY_H
#define DELAY_H

#ifdef __cplusplus
extern "C"
{
#endif
  
extern void mdelay(unsigned long milliseconds);
extern int get_tick_count(unsigned long *count);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif