
#include "lvtmr.h"
#include "status.h"

#define TMR_IS_FREE             (lvTmrTick_T)-1

static lvTmrTick_T LvTmr[LVTMR_MAX_NUM_OF_TIMERS];

void lvTmrInit(
  void)
{
  lvTmr_T               cnt;

  cnt = LVTMR_MAX_NUM_OF_TIMERS;

  while (cnt != 0u)
  {
    cnt--;
    LvTmr[cnt] = TMR_IS_FREE;
  }
}

lvTmr_T lvTmrCreateI(
  lvTmrTick_T           tick)
{
  lvTmr_T               cnt;

  cnt = LVTMR_MAX_NUM_OF_TIMERS;

  while (cnt != 0u)
  {
    cnt--;

    if (LvTmr[cnt] == TMR_IS_FREE)
    {
      LvTmr[cnt] = tick;

      return (cnt);
    }
  }

  return ((lvTmr_T)STAT_FAIL);
}

bool lvTmrIsDoneI(
  lvTmr_T tmr)
{
  if (LvTmr[tmr] != 0)
  {
    return (false);
  }
  else
  {
    LvTmr[tmr] = TMR_IS_FREE;
    
    return (true);
  }
}

void lvTmrDeleteI(
  lvTmr_T tmr)
{
  LvTmr[tmr] = TMR_IS_FREE;
}

void lvTmrEvalI(
  void)
{
  lvTmr_T               cnt;

  cnt = LVTMR_MAX_NUM_OF_TIMERS;

  while (cnt != 0)
  {
    cnt--;
    
    if ((LvTmr[cnt] != 0) && (LvTmr[cnt] != TMR_IS_FREE))
    {
      LvTmr[cnt]--;
    }
  }
}
