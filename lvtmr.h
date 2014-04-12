/*
 * This file is part of Current Monitor.
 *
 * web site:    http://github.com/nradulovic
 * e-mail  :    nenad.b.radulovic@gmail.com
 *//***********************************************************************//**
 * @file
 * @author      Nenad Radulovic
 *********************************************************************//** @{ */

#ifndef LVTMR_H__
#define	LVTMR_H__

/*=========================================================  INCLUDE FILES  ==*/

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

/*===============================================================  MACRO's  ==*/

#define LVTMR_MAX_NUM_OF_TIMERS         4

#define LVTMR_MS_TO_TICK(ms)                                                    \
  (((uint32_t)(ms) * (uint32_t)LVTMR_TICK_FREQUENCY) / (uint32_t)1000u)

/*------------------------------------------------------  C++ extern begin  --*/
#ifdef __cplusplus
extern "C" {
#endif

/*============================================================  DATA TYPES  ==*/

typedef uint8_t lvTmr_T;
typedef uint16_t lvTmrTick_T;

/*======================================================  GLOBAL VARIABLES  ==*/
/*===================================================  FUNCTION PROTOTYPES  ==*/

void lvTmrInit(
  void);

lvTmr_T lvTmrCreateI(
  lvTmrTick_T tick);

bool lvTmrIsDoneI(
  lvTmr_T tmr);

void lvTmrDeleteI(
  lvTmr_T tmr);

void lvTmrEvalI(
  void);

/*--------------------------------------------------------  C++ extern end  --*/
#ifdef __cplusplus
}
#endif

/*================================*//** @cond *//*==  CONFIGURATION ERRORS  ==*/
/** @endcond *//** @} *//******************************************************
 * END of lvtmr.h
 ******************************************************************************/
#endif /* LVTMR_H__ */

