/*
 * This file is part of Current Monitor.
 *
 * web site:    http://github.com/nradulovic
 * e-mail  :    nenad.b.radulovic@gmail.com
 *//***********************************************************************//**
 * @file
 * @author      Nenad Radulovic
 *********************************************************************//** @{ */

#ifndef STATUS_H__
#define	STATUS_H__

/*=========================================================  INCLUDE FILES  ==*/

#include <stdint.h>

/*===============================================================  MACRO's  ==*/

#define STAT_SUCCESS                    ((uint8_t)0)
#define STAT_FAIL                       ((uint8_t)-1)

#define STAT_OBJ_OSC                    (0x1 << 0)

#define STAT_SET_SUCCESS(object)                                                \
  do                                                                            \
  {                                                                             \
    DeviceStatus &= ~(object);                                                  \
  }                                                                             \
  while (0)

#define STAT_SET_FAIL(object)                                                   \
  do                                                                            \
  {                                                                             \
    DeviceStatus |= (object);                                                   \
  }                                                                             \
  while (0)

/*------------------------------------------------------  C++ extern begin  --*/
#ifdef __cplusplus
extern "C" {
#endif

/*============================================================  DATA TYPES  ==*/
/*======================================================  GLOBAL VARIABLES  ==*/

extern uint8_t DeviceStatus;

/*===================================================  FUNCTION PROTOTYPES  ==*/
/*--------------------------------------------------------  C++ extern end  --*/
#ifdef __cplusplus
}
#endif

/*================================*//** @cond *//*==  CONFIGURATION ERRORS  ==*/
/** @endcond *//** @} *//******************************************************
 * END of status.h
 ******************************************************************************/
#endif /* STATUS_H__ */

