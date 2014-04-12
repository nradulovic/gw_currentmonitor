/* 
 * File:   main.c
 * Author: Nenad Radulovic
 *
 * Created on 13.12.2013., 12.30
 */

/*=========================================================  INCLUDE FILES  ==*/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <xc.h>

#include "hw_config.h"
#include "config.h"
#include "status.h"
#include "lvtmr.h"

/*=========================================================  LOCAL MACRO's  ==*/

/**@brief       Calculate log2 for value @c x during the compilation
 */
#define BIT_UINT8_LOG2(x)                                                       \
    ((x) <   2u ? 0u :                                                          \
     ((x) <   4u ? 1u :                                                         \
      ((x) <   8u ? 2u :                                                        \
       ((x) <  16u ? 3u :                                                       \
        ((x) <  32u ? 4u :                                                      \
         ((x) <  64u ? 5u :                                                     \
          ((x) < 128u ? 6u : 7u)))))))

#define CURRENT_MA_TO_INT(current)                                              \
  ((uint16_t)(((uint32_t)(current) * (uint32_t)CFG_SENS_UV_PER_AMPER * (uint32_t)SAMPLES_PER_POINT) / ((uint32_t)CFG_ADC_UV_QUANTUM * (uint32_t)1000u)))

#define ABS16(x)                        (((int16_t)x < 0) ? (-(int16_t)x) : x)

#define RELAY_RISE_SET_ACTIVE()                                                 \
  do                                                                            \
  {                                                                             \
    RELAY_RISE_LAT |= RELAY_RISE_PIN_Msk;                                       \
  }                                                                             \
  while (0)

#define RELAY_RISE_SET_INACTIVE()                                               \
  do                                                                            \
  {                                                                             \
    RELAY_RISE_LAT &= ~RELAY_RISE_PIN_Msk;                                      \
  }                                                                             \
  while (0)

#define RELAY_DROP_SET_ACTIVE()                                                 \
  do                                                                            \
  {                                                                             \
    RELAY_DROP_LAT |= RELAY_DROP_PIN_Msk;                                       \
  }                                                                             \
  while (0)

#define RELAY_DROP_SET_INACTIVE()                                               \
  do                                                                            \
  {                                                                             \
    RELAY_DROP_LAT &= ~RELAY_DROP_PIN_Msk;                                      \
  }                                                                             \
  while (0)

#define LED_RISE_SET_ACTIVE()                                                   \
  do                                                                            \
  {                                                                             \
    LED_RISE_LAT &= ~LED_RISE_PIN_Msk;                                          \
  }                                                                             \
  while (0)

#define LED_RISE_SET_INACTIVE()                                                 \
  do                                                                            \
  {                                                                             \
    LED_RISE_LAT |= LED_RISE_PIN_Msk;                                           \
  }                                                                             \
  while (0)

#define LED_DROP_SET_ACTIVE()                                                   \
  do                                                                            \
  {                                                                             \
    LED_DROP_LAT &= ~LED_DROP_PIN_Msk;                                          \
  }                                                                             \
  while (0)

#define LED_DROP_SET_INACTIVE()                                                 \
  do                                                                            \
  {                                                                             \
    LED_DROP_LAT |= LED_DROP_PIN_Msk;                                           \
  }                                                                             \
  while (0)

#if !defined(NDEBUG)
#define DEBUG_PIN_SET_ACTIVE()                                                  \
  do                                                                            \
  {                                                                             \
    DEBUG_PIN_LAT |= DEBUG_PIN_PIN_Msk;                                         \
  }                                                                             \
  while (0)

#define DEBUG_PIN_SET_INACTIVE()                                                \
  do                                                                            \
  {                                                                             \
    DEBUG_PIN_LAT &= ~DEBUG_PIN_PIN_Msk;                                        \
  }                                                                             \
  while (0)
#else
#define DEBUG_PIN_SET_ACTIVE()                                                  \
  (void)0

#define DEBUG_PIN_SET_INACTIVE()                                                \
  (void)0
#endif

/*======================================================  LOCAL DATA TYPES  ==*/

enum sensState
{
  SENS_IDLE,
  SENS_STARTED,
  SENS_SEARCHING_RISE,
  SENS_RISE_FOUND,
  SENS_SEARCHING_DROP,
  SENS_DROP_FOUND
};

enum displayCmd
{
  LED_ON,
  LED_OFF,
  LED_BLINK
};

/*=============================================  LOCAL FUNCTION PROTOTYPES  ==*/

static void hwInit(
  void);

#if !defined(NDEBUG)
void debugPinInit(
  void);
#endif
static void oscInit(
  void);
static void relayDropInit(
  void);
static void relayRiseInit(
  void);
static void sensInit(
  void);
static void sensTriggerInit(
  void);
static void sensIsr(
  void);
void sensCompute(
  void);
static void sensProcess(
  void);
static void sysTickInit(
  void);
static void sysTickIsr(
  void);
static void displayInit(
  void);
static void displayStatus(
  void);
void ledDropFsm(
  enum displayCmd       cmd);
void ledRiseFsm(
  enum displayCmd       cmd);
void interrupt hiIsr(
  void);
void low_priority interrupt loIsr(
  void);

/*=======================================================  LOCAL VARIABLES  ==*/

static enum sensState SensState;
static uint16_t AbsSignal;

/*======================================================  GLOBAL VARIABLES  ==*/
/*============================================  LOCAL FUNCTION DEFINITIONS  ==*/

static void hwInit(
  void)
{
  INTCONbits.GIEH       = 0;                                                    /* Disable all high prio interrupts                         */
  INTCONbits.GIEL       = 0;                                                    /* Disable all low  prio interrupts                         */
  RCONbits.IPEN         = 1;                                                    /* Enable ISR priorities                                    */
  PIE1                  = 0;
  PIE2                  = 0;
  oscInit();
#if !defined(NDEBUG)
  debugPinInit();
#endif
  relayDropInit();
  relayRiseInit();
  displayInit();
  sensInit();
  sensTriggerInit();
  sysTickInit();
  INTCONbits.GIEH       = 1;                                                    /* Enable all high prio interrupts                          */
  INTCONbits.GIEL       = 1;                                                    /* Enable all low  prio interrupts                          */
}

/*-- Debug pin ---------------------------------------------------------------*/
#if !defined(NDEBUG)
void debugPinInit(
  void)
{
  DEBUG_PIN_LAT   &= ~DEBUG_PIN_PIN_Msk;
  DEBUG_PIN_PORT  &= ~DEBUG_PIN_PIN_Msk;
  DEBUG_PIN_TRIS  &= ~DEBUG_PIN_PIN_Msk;
}
#endif

/*-- Osc ---------------------------------------------------------------------*/
static void oscInit(
  void)
{
  OSCCONbits.IDLEN      = 0;                                                    /* Run mode enabled                                         */
#if (CPU_FREQUENCY == 8000000ul)
  {
    uint8_t             retry;

    OSCCONbits.IRCF     = 0x07;                                                 /* 8 MHz                                                    */
    retry = (uint8_t)-1;
    while ((OSCCONbits.IOFS == 0x0) && (retry-- != 0));

    if (OSCCONbits.IOFS == 1)
    {
      OSCCONbits.SCS0   = 0;                                                    /* Set to internal oscilator                                */
      OSCCONbits.SCS1   = 1;
    }
    else
    {
      STAT_SET_FAIL(STAT_OBJ_OSC);
    }
  }
#elif (CPU_FREQUENCY == 32000000ul)
  {
    uint8_t             retry;

    retry = (uint8_t)-1;
    while ((OSCCONbits.OSTS == 0x0) && (retry-- != 0));

    if (OSCCONbits.OSTS == 1)
    {
      OSCCONbits.SCS0   = 0;                                                    /* Set to external oscilator                                */
      OSCCONbits.SCS1   = 0;
    }
    else
    {
      STAT_SET_FAIL(STAT_OBJ_OSC);
    }
  }
#endif
}

/*-- Relay functions ---------------------------------------------------------*/
static void relayDropInit(
  void)
{
  RELAY_DROP_LAT  &= ~RELAY_DROP_PIN_Msk;
  RELAY_DROP_PORT &= ~RELAY_DROP_PIN_Msk;
  RELAY_DROP_TRIS &= ~RELAY_DROP_PIN_Msk;
}

static void relayRiseInit(
  void)
{
  RELAY_RISE_LAT  &= ~RELAY_RISE_PIN_Msk;
  RELAY_RISE_PORT &= ~RELAY_RISE_PIN_Msk;
  RELAY_RISE_TRIS &= ~RELAY_RISE_PIN_Msk;
}

/*-- Sensor functions --------------------------------------------------------*/

#define ADC_CLOCK_SOURCE_2Tosc          0x00u
#define ADC_CLOCK_SOURCE_4Tosc          0x04u
#define ADC_CLOCK_SOURCE_8Tosc          0x01u
#define ADC_CLOCK_SOURCE_16Tosc         0x05u
#define ADC_CLOCK_SOURCE_32Tosc         0x02u
#define ADC_CLOCK_SOURCE_64Tosc         0x06u
#define ADC_CLOCK_SOURCE_Frc            0x03u

#define ADC_ACQUISITION_0Tad            0x00u
#define ADC_ACQUISITION_2Tad            0x01u
#define ADC_ACQUISITION_4Tad            0x02u
#define ADC_ACQUISITION_6Tad            0x03u
#define ADC_ACQUISITION_8Tad            0x04u
#define ADC_ACQUISITION_12Tad           0x05u
#define ADC_ACQUISITION_16Tad           0x06u
#define ADC_ACQUISITION_20Tad           0x07u

static void sensInit(
  void)
{
  ADCON0bits.ADON       = 0;                                                    /* A/D converter module is off                              */
  SENS_LAT             &= ~SENS_PIN_Msk;
  SENS_PORT            &= ~SENS_PIN_Msk;
  SENS_TRIS            |=  SENS_PIN_Msk;
  ADCON2bits.ADCS       = ADC_CLOCK_SOURCE_64Tosc;                              /* A/D conversion clock selection                           */
  ADCON2bits.ACQT       = ADC_ACQUISITION_20Tad;                                /* A/D conversion acquisition time selection                */
  ADCON2bits.ADFM       = 1;                                                    /* A/D result right justified                               */
  ADCON1bits.PCFG      &= ~SENS_PIN_Msk;                                        /* Configure pins as analog                                 */
  ADCON0bits.VCFG       = 0;                                                    /* Voltage regerence set to AVdd and AVss                   */
  ADCON0bits.CHS        = SENS_PIN_Pos;                                         /* Select the current channel                               */
  ADCON0bits.GO         = 0;                                                    /* ADC is stopped                                           */
  ADCON0bits.ADON       = 1;                                                    /* A/D converter module is on                               */
  IPR1bits.ADIP         = 1;                                                    /* High ISR                                                 */
  PIR1bits.ADIF         = 0;
  PIE1bits.ADIE         = 1;
}

static void sensTriggerInit(
  void)
{
  T3CONbits.TMR3ON      = 0;                                                    /* Stop if enabled                                          */
  T3CONbits.T3CCP1      = 0;                                                    /* TMR1 is the clock source for CCP                         */
  T1CONbits.TMR1ON      = 0;
  T1CONbits.RD16        = 0;
  T1CONbits.T1RUN       = 0;
  T1CONbits.T1CKPS      = SENS_TRIG_PRESCALER;
  T1CONbits.T1OSCEN     = 0;
  T1CONbits.nT1SYNC     = 0;
  T1CONbits.TMR1CS      = 0;                                                    /* Use internal clock                                       */
  T3CONbits.TMR3CS      = 0;                                                    
  TMR1L                 = 0;
  TMR1H                 = 0;
  CCP1CONbits.CCP1M     = 0x0b;                                                 /* Compare mode, set ECCP1IF, reset TMR1 or TMR3, start ADC */
  CCPR1H                = ((SENS_TRIG_PERIOD_COUNTER) >> 8) & 0xff;
  CCPR1L                = ((SENS_TRIG_PERIOD_COUNTER) >> 0) & 0xff;
  IPR1bits.CCP1IP       = 1;                                                    /* High ISR                                                 */
  PIR1bits.CCP1IF       = 0;
  PIE1bits.CCP1IE       = 0;
  IPR1bits.TMR1IP       = 1;                                                    /* High ISR                                                 */
  PIR1bits.TMR1IF       = 0;
  PIE1bits.TMR1IE       = 0;
  T1CONbits.TMR1ON      = 1;
}

void sensCompute(
  void)
{
  int16_t             signal;
  uint8_t             samples;
  static int16_t      signalBuff[SAMPLES_PER_POINT];
  static uint8_t      signalPos;

  signalBuff[signalPos++] = (int16_t)ADRES - (int16_t)SENS_ZERO_VALUE;
  signalPos &= ~((uint8_t)-1 << (BIT_UINT8_LOG2(SAMPLES_PER_POINT)));
  signal  = 0;
  samples = SAMPLES_PER_POINT;

  while (samples != 0)
  {
    samples--;
    signal += signalBuff[samples];
  }
  AbsSignal = ABS16(signal);
}

static void sensIsr(
  void)
{
  if (PIR1bits.ADIF == 1)
  {
    PIR1bits.ADIF = 0;
    sensCompute();
    sensProcess();
  }
}

static void sensProcess(
  void)
{
  enum sensProcessState
  {
    STATE_INIT,
    STATE_FIND_START,
    STATE_INITIAL_PEAK,
    STATE_COMPUTE_MIN,
    STATE_SEARCH_RISE,
    STATE_SEARCH_RISE_EXIT,
    STATE_SEARCH_DROP,
    STATE_SEARCH_DROP_EXIT,
    STATE_NOTIFY_ACTIVE,
    STATE_NOTIFY_INACTIVE
  };
  static enum sensProcessState state = STATE_INIT;
  static union
  {
    lvTmr_T             initialPeak;
#if (CFG_TIME_FIND_MAX != 0)
    lvTmr_T             findMax;
#endif
    lvTmr_T             findMin;
    lvTmr_T             notify;
  }                     tmr;
  static lvTmr_T        tmrExit;
  static uint16_t       absSignalMax;
#if (CFG_CURRENT_RISE_ALGORITHM == 0)
  static uint32_t       absSignalMinAcc;
#else
  static uint16_t       absSignalMin;
#endif
  static uint24_t       factAbsSignalMin;
  
  switch (state)
  {
    case STATE_INIT:
    {
      RELAY_DROP_SET_INACTIVE();
      RELAY_RISE_SET_INACTIVE();
      SensState = SENS_IDLE;
      state     = STATE_FIND_START;
      return;
    }
    case STATE_FIND_START:
    {
      if (AbsSignal > CURRENT_MA_TO_INT(CFG_CURRENT_START_DETECTION_MA))
      {
        tmr.initialPeak = lvTmrCreateI(LVTMR_MS_TO_TICK(CFG_TIME_INITIAL_PEAK));
        SensState = SENS_STARTED;
        state     = STATE_INITIAL_PEAK;
      }

      return;
    }
    case STATE_INITIAL_PEAK:
    {
      if (lvTmrIsDoneI(tmr.initialPeak))
      {
        tmr.findMin = lvTmrCreateI(LVTMR_MS_TO_TICK(CFG_TIME_FIND_MIN));
#if (CFG_CURRENT_RISE_ALGORITHM == 0)
        absSignalMinAcc = (uint32_t)0u;
#else
        absSignalMin = (uint16_t)-1;
#endif
        state = STATE_COMPUTE_MIN;
      }

      return;
    }
    case STATE_COMPUTE_MIN :
    {
#if (CFG_CURRENT_RISE_ALGORITHM == 0)
      absSignalMinAcc += AbsSignal;

      if (lvTmrIsDoneI(tmr.findMin))
      {
        uint24_t       absSignalMin;

        absSignalMin = (uint24_t)(absSignalMinAcc /
          (((uint32_t)CFG_TIME_FIND_MIN * (uint32_t)SENS_FREQUENCY) / (uint32_t)1000ul));
        factAbsSignalMin = absSignalMin * (uint24_t)(CFG_CURRENT_RISE_PERCENTAGE + 100ul);
        state = STATE_SEARCH_RISE;
      }
#else
      if (absSignalMin > AbsSignal)
      {
        if (AbsSignal > CURRENT_MA_TO_INT(CFG_CURRENT_MIN_DETECTION_MA))
        {
          absSignalMin = AbsSignal;
        }
      }

      if (lvTmrIsDoneI(tmr.findMin))
      {
        if (absSignalMin == (uint16_t)-1)
        {
          state = STATE_INIT;
        }
        else
        {
          factAbsSignalMin = absSignalMin * (uint24_t)(CFG_CURRENT_RISE_PERCENTAGE + 100ul);
          state = STATE_SEARCH_RISE;
        }
      }
#endif
      return;
    }
    case STATE_SEARCH_RISE :
    {
      uint24_t          factSignal;

      SensState  = SENS_SEARCHING_RISE;

      if (AbsSignal < CURRENT_MA_TO_INT(CFG_CURRENT_MIN_DETECTION_MA))
      {
        tmrExit = lvTmrCreateI(LVTMR_MS_TO_TICK(CFG_TIME_WAIT_FOR_EXIT));
        state = STATE_SEARCH_RISE_EXIT;

        return;
      }
      factSignal = (uint24_t)AbsSignal * (uint24_t)100ul;

      if (factAbsSignalMin < factSignal)
      {
        RELAY_RISE_SET_ACTIVE();
#if (CFG_TIME_FIND_MAX != 0)
        tmr.findMax = lvTmrCreateI(LVTMR_MS_TO_TICK(CFG_TIME_FIND_MAX));
#endif
        absSignalMax = 0u;
        SensState = SENS_RISE_FOUND;
        state     = STATE_SEARCH_DROP;
      }

      return;
    }
    case STATE_SEARCH_RISE_EXIT :
    {
      if (AbsSignal > CURRENT_MA_TO_INT(CFG_CURRENT_MIN_DETECTION_MA))
      {
        lvTmrDeleteI(tmrExit);
        state = STATE_SEARCH_RISE;
      }
      else if (lvTmrIsDoneI(tmrExit))
      {
        state = STATE_INIT;
      }

      return;
    }
    case STATE_SEARCH_DROP:
    {
      uint24_t          factAbsSignalMax;
      uint24_t          factSignal;

      SensState = SENS_SEARCHING_DROP;

#if (CFG_TIME_FIND_MAX != 0)
      if (lvTmrIsDoneI(tmr.findMax))
      {
        state = STATE_INIT;
      }
      else 
#endif
      if (AbsSignal < CURRENT_MA_TO_INT(CFG_CURRENT_MIN_DETECTION_MA))
      {
        tmrExit = lvTmrCreateI(LVTMR_MS_TO_TICK(CFG_TIME_WAIT_FOR_EXIT));
        state = STATE_SEARCH_DROP_EXIT;
      }
      else if (AbsSignal > CURRENT_MA_TO_INT(CFG_CURRENT_START_DETECTION_MA))
      {
        if (absSignalMax < AbsSignal)
        {
          absSignalMax = AbsSignal;
        }
        factAbsSignalMax = (uint24_t)absSignalMax * (uint24_t)100u;
        factSignal       = (uint24_t)AbsSignal *
          (uint24_t)(CFG_CURRENT_DROP_PERCENTAGE + 100u);

        if (factAbsSignalMax > factSignal)
        {
#if (CFG_TIME_FIND_MAX != 0)
          lvTmrDeleteI(tmr.findMax);
#endif
          SensState = SENS_DROP_FOUND;
          state     = STATE_NOTIFY_ACTIVE;
        }
      }

      return;
    }
    case STATE_SEARCH_DROP_EXIT :
    {
      if (AbsSignal > CURRENT_MA_TO_INT(CFG_CURRENT_MIN_DETECTION_MA))
      {
        lvTmrDeleteI(tmrExit);
        state = STATE_SEARCH_DROP;
      }
      else if (lvTmrIsDoneI(tmrExit))
      {
#if (CFG_TIME_FIND_MAX != 0)
        lvTmrDeleteI(tmr.findMax);
#endif
        state = STATE_INIT;
      }
#if (CFG_TIME_FIND_MAX != 0)
      else if (lvTmrIsDoneI(tmr.findMax))
      {
        lvTmrDeleteI(tmrExit);
        state = STATE_INIT;
      }
#endif

      return;
    }
    case STATE_NOTIFY_ACTIVE:
    {
      RELAY_DROP_SET_ACTIVE();
#if (CFG_TIME_NOTIFY_ACTIVE != 0)
      tmr.notify  = lvTmrCreateI(LVTMR_MS_TO_TICK(CFG_TIME_NOTIFY_ACTIVE));
      state       = STATE_NOTIFY_INACTIVE;
#endif
      return;
    }
    case STATE_NOTIFY_INACTIVE:
    {
#if (CFG_TIME_NOTIFY_ACTIVE != 0)
      if (lvTmrIsDoneI(tmr.notify))
      {
        state = STATE_INIT;
      }
#endif
      return;
    }
  }
}

/*-- Systick functions -------------------------------------------------------*/

static void sysTickInit(
  void)
{
#if (SYSTICK_USE_DEDICATED_TMR == 1)
  T0CONbits.TMR0ON      = 0;                                                    /* Turn TMR0 off if it was on                               */
  TMR0L                 = 0;
  TMR0H                 = 0;
  T0CONbits.T08BIT      = 0;                                                    /* 16bit mode                                               */
  T0CONbits.T0CS        = 0;                                                    /* Internal instruction clock CLK0                          */
  T0CONbits.T0SE        = 1;                                                    /* Increment on lot-to-high transition                      */
  T0CONbits.PSA         = 1;                                                    /* Prescaler is used                                        */
  T0CONbits.T0PS        = 0;                                                    /* 1/2                                                      */
  INTCON2bits.TMR0IP    = 0;                                                    /* Low ISR                                                  */
  INTCONbits.TMR0IF     = 0;
  INTCONbits.TMR0IE     = 1;
  T0CONbits.TMR0ON      = 1;                                                    /* Start timer                                              */
#else
#endif
}

static void sysTickIsr(
  void)
{
#if (SYSTICK_USE_DEDICATED_TMR == 1)
  if (INTCONbits.TMR0IF == 0x1)
  {
    INTCONbits.TMR0IF = 0;
    lvTmrEvalI();
  }
#else
  lvTmrEvalI();
#endif
}

static void displayInit(
  void)
{
  LED_DROP_LAT    &= ~LED_DROP_PIN_Msk;
  LED_DROP_PORT   &= ~LED_DROP_PIN_Msk;
  LED_DROP_TRIS   &= ~LED_DROP_PIN_Msk;

  LED_RISE_LAT    &= ~LED_RISE_PIN_Msk;
  LED_RISE_PORT   &= ~LED_RISE_PIN_Msk;
  LED_RISE_TRIS   &= ~LED_RISE_PIN_Msk;
}

static void displayStatus(
  void)
{
  static enum displayCmd ledDropCmd = LED_OFF;
  static enum displayCmd ledRiseCmd = LED_OFF;
  
  if (DeviceStatus != 0u)
  {
    LED_DROP_SET_ACTIVE();
    LED_RISE_SET_ACTIVE();

    while (true);
  }

  switch (SensState)
  {
    case SENS_IDLE:
    {
      ledDropCmd = LED_OFF;
      ledRiseCmd = LED_OFF;
      break;
    }
    case SENS_STARTED :
    {
      ledDropCmd = LED_OFF;
      ledDropCmd = LED_OFF;
      break;
    }
    case SENS_SEARCHING_RISE:
    {
      ledDropCmd = LED_OFF;
      ledRiseCmd = LED_BLINK;
      break;
    }
    case SENS_RISE_FOUND:
    {
      ledRiseCmd = LED_ON;
      break;
    }
    case SENS_SEARCHING_DROP:
    {
      ledDropCmd = LED_BLINK;
      break;
    }
    case SENS_DROP_FOUND:
    {
      ledDropCmd = LED_ON;
      break;
    }
  }
  ledDropFsm(ledDropCmd);
  ledRiseFsm(ledRiseCmd);
}

void ledDropFsm(
  enum displayCmd       cmd)
{
  enum displayState
  {
    DISP_ON,
    DISP_OFF,
    DISP_BLINK_ON,
    DISP_BLINK_OFF
  };
  static enum displayState ledDropState = DISP_OFF;
  static lvTmr_T        ledDropTmr;

  switch (ledDropState)
  {
    case DISP_ON:
    {
      LED_DROP_SET_ACTIVE();

      if (cmd == LED_OFF)
      {
        ledDropState = DISP_OFF;
      }
      else if (cmd == LED_BLINK)
      {
        ledDropTmr = lvTmrCreateI(LVTMR_MS_TO_TICK(CFG_TIME_LED_BLINK));
        ledDropState = DISP_BLINK_OFF;
      }
      break;
    }
    case DISP_OFF:
    {
      LED_DROP_SET_INACTIVE();

      if (cmd == LED_ON)
      {
        ledDropState = DISP_ON;
      }
      else if (cmd == LED_BLINK)
      {
        ledDropTmr = lvTmrCreateI(LVTMR_MS_TO_TICK(CFG_TIME_LED_BLINK));
        ledDropState = DISP_BLINK_ON;
      }
      break;
    }
    case DISP_BLINK_ON:
    {
      LED_DROP_SET_ACTIVE();

      if (cmd == LED_ON)
      {
        lvTmrDeleteI(ledDropTmr);
        ledDropState = DISP_ON;
      }
      else if (cmd == LED_OFF)
      {
        lvTmrDeleteI(ledDropTmr);
        ledDropState = DISP_OFF;
      }
      else if (lvTmrIsDoneI(ledDropTmr))
      {
        ledDropTmr = lvTmrCreateI(LVTMR_MS_TO_TICK(CFG_TIME_LED_BLINK));
        ledDropState = DISP_BLINK_OFF;
      }
    }
    case DISP_BLINK_OFF:
    {
      LED_DROP_SET_INACTIVE();

      if (cmd == LED_ON)
      {
        lvTmrDeleteI(ledDropTmr);
        ledDropState = DISP_ON;
      }
      else if (cmd == LED_OFF)
      {
        lvTmrDeleteI(ledDropTmr);
        ledDropState = DISP_OFF;
      }
      else if (lvTmrIsDoneI(ledDropTmr))
      {
        ledDropTmr = lvTmrCreateI(LVTMR_MS_TO_TICK(CFG_TIME_LED_BLINK));
        ledDropState = DISP_BLINK_ON;
      }
    }
  }
}

void ledRiseFsm(
  enum displayCmd       cmd)
{
  enum displayState
  {
    DISP_ON,
    DISP_OFF,
    DISP_BLINK_ON,
    DISP_BLINK_OFF
  };
  static enum displayState ledRiseState = DISP_OFF;
  static lvTmr_T        ledRiseTmr;

  switch (ledRiseState)
  {
    case DISP_ON:
    {
      LED_RISE_SET_ACTIVE();

      if (cmd == LED_OFF)
      {
        ledRiseState = DISP_OFF;
      }
      else if (cmd == LED_BLINK)
      {
        ledRiseTmr = lvTmrCreateI(LVTMR_MS_TO_TICK(CFG_TIME_LED_BLINK));
        ledRiseState = DISP_BLINK_OFF;
      }
      break;
    }
    case DISP_OFF:
    {
      LED_RISE_SET_INACTIVE();

      if (cmd == LED_ON)
      {
        ledRiseState = DISP_ON;
      }
      else if (cmd == LED_BLINK)
      {
        ledRiseTmr = lvTmrCreateI(LVTMR_MS_TO_TICK(CFG_TIME_LED_BLINK));
        ledRiseState = DISP_BLINK_ON;
      }
      break;
    }
    case DISP_BLINK_ON:
    {
      LED_RISE_SET_ACTIVE();

      if (cmd == LED_ON)
      {
        lvTmrDeleteI(ledRiseTmr);
        ledRiseState = DISP_ON;
      }
      else if (cmd == LED_OFF)
      {
        lvTmrDeleteI(ledRiseTmr);
        ledRiseState = DISP_OFF;
      }
      else if (lvTmrIsDoneI(ledRiseTmr))
      {
        ledRiseTmr = lvTmrCreateI(LVTMR_MS_TO_TICK(CFG_TIME_LED_BLINK));
        ledRiseState = DISP_BLINK_OFF;
      }
    }
    case DISP_BLINK_OFF:
    {
      LED_RISE_SET_INACTIVE();

      if (cmd == LED_ON)
      {
        lvTmrDeleteI(ledRiseTmr);
        ledRiseState = DISP_ON;
      }
      else if (cmd == LED_OFF)
      {
        lvTmrDeleteI(ledRiseTmr);
        ledRiseState = DISP_OFF;
      }
      else if (lvTmrIsDoneI(ledRiseTmr))
      {
        ledRiseTmr = lvTmrCreateI(LVTMR_MS_TO_TICK(CFG_TIME_LED_BLINK));
        ledRiseState = DISP_BLINK_ON;
      }
    }
  }
}

/*===================================  GLOBAL PRIVATE FUNCTION DEFINITIONS  ==*/
/*====================================  GLOBAL PUBLIC FUNCTION DEFINITIONS  ==*/

int main(
  int                   argc,
  char **               argv)
{
  (void)argc;
  (void)argv;
  lvTmrInit();
  hwInit();

  while (true);

  return (EXIT_SUCCESS);
}

void interrupt hiIsr(
  void)
{
  DEBUG_PIN_SET_ACTIVE();
  sensIsr();
  sysTickIsr();
  displayStatus();
  DEBUG_PIN_SET_INACTIVE();
}

void low_priority interrupt loIsr(
  void)
{
  sysTickIsr();
}

/*================================*//** @cond *//*==  CONFIGURATION ERRORS  ==*/
/** @endcond *//** @} *//** @} *//*********************************************
 * END of main.c
 ******************************************************************************/
