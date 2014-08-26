/* 
 * File:   config.h
 * Author: Nenad Radulovic
 *
 * Created on 13.12.2013., 12.37
 */

#ifndef CONFIG_H
#define	CONFIG_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <p18f1220.h>

/**@brief       Initial peak time duration in ms
 */
#define CFG_TIME_INITIAL_PEAK           150u

/**@brief       Time period to calculate average value (which corresponds to
 *              minimal value). Time in ms.
 * @details     This period is positioned right after Initial peak time.
 */
#define CFG_TIME_FIND_MIN               20u

/**@brief       Allowable time to search for maximum value. Time in ms.
 */
#define CFG_TIME_FIND_MAX               0

#define CFG_TIME_WAIT_FOR_EXIT          40u
#define CFG_TIME_NOTIFY_ACTIVE          0
#define CFG_TIME_LED_BLINK              60u
#define CFG_CURRENT_RISE_PERCENTAGE     100u
#define CFG_CURRENT_RISE_ALGORITHM      0
#define CFG_CURRENT_DROP_PERCENTAGE     40u
#define CFG_CURRENT_START_DETECTION_MA  1000u
#define CFG_CURRENT_MIN_DETECTION_MA    350u
#define CFG_SENS_UV_PER_AMPER           66000ul
#define CFG_ADC_UV_QUANTUM              4882ul
 
#define RELAY_DROP_PORT                 PORTB
#define RELAY_DROP_TRIS                 TRISB
#define RELAY_DROP_LAT                  LATB
#define RELAY_DROP_PIN_Pos              0
#define RELAY_DROP_PIN_Msk              (0x01u << RELAY_DROP_PIN_Pos)

#define RELAY_RISE_PORT                 PORTB
#define RELAY_RISE_TRIS                 TRISB
#define RELAY_RISE_LAT                  LATB
#define RELAY_RISE_PIN_Pos              1
#define RELAY_RISE_PIN_Msk              (0x01u << RELAY_RISE_PIN_Pos)

#define DEBUG_PIN_PORT                  PORTB
#define DEBUG_PIN_TRIS                  TRISB
#define DEBUG_PIN_LAT                   LATB
#define DEBUG_PIN_PIN_Pos               4
#define DEBUG_PIN_PIN_Msk               (0x01u << DEBUG_PIN_PIN_Pos)
  
#define SENS_PORT                       PORTA
#define SENS_TRIS                       TRISA
#define SENS_LAT                        LATA
#define SENS_PIN_Pos                    0
#define SENS_PIN_Msk                    (0x01u << SENS_PIN_Pos)

#define LED_RISE_PORT                   PORTA
#define LED_RISE_TRIS                   TRISA
#define LED_RISE_LAT                    LATA
#define LED_RISE_PIN_Pos                1
#define LED_RISE_PIN_Msk                (0x01u << LED_RISE_PIN_Pos)

#define LED_DROP_PORT                   PORTA
#define LED_DROP_TRIS                   TRISA
#define LED_DROP_LAT                    LATA
#define LED_DROP_PIN_Pos                4
#define LED_DROP_PIN_Msk                (0x01u << LED_DROP_PIN_Pos)

#define SENS_FREQUENCY                  1000ul

#define SENS_ZERO_VALUE                 (1024u / 2u)

#define SAMPLES_PER_POINT               16

#define LVTMR_TICK_FREQUENCY            SENS_FREQUENCY
  
#define CPU_FREQUENCY                   32000000ul
#define SYSTICK_USE_DEDICATED_TMR       0

#define SENS_TRIG_DIVISOR               (CPU_FREQUENCY / (4ul * SENS_FREQUENCY))
  
#if   (SENS_TRIG_DIVISOR < 65534)
#define SENS_TRIG_PRESCALER             0x00
#define SENS_TRIG_PERIOD_COUNTER        SENS_TRIG_DIVISOR
#elif (SENS_TRIG_DIVISOR < 131070)
#define SENS_TRIG_PRESCALER             0x01
#define SENS_TRIG_PERIOD_COUNTER        (SENS_TRIG_DIVISOR / 2)
#elif (SENS_TRIG_DIVISOR < 262142)
#define SENS_TRIG_PRESCALER             0x02
#define SENS_TRIG_PERIOD_COUNTER        (SENS_TRIG_DIVISOR / 4)
#elif (SENS_TRIG_DIVISOR < 524286)
#define SENS_TRIG_PRESCALER             0x04
#define SENS_TRIG_PERIOD_COUNTER        (SENS_TRIG_DIVISOR / 8)
#else
# error "Invalid setting for sensor trigger frequency"
#endif

#if (SAMPLES_PER_POINT != 2) && (SAMPLES_PER_POINT != 4) &&                     \
    (SAMPLES_PER_POINT != 8) && (SAMPLES_PER_POINT != 16)
#error "Invalid settting for number of samples per data point"
#endif
            
#ifdef	__cplusplus
}
#endif

#endif	/* CONFIG_H */

