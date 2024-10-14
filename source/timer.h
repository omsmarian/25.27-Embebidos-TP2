/***************************************************************************//**
  @file     timer.h
  @brief    Timer driver. Simple implementation, support multiple timers.
  @author   Group 4: - Oms, Mariano
                     - Solari Raigoso, Agustín
                     - Wickham, Tomás
                     - Vieira, Valentin Ulises
  @note     Based on the work of Nicolás Magliola
 ******************************************************************************/

#ifndef _TIMER_H_
#define _TIMER_H_

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

#define TIMER_TICK_US       500
#define TIMER_MS2TICKS(ms)	(1000*(ms)/TIMER_TICK_US)

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

typedef int64_t ticks_t;

/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/**
 * @brief Initialice timer and corresponding peripheral
 */
void timerInit(void);

// Non-Blocking services ///////////////////////////////////////////////////////

/**
 * @brief Begin to run a new timer
 * @param ticks time until timer expires, in ticks
 * @return Timeout value
 */
ticks_t timerStart(ticks_t ticks);

/**
 * @brief Verify if a timer has run timeout
 * @param timeout timeout to check for expiration
 * @return 1 = timer expired
 */
bool timerExpired(ticks_t timeout);

// Blocking services ///////////////////////////////////////////////////////////

/**
 * @brief Wait the specified time.
 * @param ticks time to wait in ticks
 */
void timerDelay(ticks_t ticks);

ticks_t timerCounter(void);

/*******************************************************************************
 ******************************************************************************/

#endif // _TIMER_H_
