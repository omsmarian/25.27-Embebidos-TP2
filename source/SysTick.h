/***************************************************************************//**
  @file     SysTick.h
  @brief    SysTick driver
  @author   Nicol�s Magliola
 ******************************************************************************/

#ifndef _SYSTICK_H_
#define _SYSTICK_H_

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>





/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

#define SYSTICK_ISR_FREQUENCY_HZ 1000U




/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/
typedef void (*Funcallback_t)(void);


typedef struct
{
	Funcallback_t funcallback;
	uint32_t numTicks;
} SysTick_Callback_t;


//_Funcallback* arr_funcallback = (Funcallback_t*)malloc(sizeof(SysTick_Callback_t)*10);

/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/**
 * @brief Initialize SysTic driver
 * @param funcallback Function to be call every SysTick
 * @return Initialization and registration succeed
 */
bool SysTick_Init (uint32_t numOfInterruptions, ...);

/*******************************************************************************
 ******************************************************************************/

#endif // _SYSTICK_H_
