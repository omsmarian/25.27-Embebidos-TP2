/*
 * SysTick.c
 *
 *  Created on: 19 ago 2024
 *      Author: asolari
 */

#include "SysTick.h"
#include "hardware.h"
#include "fsl_clock.h"

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 */

/*******************************************************************************
 * VARIABLES WITH GLOBAL SCOPE
 * ******************************************************************************/
static uint32_t ticksCounter = 0;

#define CALLBACK_QUANTITY 20
static SysTick_Callback_t arr_funcallback[CALLBACK_QUANTITY];
static uint32_t numOfCallbacks = 0;



bool SysTick_Init (uint32_t numOfInterruptions, ...)
{
	SysTick->CTRL = 0; // Disable SysTick
	SysTick->LOAD = (CLOCK_GetFreq(kCLOCK_CoreSysClk) / SYSTICK_ISR_FREQUENCY_HZ) - 1; // Set reload register
	SysTick->VAL = 0; // Reset counter
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk; // Enable SysTick


	numOfCallbacks = numOfInterruptions;
	va_list args;
	va_start(args, numOfInterruptions);
	for (uint32_t i = 0; i < numOfInterruptions; i++) {
		// Register callback
		arr_funcallback[i]= va_arg(args, SysTick_Callback_t);
		if(i>=CALLBACK_QUANTITY)
		{
			break;
		}
	}
	va_end(args);


	return true;
}

__ISR__ SysTick_Handler(void)
{
	ticksCounter++; // Increment counter of callbacks
	for(uint32_t i = 0; i < numOfCallbacks; i++)
	{
		if (ticksCounter % arr_funcallback[i].numTicks == 0 && arr_funcallback[i].funcallback != NULL) {
			arr_funcallback[i].funcallback();
		}
	}
}
