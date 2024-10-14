/***************************************************************************//**
  @file     pisr.c
  @brief    Periodic Interrupt (PISR) driver
  @author   Group 4
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include "debug.h"
#include "hardware.h"
#include "pisr.h"

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

#define SYSTICK_LOAD_INIT   ((__CORE_CLOCK__ / PISR_FREQUENCY_HZ) - 1U)

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

typedef struct {
	pisr_callback_t funadrrs;
	uint32_t time;
	uint32_t counter;
} funData;

/*******************************************************************************
 * STATIC VARIABLES AND CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

static funData funArr[PISR_CANT];
static bool init_flag;
static uint32_t count;

/*******************************************************************************
 *******************************************************************************
                        GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

bool pisrRegister (pisr_callback_t fun, unsigned int period)
{
	if(!init_flag) // Initialize SysTick
	{
		SysTick->CTRL = 0x00;
		SysTick->LOAD = SYSTICK_LOAD_INIT;
		SysTick->VAL= 0x00;
		SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk|SysTick_CTRL_TICKINT_Msk|SysTick_CTRL_ENABLE_Msk;
		init_flag = true;
	}
	
	if(count < PISR_CANT) // Load the function and its period
	{
		funArr[count].funadrrs = fun;
		funArr[count].time = period;
		funArr[count++].counter = period;
	}

	return 0;
}

/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

__ISR__ SysTick_Handler(void)
{
#if DEBUG_PISR
P_DEBUG_TP_SET
#endif
	for(int i = 0; i < count ; i++)
		if(--((funArr[i]).counter) == 0)
		{
			(funArr[i]).funadrrs();
			(funArr[i]).counter = (funArr[i]).time;
		}
#if DEBUG_PISR
P_DEBUG_TP_CLR
#endif
}

/******************************************************************************/
