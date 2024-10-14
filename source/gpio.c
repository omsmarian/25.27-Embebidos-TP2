/***************************************************************************//**
  @file     gpio.h
  @brief    Simple GPIO Pin services, similar to Arduino
  @author   Group 4: - Oms, Mariano
                     - Solari Raigoso, Agustín
                     - Wickham, Tomás
                     - Vieira, Valentin Ulises
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include "board.h"
#include "debug.h"
#include "gpio.h"
#include "hardware.h"
#include "macros.h"
#include "MK64F12.h"

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

#define DEVELOPMENT_MODE	1

#define PORTS_CANT			5	// A, B, C, D, E
#define FUN_CANT			32	// Only 1 per PIN

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

pinIrqFun_t irqFuns[PORTS_CANT][FUN_CANT] = { NULL };

/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/

uint8_t PinBit2Num (uint32_t pin);

/*******************************************************************************
 * STATIC VARIABLES AND CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

static GPIO_Type * const    GPIO_Ports[]        =   GPIO_BASE_PTRS;
static PORT_Type * const    PORT_Ports[]        =   PORT_BASE_PTRS;
static uint32_t             SCSCGC5Ports_t[]    = { SIM_SCGC5_PORTA(HIGH),
									  	  	        SIM_SCGC5_PORTB(HIGH),
											        SIM_SCGC5_PORTC(HIGH),
											        SIM_SCGC5_PORTD(HIGH),
											        SIM_SCGC5_PORTE(HIGH) };
static uint8_t const        PORT_IRQn[] 		=   PORT_IRQS;
static uint8_t const        GPIO_IRQn[] 		= { PORT_eDisabled,
									 	 	        PORT_eInterruptRising,
											        PORT_eInterruptFalling,
											        PORT_eInterruptEither };

/*******************************************************************************
 *******************************************************************************
                        GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

void gpioMode (pin_t pin, uint8_t mode)
{
	PINData_t pin_data = { PIN2PORT(pin), PIN2NUM(pin) };
	uint32_t * pin_PCR = &(PORT_Ports[pin_data.port]->PCR[pin_data.num]);
	uint32_t * pin_PDDR = &(GPIO_Ports[pin_data.port]->PDDR);

	SIM->SCGC5 |= SCSCGC5Ports_t[pin_data.port];
	*pin_PCR |= PORT_PCR_MUX(PORT_mGPIO);

	if (mode == OUTPUT)
		*pin_PDDR |= (HIGH << (pin_data.num));
	else
	{
		*pin_PDDR |= (LOW << (pin_data.num));

		if (mode != INPUT)
		{
			*pin_PCR |= PORT_PCR_PE(HIGH);

			if (mode == INPUT_PULLUP)
				*pin_PCR |= PORT_PCR_PS(HIGH);
			else
				*pin_PCR |= PORT_PCR_PS(LOW);
		}
		else
			*pin_PCR |= PORT_PCR_PE(LOW);
	}
}

bool gpioIRQ (pin_t pin, uint8_t irqMode, pinIrqFun_t irqFun)
{
	bool status = true;

	NVIC_EnableIRQ(PORT_IRQn[PIN2PORT(pin)]);
	PORT_Ports[PIN2PORT(pin)]->PCR[PIN2NUM(pin)] |= PORT_PCR_IRQC(GPIO_IRQn[irqMode]);
	if (PIN2NUM(pin) < FUN_CANT && irqFun != NULL)
	{
		irqFuns[PIN2PORT(pin)][PIN2NUM(pin)] = irqFun;
		status = false;
	}

	return status; // 0 = success
}

void gpioWrite (pin_t pin, bool value)
{
//	GPIO_Ports[PIN2PORT(pin)]->PDOR &= ~(HIGH<<PIN2NUM(pin)); // Clear bit
//	GPIO_Ports[PIN2PORT(pin)]->PDOR |= (value<<PIN2NUM(pin)); // Change bit
	if (value)
		GPIO_Ports[PIN2PORT(pin)]->PSOR = (HIGH<<PIN2NUM(pin));
	else
		GPIO_Ports[PIN2PORT(pin)]->PCOR = (HIGH<<PIN2NUM(pin));
}

void gpioToggle (pin_t pin)
{
//	GPIO_Ports[PIN2PORT(pin)]->PDOR ^= (HIGH<<PIN2NUM(pin));
	GPIO_Ports[PIN2PORT(pin)]->PTOR = (HIGH<<PIN2NUM(pin));
}

bool gpioRead (pin_t pin)
{
//	return ((GPIO_Ports[PIN2PORT(pin)]->PDIR & (HIGH<<PIN2NUM(pin)))>>PIN2NUM(pin));
	return GPIO_Ports[PIN2PORT(pin)]->PDIR & (HIGH<<PIN2NUM(pin));
}

/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

__ISR__ PORTA_IRQHandler (void)
{
#if DEBUG_GPIO
D_DEBUG_TP_SET
#endif
	for (uint8_t i = 0; i < 32; i++) // Very slow, there are better ways such as CLZ
		if (BITGET(PORTA->ISFR, i))
		{
			PORTA->PCR[i] |= PORT_PCR_ISF(HIGH);
			if(irqFuns[PA][i] != NULL)
				irqFuns[PA][i]();
		}
#if DEBUG_GPIO
D_DEBUG_TP_CLR
#endif
}

__ISR__ PORTB_IRQHandler (void)
{
#if DEBUG_GPIO
D_DEBUG_TP_SET
#endif
	for (uint8_t i = 0; i < 32; i++)
		if (BITGET(PORTB->ISFR, i))
		{
			PORTB->PCR[i] |= PORT_PCR_ISF(HIGH);
			if(irqFuns[PB][i] != NULL)
				irqFuns[PB][i]();
		}
#if DEBUG_GPIO
D_DEBUG_TP_CLR
#endif
}

__ISR__ PORTC_IRQHandler (void)
{
#if DEBUG_GPIO
D_DEBUG_TP_SET
#endif
	for (uint8_t i = 0; i < 32; i++)
		if (BITGET(PORTC->ISFR, i))
		{
			PORTC->PCR[i] |= PORT_PCR_ISF(HIGH);
			if(irqFuns[PC][i] != NULL)
				irqFuns[PC][i]();
		}
#if DEBUG_GPIO
D_DEBUG_TP_CLR
#endif
}

__ISR__ PORTD_IRQHandler (void)
{
#if DEBUG_GPIO
D_DEBUG_TP_SET
#endif
	for (uint8_t i = 0; i < 32; i++)
		if (BITGET(PORTD->ISFR, i))
		{
			PORTD->PCR[i] |= PORT_PCR_ISF(HIGH);
			if(irqFuns[PD][i] != NULL)
				irqFuns[PD][i]();
		}
#if DEBUG_GPIO
D_DEBUG_TP_CLR
#endif
}

__ISR__ PORTE_IRQHandler (void)
{
#if DEBUG_GPIO
D_DEBUG_TP_SET
#endif
	for (uint8_t i = 0; i < 32; i++)
		if (BITGET(PORTE->ISFR, i))
		{
			PORTE->PCR[i] |= PORT_PCR_ISF(HIGH);
			if(irqFuns[PE][i] != NULL)
				irqFuns[PE][i]();
		}
#if DEBUG_GPIO
D_DEBUG_TP_CLR
#endif
}

// Helper functions ////////////////////////////////////////////////////////////

uint8_t PinBit2Num (uint32_t pin)
{
	uint8_t i = 0;
	while (pin >> i++);
	return i;
}

/******************************************************************************/
