/***************************************************************************//**
  @file     App.c
  @brief    Application functions
  @author   Group 4, based on the work of Nicolás Magliola
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include "board.h"
#include "fsm.h"
#include "gpio.h"
#include "hardware.h"
#include "timer.h"
#include "uart.h"

/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/

/*
 * @brief Check for events
 * @return True if there was an event
 */
bool getStatus (void);

/*
 * @brief Get the event that occurred
 * @return Event to be processed by the FSM
 * @note This function should be called only if getStatus() returns true
 */
fsm_event_t getEvent (void);


/*******************************************************************************
 * STATIC VARIABLES AND CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

static fsm_state_t * state = NULL;
// static fsm_event_t event = EVENTS_CANT;


/*******************************************************************************
 *******************************************************************************
                        GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

/* Función que se llama 1 vez, al comienzo del programa */
void App_Init (void)
{
	uart_cfg_t config = {9600, UART_MODE_8,
						 UART_PARITY_NONE,
						 UART_STOPS_1,
						 UART_RX_TX_ENABLED,
						 UART_FIFO_RX_TX_ENABLED};
	uartInit(UART0_ID, config);

	timerInit();

	state = fsmInit();
}

/* Función que se llama constantemente en un ciclo infinito */
void App_Run (void)
{
	if(getStatus())
		state = fsm(state, getEvent());
}


/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

bool getStatus(void)
{
	// Check for events
}

fsm_event_t getEvent(void)
{
	// Get event
}


/******************************************************************************/
