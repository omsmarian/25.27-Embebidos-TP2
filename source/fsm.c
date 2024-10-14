/***************************************************************************//**
  @file     fsm.c
  @brief    Finite State Machine Implementation
  @author   Group 4: - Oms, Mariano
                     - Solari Raigoso, Agustín
                     - Wickham, Tomás
                     - Vieira, Valentin Ulises
  @note     Based on the work of Daniel Jacoby
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include "fsm.h"

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

#define DEVELOPMENT_MODE	1

#define TABLE_END			EVENTS_CANT

// Number of edges per state (transitions) /////////////////////////////////////

#define STATE_0_EDGES		3													// Extra space for default transition
#define STATE_1_EDGES		2
// Add more state edges here


/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/

/*
 * @brief Do nothing
 */
static void pass (void);

/*
 * @brief Reset the FSM
 */
static void reset (void);

static void updateAngle (void);

/*******************************************************************************
 * STATIC VARIABLES AND CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

// State-transition tables /////////////////////////////////////////////////////

static fsm_state_t READ_ORIENTATION[];
static fsm_state_t TRANSMIT_CAN[];
static fsm_state_t RECEIVE_CAN[];
static fsm_state_t TRASMIT_UART[];
static fsm_state_t CHECK_UPDATE[];
static fsm_state_t WAIT[];
static fsm_state_t ERROR[];
static fsm_state_t END[];

static fsm_state_t CHECK_UPDATE[] = { {I2C_MSG, READ_ORIENTATION, updateAngle},
		   							  {TABLE_END, END, reset} };

static fsm_state_t READ_ORIENTATION[] = { {TABLE_END, END, reset} };

// Add more states here

/*******************************************************************************
 *******************************************************************************
						GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

fsm_state_t* fsm (fsm_state_t* state, fsm_event_t event)
{
   	while ((state->event != event) && (state->event != TABLE_END))
		++state;
	
	(*state->callback)();

	return state->next_state;
}

fsm_state_t* fsmInit (void)
{
 	return CHECK_UPDATE;
}

/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

// Callback functions //////////////////////////////////////////////////////////

static void pass (void) {}
static void reset (void) {}

static void updateAngle (void)
{
	// Update angle
}

/******************************************************************************/
