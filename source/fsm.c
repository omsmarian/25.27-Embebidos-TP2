/***************************************************************************//**
  @file     fsm.c
  @brief    Finite State Machine Implementation
  @author   Group 4, based on the work of Daniel Jacoby
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include "fsm.h"


/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

#define TABLE_END		EVENTS_CANT

// Number of edges per state (transitions) /////////////////////////////////////
#define STATE_0_EDGES	3														// Extra space for default transition
#define STATE_1_EDGES	2
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
static void fsmReset (void);


/*******************************************************************************
 * STATIC VARIABLES AND CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

// State-transition tables /////////////////////////////////////////////////////

extern fsm_state_t state0[];
extern fsm_state_t state1[];
// static fsm_state_t state0[];
// static fsm_state_t state1[];

state0 = { {EVENT_0,	state1, pass},
		   {EVENT_1,	state1, pass},
		   {EVENT_2,	state1, pass},
		   {TABLE_END,	state0, fsmReset} };

state1 = { {EVENT_0,	state1, pass},
		   {EVENT_1,	state1, pass},
		   {EVENT_2,	state1, pass},
		   {TABLE_END,	state0, fsmReset} };

// Add more states here


/*******************************************************************************
 *******************************************************************************
						GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

fsm_state_t * fsm (fsm_state_t * state, fsm_event_t event)
{
   	while ((state->event != event) && (state->event != TABLE_END))
		++state;
	
	(* state->callback)();

	return state->next_state;
}

fsm_state_t * fsmInit (void)
{
 	return state0;
}


/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

// Callback functions //////////////////////////////////////////////////////////

static void pass (void) {}
static void fsmReset (void) {}


/******************************************************************************/
