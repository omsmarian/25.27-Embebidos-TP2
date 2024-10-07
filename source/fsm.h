/***************************************************************************//**
  @file     fsm.h
  @brief    Finite State Machine Implementation
  @author   Group 4, based on the work of Daniel Jacoby
 ******************************************************************************/

#ifndef _FSM_H_
#define _FSM_H_

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

typedef struct transition_edge fsm_state_t;
struct transition_edge
{
	fsm_event_t  event;
	fsm_state_t * next_state;
	void (* callback)(void);
};

typedef enum
{
	EVENT_0,
	EVENT_1,
	EVENT_2,
	// Add more events here

	EVENTS_CANT
} fsm_event_t;


/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/**
 * @brief FSM initialization
 * @return Initial state
 */
fsm_state_t * fsmInit (void);

/**
 * @brief FSM interpreter
 * @param state Current state
 * @param event Incoming event
 * @return Next state
 */
fsm_state_t * fsm (fsm_state_t * state, fsm_event_t event);


/*******************************************************************************
 ******************************************************************************/

#endif // _FSM_H_
