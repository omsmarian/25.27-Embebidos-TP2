/***************************************************************************//**
  @file     fsm.h
  @brief    Finite State Machine Implementation
  @author   Group 4: - Oms, Mariano
                     - Solari Raigoso, Agustín
                     - Wickham, Tomás
                     - Vieira, Valentin Ulises
  @note     Based on the work of Daniel Jacoby
 ******************************************************************************/

#ifndef _FSM_H_
#define _FSM_H_

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

typedef enum {
	UART_MSG,
	I2C_MSG,
	CAN_MSG,

	EVENTS_CANT
} fsm_event_t;

//typedef enum {
//	INIT,
//	READ_ORIENTATION,
//	TRANSMIT_CAN,
//	RECEIVE_CAN,
//	TRASMIT_UART,
//	CHECK_UPDATE,
//	WAIT,
//	ERROR,
//	END,
//
//	STATES_CANT
//} fsm_state_id_t;

typedef struct transition_edge fsm_state_t;
struct transition_edge {
	fsm_event_t event;
	fsm_state_t * next_state;
	void (* callback)(void);
};

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
