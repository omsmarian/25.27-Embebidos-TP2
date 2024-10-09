/***************************************************************************//**
  @file     App.c
  @brief    Application functions
  @author   Group 4: - Oms, Mariano
                     - Solari Raigoso, Agustín
                     - Wickham, Tomás
                     - Vieira, Valentin Ulises
  @note     Based on the work of Nicolás Magliola
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include "board.h"
#include "fsm.h"
#include "gpio.h"
#include "hardware.h"
#include "sensor.h"
#include "serial.h"
#include "station.h"
#include "timer.h"
#include "uart.h"


/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/

/*
 * @brief Check if an event occurred (new message in any peripheral) and get it
 * @return Event to be processed by the FSM, EVENTS_CANT if there was no event
 */
fsm_event_t getEvent (void);


/*******************************************************************************
 * STATIC VARIABLES AND CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

static fsm_state_t * state = NULL;
// static fsm_event_t event = EVENTS_CANT;

static ticks_t timeout;

// static int8_t angle;
static sensor_t data;


/*******************************************************************************
 *******************************************************************************
                        GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

/*
 * @brief Initialize the application
 * @note This function is called once at the beginning of the main program
 */
void App_Init (void)
{
	serialInit();
	sensorInit();
	stationInit();

	timerInit();
	timeout = timerStart(TIMER_MS2TICKS(1000));

	state = fsmInit();
}

/*
 * @brief Run the application
 * @note This function is called constantly in an infinite loop
 */
void App_Run (void)
{
	// fsm_event_t event = getEvent();
	// if(event != EVENTS_CANT)
	// 	state = fsm(state, event);

	// if(uartIsRxMsg(UART0_ID))
	// 	// Do something
	// else if(i2cIsRxMsg(I2C0_ID))
	// {
	// 	i2cReadMsg(I2C0_ID, &angle, 1);
	// 	state = fsm(state, I2C_MSG);
	// }
	// else if(canIsRxMsg(CAN0_ID))
	// 	// Do something

	if(timerExpired(timeout))
	{
		if(sensorIsRxMsg())
		{
			data = *sensorGetData();
			stationSendData(&data);
			serialSendData(processData(&data));
			uartWriteMsg(UART0_ID, &data, sizeof(sensor_t));
		}

		if(stationIsRxMsg())
		{
			data = *stationGetData(&data);
			serialSendData(processData(&data));
			uartWriteMsg(UART0_ID, &data, sizeof(sensor_t));
		}

		timeout = timerStart(TIMER_MS2TICKS(1000));
	}
}


/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

// fsm_event_t getEvent (void)
// {
// 	fsm_event_t event = EVENTS_CANT;

// 	// if(uartIsRxMsg(UART0_ID))
// 	// 	event = UART_MSG;
// 	// else if(i2cmIsRxMsg(I2C0_ID))
// 	// 	event = I2C_MSG;
// 	// else if(canIsRxMsg(CAN0_ID))
// 	// 	event = CAN_MSG;

// 	return event;
// }

char* processData (sensor_t* data)
{
	// static char msg[32];
	// sprintf(msg, "Roll: %d, Pitch: %d, Yaw: %d", data->roll, data->pitch, data->yaw);
	// return msg;
}


/******************************************************************************/
