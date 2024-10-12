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
// #include "fsm.h"
#include "gpio.h"
#include "hardware.h"
#include "macros.h"
#include "protocol.h"
#include "sensor.h"
#include "serial.h"
#include "station.h"
#include "timer.h"


/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/

/*
 * @brief Check if an event occurred (new message in any peripheral) and get it
 * @return Event to be processed by the FSM, EVENTS_CANT if there was no event
 */
// fsm_event_t getEvent (void);


/*******************************************************************************
 * STATIC VARIABLES AND CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

// static fsm_state_t * state = NULL;
// static fsm_event_t event = EVENTS_CANT;

static ticks_t timeout_10ms, timeout_50ms, timeout_1s, timeout_2s;

// static int8_t angle;
static sensor_t data;


/*******************************************************************************
 *******************************************************************************
                        GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

/**
 * @brief Initialize the application
 * @note This function is called once at the beginning of the main program
 */
void App_Init (void)
{
	serialInit();
	sensorInit();
	stationInit();

	timerInit();
	timeout_10ms	= timerStart(TIMER_MS2TICKS(10));
	timeout_50ms	= timerStart(TIMER_MS2TICKS(50));
	timeout_1s		= timerStart(TIMER_MS2TICKS(1000));
	timeout_2s		= timerStart(TIMER_MS2TICKS(2000));

	// state = fsmInit();
}

/**
 * @brief Run the application
 * @note This function is called constantly in an infinite loop
 */
void App_Run (void)
{
	sensor_axis_t axis[ALL] = { ROLL, PITCH, YAW };
	angle_t angles[ALL] = { data.roll, data.pitch, data.yaw };
	station_id_t stations[STATIONS_CANT] = { STATION_1, STATION_2, STATION_3, STATION_4, STATION_5 };
	static bool update[ALL] = { false, false, false };
	static uint8_t index = 0, index2 = 0;
	protocol_t angle_data;
	uchar_t *msg, *station;

	if(timerExpired(timeout_10ms))
	{
		if (sensorGetStatus(ALL))
			data = *sensorGetAngleData();

		update[ROLL]	= sensorGetStatus(ROLL);
		update[PITCH]	= sensorGetStatus(PITCH);
		update[YAW]		= sensorGetStatus(YAW);

		angles[ROLL]	= data.roll;
		angles[PITCH]	= data.pitch;
		angles[YAW]		= data.yaw;

		if (timerExpired(timeout_50ms))
		{
			if(timerExpired(timeout_1s))
			{
				for (uint8_t i = 0; i < ALL; i++)
				{
					if (!update[index])
					{
						angle_data = (protocol_t){ axis[index], angles[index] };
						stationSendAll(protocolPack(&angle_data), PROTOCOL_DIGS);

						i = ALL;
					}

					index++;
					index %= ALL;
				}
			}
			else
			{
				for (uint8_t i = 0; i < ALL; i++)
				{
					if (update[index2])
					{
						angle_data = (protocol_t){ axis[index2], angles[index2] };
						stationSendAll(protocolPack(&angle_data), PROTOCOL_DIGS);

						i = ALL;
					}

					index2++;
					index2 %= ALL;
				}
			}

			timeout_50ms = timerStart(TIMER_MS2TICKS(50));
		}

		for (int i = 0; i < STATIONS_CANT; i++)
		{
			*station = NUM2ASCII(stationReceive(msg));
			if (*station != STATIONS_CANT)
			{
				serialWriteData(station, 1);
				serialWriteData(protocolPack(protocolUnpack(msg)), PROTOCOL_DIGS);
			}
		}

		timeout_10ms = timerStart(TIMER_MS2TICKS(10));
	}
}


/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/


/******************************************************************************/
