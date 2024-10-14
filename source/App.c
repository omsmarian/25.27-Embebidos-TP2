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

#include <string.h>
#include <stdlib.h>

#include "board.h"
#include "debug.h"
#include "hardware.h"
#include "macros.h"
#include "protocol.h"
#include "sensor.h"
#include "serial.h"
#include "station.h"
#include "timer.h"

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

enum {
	TIM_STATION		= TIMER_MS2TICKS(STATION_PERIOD_MS),
	TIM_SENSOR		= TIMER_MS2TICKS(SENSOR_PERIOD_MS),
	TIM_UPDATE		= TIMER_MS2TICKS(50),										// Send data every 50ms if there is any change
	TIM_NO_UPDATE	= TIMER_MS2TICKS(1000)										// Send data every 1s if there is no change
};

/*******************************************************************************
 * STATIC VARIABLES AND CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

static bool init_sensor, update[AXIS_CANT] = { false, false, false };
static uint8_t index_update, index_no_update;

static const uchar_t id2Chars[] = { 'R', 'C', 'O' };							// Roll (Rolido), Pitch (Cabeceo), Yaw (Orientacion)
static angle_t angles[AXIS_CANT] = { 0, 0, 0 };
static sensor_t data;
static sensor_status_t status;

static ticks_t timeout_station, timeout_sensor, timeout_update, timeout_no_update;

/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/

/**
 * @brief Update and send outgoing data
 * @param _index Poins (in update[]) to the axis to be checked for update
 * @param _update Update condition (value in update[])
 */
void updateOutgoing (uint8_t *_index, bool _update);

/**
 * @brief Receive and update incoming data
 */
void updateIncoming (void);

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
//	debugInit();

	timerInit();
	timeout_station		= timerStart(TIM_STATION);
	timeout_sensor		= timerStart(TIM_SENSOR);
	timeout_update		= timerStart(TIM_UPDATE);
	timeout_no_update	= timerStart(TIM_NO_UPDATE);
}

/**
 * @brief Run the application
 * @note This function is called constantly in an infinite loop
 */
void App_Run (void)
{
	if (!init_sensor) { init_sensor = sensorConfigStatus(); }					// Needs to be done with interrupts enabled
	else
	{
		if(timerExpired(timeout_station))										// Receive other stations data
		{
			if (timerExpired(timeout_sensor))									// Update own sensor data (without sending)
			{
				if (timerExpired(timeout_update))								// Send own station data
				{
					if(timerExpired(timeout_no_update))
					{
						updateOutgoing(&index_update, false);
						timeout_no_update = timerStart(TIM_NO_UPDATE);
					}
					else
						updateOutgoing(&index_no_update, true);

					timeout_update = timerStart(TIM_UPDATE);
				}

				status = *sensorGetStatus();
				update[ROLL]	= status.roll;
				update[PITCH]	= status.pitch;
				update[YAW]		= status.yaw;

				if (status.roll || status.pitch || status.yaw)
					data = *sensorGetAngleData();

		//		update[PITCH]	= update[PITCH];

				angles[ROLL]	= data.roll;
				angles[PITCH]	= data.pitch;
				angles[YAW]		= data.yaw;

				timeout_sensor = timerStart(TIM_SENSOR);
			}

			 updateIncoming();

			timeout_station = timerStart(TIM_STATION);
		}
	}
}

/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

void updateOutgoing (uint8_t *_index, bool _update)								// Send data to the stations and serial port
{
	uchar_t msg[PROTOCOL_DIGS];
	uint8_t len, sid;
	station_t s;
	protocol_t angle_data;

	for (uint8_t i = 0; i < AXIS_CANT; i++)
	{
		if (update[*_index] == _update)
		{
			angle_data = (protocol_t){ id2Chars[*_index], angles[*_index] };
			len = protocolPack(&angle_data, msg);
			s = (station_t){ GN, msg, len };
			stationSend(&s);

			sid = NUM2ASCII(GN);
			serialWriteData(&sid, 1);
			serialWriteData(msg, len);
			sid = '\n';
			serialWriteData(&sid, 1);

			i = AXIS_CANT;
		}

		(*_index)++;
		(*_index) %= AXIS_CANT;
	}
}

void updateIncoming (void)														// Receive data from the serial port
{
	uchar_t msg[PROTOCOL_DIGS];
	uint8_t len, sid;
	station_t s;
	protocol_t angle_data;

	s = (station_t){ GN, msg, PROTOCOL_DIGS };
	stationReceive(&s);
	if (s.len)
	{
		sid = NUM2ASCII(s.id);
		serialWriteData(&sid, 1);
		serialWriteData(s.data, s.len);
		sid = '\n';
		serialWriteData(&sid, 1);
		// len = protocolPack(protocolUnpack(s.data, s.len), msg);
		// serialWriteData(msg, len);
	}
}

/******************************************************************************/
