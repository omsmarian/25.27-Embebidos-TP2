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

#include <gpio.h>
#include <string.h>
#include <stdlib.h>

#include "board.h"
#include "hardware.h"
#include "macros.h"
#include "protocol.h"
#include "sensor.h"
#include "serial.h"
#include "station.h"
#include "timer.h"

/*******************************************************************************
 * STATIC VARIABLES AND CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

static sensor_t data;
sensor_axis_t axis[ALL] = { ROLL, PITCH, YAW };
angle_t angles[ALL] = { 0, 0, 0 };
station_id_t stations[STATIONS_CANT] = { STATION_1, STATION_2, STATION_3, STATION_4, STATION_5 };
static bool update[ALL] = { false, false, false };
static uint8_t index1 = 0, index2 = 0;
protocol_t angle_data, *p;
uchar_t sid, station[5];
uchar_t msg[PROTOCOL_DIGS] = "hola";
uint8_t len = 0;
station_t s;
static bool init_sensor;
sensor_status_t status;

const uchar_t id2Chars[] = { 'R', 'C', 'O' };									// Roll (Rolido), Pitch (Cabeceo), Yaw (Orientacion)

static ticks_t timeout_1ms, timeout_10ms, timeout_50ms, timeout_1s, timeout_2s;

typedef enum {
	MS_1 = TIMER_MS2TICKS(1),
	MS_10 = TIMER_MS2TICKS(10),
	MS_50 = TIMER_MS2TICKS(50),
	S_1 = TIMER_MS2TICKS(1000),
	S_2 = TIMER_MS2TICKS(2000)
} ms_t;

void update2 (uint8_t *_index, bool __index);

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
	timeout_1ms		= timerStart(TIMER_MS2TICKS(1));
	timeout_10ms	= timerStart(TIMER_MS2TICKS(10));
	timeout_50ms	= timerStart(TIMER_MS2TICKS(5));
	timeout_1s		= timerStart(TIMER_MS2TICKS(10));
	timeout_2s		= timerStart(TIMER_MS2TICKS(2000));
}

/**
 * @brief Run the application
 * @note This function is called constantly in an infinite loop
 */
void App_Run (void)
{
	if (!init_sensor) { init_sensor = sensorConfig(); }							// Needs to be done with interrupts enabled
	else
	{
		if(timerExpired(timeout_1s))
		{
			update2(&index1, 0);
			timeout_1s = timerStart(TIMER_MS2TICKS(1000));
		}

		if (timerExpired(timeout_50ms))
		{
			update2(&index2, 1);
			timeout_50ms = timerStart(TIMER_MS2TICKS(50));
		}

		if(timerExpired(timeout_1ms))												// Update own data (without sending) and receive other stations data
		{
			status = *sensorGetStatus();
			update[ROLL]	= status.roll;
			update[PITCH]	= status.pitch;
			update[YAW]		= status.yaw;

			if (status.roll || status.pitch || status.yaw)
				data = *sensorGetAngleData();

			angles[ROLL]	= data.roll;
			angles[PITCH]	= data.pitch;
			angles[YAW]		= data.yaw;

			s = (station_t){ GN, msg, PROTOCOL_DIGS };
			stationReceive(&s);
			if (s.len)
			{
				*station = NUM2ASCII(s.id);
				serialWriteData(station, 1);
				serialWriteData(s.data, s.len);
				// len = protocolPack(protocolUnpack(s.data, s.len), msg);
				// serialWriteData(msg, len);
			}

			timeout_1ms = timerStart(TIMER_MS2TICKS(1));
		}
	}
}

/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

void update2 (uint8_t *_index, bool __index)									// Send data to the stations and serial port
{
	for (uint8_t i = 0; i < ALL; i++)
	{
		if (update[*_index] == __index)
		{
			angle_data = (protocol_t){ id2Chars[axis[*_index]], angles[*_index] };
			len = protocolPack(&angle_data, msg);
			s = (station_t){ GN - 1, msg, len };
			stationSend(&s);

			*station = NUM2ASCII(GN);
			serialWriteData(station, 1);
			serialWriteData(msg, len);

			i = ALL;
		}

		(*_index)++;
		(*_index) %= ALL;
	}
}

/******************************************************************************/
