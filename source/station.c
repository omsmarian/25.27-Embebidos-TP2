/***************************************************************************//**
  @file     station.c
  @brief    K64F stations communications handler, using CAN bus
  @author   Group 4: - Oms, Mariano
					 - Solari Raigoso, Agustín
					 - Wickham, Tomás
					 - Vieira, Valentin Ulises
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include "board.h"
#include "can.h"
#include "station.h"

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

#define DEVELOPMENT_MODE			1
#define DEBUG_TP					1											// Debugging Test Points to measure ISR time

/*******************************************************************************
 *******************************************************************************
						GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

// Main Services ///////////////////////////////////////////////////////////////

void stationInit (void)
{
	bool status = false;

	if (CAN_Init())
	{
		for(uint8_t i = 0; i < STATIONS_CANT; i++)
		{
			CAN_ConfigureRxMB(i, STATION_BASE_ID + i);							// Initialize reception buffers
			CAN_EnableMbInterrupts(i);											// Enable the interrupt
		}

		CAN_ConfigureTxMB(STATION_ID);											// Initialize transmission buffer

		status = true;
	}

	return status;
}

bool stationStatus (station_id_t station)
{
//	return canStatus(station);
	return true;
}

bool stationStatusAll (void)
{
//	return canStatusAll();
	return true;
}

void stationSendAll (uchar_t* data, uint8_t len)
{
	CAN_DataFrame frame = { .ID = STATION_ID, .length = len };

	for (uint8_t i = 0; i < FRAME_SIZE; i++)
		frame.data[i] = 0;														//Clean frame data

	for (uint8_t i = 0; (i < FRAME_SIZE) && (i < len); i++)
		frame.data[i] = data[i];

	CAN_WriteTxMB(STATION_ID, &frame);											// Send frame
}

void stationSend (station_id_t station, uchar_t* data, uint8_t len)
{
	// canSend(station, data);
}

station_id_t stationReceive (uchar_t* data)
{
    CAN_DataFrame frame = getFromBuffer(&cb);									// Read frame from circular buffer
	// CAN_DataFrame frame;
	// CAN_ReadRxMB(STATION_ID, &frame);										// Read frame from reception buffer

    if ((frame.ID != 0) || (frame.length != 0))									// If the frame is empty, the buffer was empty
		for (uint8_t i = 0; i < frame.length; i++)
			data[i] = frame.data[i];
	else
		frame.ID = STATION_BASE_ID + STATIONS_CANT;

    return frame.ID - STATION_BASE_ID;
}

void stationReceiveAll (uchar_t** data)
{
	// for (uint8_t i = 0; i < STATIONS_CANT; i++)
	// 	stationReceive(i, data[i]);
}

/*******************************************************************************
 *******************************************************************************
						LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/


/******************************************************************************/
