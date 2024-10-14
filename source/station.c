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

/*******************************************************************************
 *******************************************************************************
						GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

// Main Services ///////////////////////////////////////////////////////////////

bool stationInit (void)
{
	bool status = false;

	if (CAN_Init() == CAN_SUCCESS)
	{
		for(uint8_t i = 0; i <= STATIONS_CANT; i++)
		{
			CAN_ConfigureRxMB(i,BASE_ID+i);
			CAN_SetRxIndividualMask( i, MASK_ID);
			CAN_EnableMbInterrupts(i);									// Enable the interrupt
		}

		status = true;
	}

	return status;
}

void stationSend (station_t* station)
{
	CAN_DataFrame frame = { .ID = STATION_ID, .length = station->len };

	for (uint8_t i = 0; i < FRAME_SIZE; i++)
		frame.data[i] = 0;														//Clean frame data

	for (uint8_t i = 0; (i < FRAME_SIZE) && (i < frame.length); i++)
		frame.data[i] = station->data[i];

	CAN_WriteTxMB(TX_BUFFER, &frame);													// Send frame
}

void stationReceive (station_t* station)
{
    CAN_DataFrame frame = getFromBuffer(&cb);									// Read frame from circular buffer

    if ((frame.ID != 0) || (frame.length != 0))									// If the frame is empty, the buffer was empty
    {
    	for (uint8_t i = 0; (i < station->len) && (i < frame.length); i++)
			station->data[i] = frame.data[i];

		station->len = frame.length;
		station->id = frame.ID - STATION_BASE_ID;
    }
    else
    	station->len = 0;
}

/******************************************************************************/
