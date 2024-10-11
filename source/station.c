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
//#include "can.h"
#include "timer.h"
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
//	canInit();		// Initialize CAN
	timerInit();	// Initialize Timer
}

bool stationStatus (station_id_t station)
{
//	return canStatus(station);
	return true;
}

void stationSend (station_id_t station, uint8_t* data)
{
//	canSend(station, data);
}

void stationSendAll (uint8_t* data)
{
	for (uint8_t i = 0; i < STATIONS_CANT; i++)
	{
//		canSend(i, data);
	}
}

void stationReceive (station_id_t station, uint8_t* data)
{
//	canReceive(station, data);
}

void stationReceiveAll (uint8_t* data)
{
	for (uint8_t i = 0; i < STATIONS_CANT; i++)
	{
//		canReceive(i, data);
	}
}

//void stationSetCallback (stations_id_t station, can_callback_t callback)
//{
////	canSetCallback(station, callback);
//}
//
//void stationSetCallbacksAll (can_callback_t callback)
//{
//	for (uint8_t i = 0; i < STATIONS_CANT; i++)
//	{
////		canSetCallback(i, callback);
//	}
//}


/*******************************************************************************
 *******************************************************************************
						LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/


/******************************************************************************/
