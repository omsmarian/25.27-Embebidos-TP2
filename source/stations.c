/***************************************************************************//**
  @file     stations.c
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
#include "timer.h"
#include "stations.h"


/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

#define DEVELOPMENT_MODE			1

#define DEBUG_TP					1											// Debugging Test Points to measure ISR time


/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/


/*******************************************************************************
 * STATIC VARIABLES AND CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/


/*******************************************************************************
 *******************************************************************************
						GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

// Main Services ///////////////////////////////////////////////////////////////

void stationsInit(void)
{
	canInit();		// Initialize CAN
	timerInit();	// Initialize Timer
}

void stationsSend(stations_id_t station, uint8_t data)
{
	canSend(station, data);
}

void stationsSendAll(uint8_t data)
{
	for (uint8_t i = 0; i < STATIONS_CANT; i++)
	{
		canSend(i, data);
	}
}

void stationsReceive(stations_id_t station, uint8_t data)
{
	canReceive(station, data);
}

void stationsReceiveAll(uint8_t data)
{
	for (uint8_t i = 0; i < STATIONS_CANT; i++)
	{
		canReceive(i, data);
	}
}


/*******************************************************************************
 *******************************************************************************
						LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

// ISR Functions ///////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////


/******************************************************************************/
