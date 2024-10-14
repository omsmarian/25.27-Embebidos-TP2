/***************************************************************************//**
  @file     station.h
  @brief    K64F station communications handler, using CAN bus
  @author   Group 4: - Oms, Mariano
                     - Solari Raigoso, Agustín
                     - Wickham, Tomás
                     - Vieira, Valentin Ulises
 ******************************************************************************/

#ifndef _STATION_H_
#define _STATION_H_

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

#define DEVELOPMENT_MODE			1
#define DEBUG_TP					1											// Debugging Test Points to measure ISR time

#define GN							4											// Group number
#define STATION_BASE_ID				0x100U
#define STATION_ID					(STATION_BASE_ID + GN)
#define STATIONS_MASK				0b000100000111

#define DATA_MAX_SIZE				8											// Complete according to expected data size

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

typedef enum {
	STATION_1,
	STATION_2,
	STATION_3,
	STATION_4,
	STATION_5,

	STATIONS_CANT
} station_id_t;

typedef unsigned char uchar_t;

typedef struct {
	station_id_t id;
	uchar_t* data;
	uint8_t len;
} station_t;

/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/**
 * @brief Initialize the stations module
 * @return Initialization succeed
 */
bool stationInit (void);

/**
 * @brief Send data to all station
 * @param station Data to send (including Id and message length)
 */
void stationSend (station_t* station);

/**
 * @brief Receive data from a station
 * @param station Place to store received data (including Id and message length)
 */
void stationReceive (station_t* station);

/*******************************************************************************
 ******************************************************************************/

#endif // _STATION_H_
