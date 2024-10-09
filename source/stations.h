/***************************************************************************//**
  @file     stations.h
  @brief    K64F stations communications handler, using CAN bus
  @author   Group 4: - Oms, Mariano
                     - Solari Raigoso, Agustín
                     - Wickham, Tomás
                     - Vieira, Valentin Ulises
 ******************************************************************************/

#ifndef _STATIONS_H_
#define _STATIONS_H_

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>


/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/


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
} stations_id_t;


/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/**
 * @brief Initialize the stations module
 */
void stationsInit(void);

/**
 * @brief Send data to a station
 * @param station Station to send data to
 * @param data Data to send
 */
void stationsSend(stations_id_t station, uint8_t data);

/**
 * @brief Send data to all stations
 * @param data Data to send
 */
void stationsSendAll(uint8_t data);

/**
 * @brief Receive data from a station
 * @param station Station to receive data from
 * @param data Data received
 */
void stationsReceive(stations_id_t station, uint8_t data);

/**
 * @brief Receive data from all stations
 * @param data Data received
 */
void stationsReceiveAll(uint8_t data);

/**
 * @brief Set the callback for a station
 * @param station Station to set the callback for
 * @param callback Callback to set
 */
void stationsSetCallback(stations_id_t station, void (*callback)(uint8_t));

/**
 * @brief Set the callback for all stations
 * @param callback Callback to set
 */
void stationsSetCallbackAll(void (*callback)(uint8_t));


/*******************************************************************************
 ******************************************************************************/

#endif // _STATIONS_H_
