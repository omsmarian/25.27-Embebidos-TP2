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

typedef enum {
	SINGLE,
	PERIODIC
} station_mode_t;


/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/**
 * @brief Initialize the stations module
 */
void stationInit (void);

/**
 * @brief Check the status of a station
 * @param station Station to check
 * @return true if the station has sent a message
 */
bool stationStatus (station_id_t station);

/**
 * @brief Check the status of all stations
 * @return true if any station has sent a message
 */
bool stationStatusAll (void);

/**
 * @brief Send data to a station
 * @param station Station to send data to
 * @param data Data to send
 */
void stationSend (station_id_t station, uint8_t* data);

/**
 * @brief Send data to all stations
 * @param data Data to send
 */
void stationSendAll (uint8_t* data);

/**
 * @brief Receive data from a station
 * @param station Station to receive data from
 * @param data Data received
 */
void stationReceive (station_id_t station, uint8_t* data);

/**
 * @brief Receive data from all stations
 * @param data Data received
 */
void stationReceiveAll (uint8_t* data);

///**
// * @brief Set the callback for a station
// * @param station Station to set the callback for
// * @param callback Callback to set
// */
//void stationSetCallback (station_id_t station, void (*callback)(uint8_t*));
//
///**
// * @brief Set the callback for all stations
// * @param callback Callback to set
// */
//void stationSetCallbackAll (void (*callback)(uint8_t*));


/*******************************************************************************
 ******************************************************************************/

#endif // _STATION_H_
