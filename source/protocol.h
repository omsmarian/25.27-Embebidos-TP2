/***************************************************************************//**
  @file     protocol.h
  @brief    Data frame packer and unpacker, using protocol '{Id}[ValSign]{Val}'
  @author   Group 4: - Oms, Mariano
                     - Solari Raigoso, Agustín
                     - Wickham, Tomás
                     - Vieira, Valentin Ulises
 ******************************************************************************/

#ifndef _PROTOCOL_H_
#define _PROTOCOL_H_

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include <stdint.h>

#include "sensor.h"

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

// Protocol configuration //////////////////////////////////////////////////////

#define MAX_DIGS		3														// Maximum digits for the angle
#define PROTOCOL_DIGS	(MAX_DIGS + 2)											// Protocol size: id + sign + val

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

// typedef struct {
// 	// station_id_t stationId;
// 	sensor_axis_t angleId;
// 	angle_t angleVal;
// } protocol_t;

// Data types for the protocol /////////////////////////////////////////////////

typedef unsigned char uchar_t;
typedef uchar_t protocol_id_t;
typedef int16_t protocol_val_t;

/**
 * @brief Protocol data structure
 * @param id Protocol ID, not processed when packing
 * @param val Protocol value, converted to string before packing
 */
typedef struct {
	protocol_id_t id;
	protocol_val_t val;
} protocol_t;

/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/**
 * @brief Pack data into a message using the protocol
 * @param data Data to pack
 * @param msg Place to store packed message
 * @return Message length
 */
uint8_t protocolPack (protocol_t* const data, uchar_t msg[PROTOCOL_DIGS]);

/**
 * @brief Unpack a message using the protocol
 * @param msg Message to unpack
 * @param len Message length
 * @return Unpacked data
 */
protocol_t* protocolUnpack (uchar_t* const msg, const uint8_t len);

/*******************************************************************************
 ******************************************************************************/

#endif // _PROTOCOL_H_
