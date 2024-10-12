/***************************************************************************//**
  @file     protocol.h
  @brief    Data frame packer and unpacker, using angle protocol 'R+123'
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

#define MAX_DIGS		3														// Maximum digits for the angle
#define PROTOCOL_DIGS	(MAX_DIGS + 2)											// Protocol size: id + sign + val

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

typedef struct
{
	// station_id_t stationId;
	sensor_axis_t angleId;
	angle_t angleVal;
} protocol_t;

typedef unsigned char uchar_t;

// typedef struct
// {
// 	uchar_t id;
// 	uchar_t sign;
// 	uchar_t val[MAX_DIGS];
// } protocol_msg_t;

// typedef struct
// {
// 	uchar_t id;
// 	uchar_t sign;
// 	int16_t val;
// } protocol_msg2_t;

// typedef struct
// {
// 	sensor_axis_t angleId;
// 	uchar_t angleIdChar;
// } angle_id_t;

//const uchar_t id2Chars[] = { 'R', 'C', 'O' };									// Roll (Rolido), Pitch (Cabeceo), Yaw (Orientacion)

// angle_id_t angleIdChars[] = { { ROLL, 'R' }, { PITCH, 'C' }, { YAW, 'O' } };

/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/**
 * @brief Pack data into a message using the protocol
 * @param data Data to pack
 * @return Packed message
 */
uchar_t* protocolPack (protocol_t* data);

/**
 * @brief Unpack a message using the protocol
 * @param msg Message to unpack
 * @return Unpacked data
 */
protocol_t* protocolUnpack (uchar_t* msg);

/*******************************************************************************
 ******************************************************************************/

#endif // _PROTOCOL_H_
