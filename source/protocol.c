/***************************************************************************//**
  @file     protocol.c
  @brief    Data frame packer and unpacker, using angle protocol 'R+123'
  @author   Group 4: - Oms, Mariano
                     - Solari Raigoso, Agustín
                     - Wickham, Tomás
                     - Vieira, Valentin Ulises
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include "macros.h"
#include "protocol.h"
#include "sensor.h"
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

unsigned char* protocolPack (protocol_t* data)
{
	unsigned char* msg = __Num2Chars__(data->angleVal);

	msg[0] = NUM2ASCII(data->angleId);
	msg[1] = (data->angleId >= 0) ? '+' : '-';

	return msg;
}

protocol_t* protocolUnpack (char* msg)
{
	static protocol_t protocol;

	protocol.angleId = msg[0] - '0';
	protocol.angleVal = __Chars2Num__(msg) * ((msg[1] == '+') ? 1 : -1);

	return &protocol;
}

unsigned char* __Num2Chars__ (uint8_t num)
{
	unsigned char chars[PROTOCOL_DIGS] = { 0 };

	for (uint8_t i = 0; i < MAX_DIGS; i++)
	{
		chars[PROTOCOL_DIGS - 1 - i] = NUM2ASCII(num % 10);
		num /= 10;
	}

	return chars;
}

uint8_t __Chars2Num__ (unsigned char* chars)
{
	uint8_t num = 0;

	for (uint8_t i = 0; i < MAX_DIGS; i++)
	{
		num += ASCII2NUM(chars[i + 2]);
		num *= 10;
	}

	return num;
}


/*******************************************************************************
 *******************************************************************************
						LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/


/******************************************************************************/
