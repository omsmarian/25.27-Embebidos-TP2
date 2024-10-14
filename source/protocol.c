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

#include <stdio.h>

#include "macros.h"
#include "protocol.h"
#include "sensor.h"
#include "station.h"

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

#define DEVELOPMENT_MODE			1

/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/

/**
 * @brief Convert a number to a string (no terminator)
 * @param num Number to convert
 * @param chars String to store the number
 * @return Number of digits in the number (string length)
 */
uint8_t __Num2Chars__ (const uint16_t num, uchar_t* chars);

/**
 * @brief Convert a string (no terminator) to a number
 * @param chars String to convert
 * @param len Length of the string (number of digits)
 * @return Number converted
 */
int16_t __Chars2Num__ (uchar_t* const chars, const int8_t len);

/*******************************************************************************
 *******************************************************************************
						GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

uint8_t protocolPack (protocol_t* const data, uchar_t chars[PROTOCOL_DIGS])
{
	uint8_t len, index = 1;

	chars[0] = data->id;

	if (data->val < 0)
	{
		chars[1] = '-';
		index = 2;
	}

	len = __Num2Chars__(ABS(data->val), chars + index);
	if(!len) { index = 0; }

	return len + index;
}

protocol_t* protocolUnpack (uchar_t* const msg, const uint8_t len)
{
	static protocol_t data;
	uint8_t index = 1;

	data = (protocol_t){ 0, 0 };
	if (len > 1)
	{
		if (msg[1] == '+' || msg[1] == '-') { index = 2; }
		if (len - index <= MAX_DIGS)
		{
			data.id = msg[0];
			data.val = __Chars2Num__(msg + index, len - index) * ((msg[1] == '-') ? -1 : 1);
		}
	}

	return &data;
}

/*******************************************************************************
 *******************************************************************************
						LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

uint8_t __Num2Chars__ (const uint16_t num, uchar_t* chars)
{
	int16_t aux = num;
	uint8_t len = 0;

	do { aux /= 10; }
	while ((++len < MAX_DIGS) && aux);

	for (uint8_t i = 0; i < len; i++)
		chars[i] = 0;

	if (!aux)
	{
		aux = num;
		for (uint8_t i = 0; i < len; i++)
		{
			chars[len - 1 - i] = NUM2ASCII(aux % 10);
			aux /= 10;
		}
	}
	else { len = 0; }

	return len;
}

int16_t __Chars2Num__ (uchar_t* const chars, const int8_t len)
{
	int16_t num = 0;

	for (uint8_t i = 0; (i < len) && (i < MAX_DIGS); i++)
	{
		num *= 10;
		num += ASCII2NUM(chars[i]);
	}

	return num;
}

/******************************************************************************/
