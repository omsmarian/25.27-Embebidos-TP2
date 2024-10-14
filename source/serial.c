/***************************************************************************//**
  @file     serial.c
  @brief    Serial communication driver for K64F, using UART
  @author   Group 4: - Oms, Mariano
                     - Solari Raigoso, Agustín
                     - Wickham, Tomás
                     - Vieira, Valentin Ulises
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include "board.h"
#include "uart.h"
#include "pisr.h"
#include "cqueue.h"
#include "timer.h"
#include "serial.h"

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

#define DEVELOPMENT_MODE			1

#define SERIAL_PORT					UART0_ID

/*******************************************************************************
 *******************************************************************************
						GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

// Main Services ///////////////////////////////////////////////////////////////

bool serialInit (void)
{
	uart_cfg_t config = {9600,
						 UART_MODE_8,
						 UART_PARITY_NONE,
						 UART_STOPS_1,
						 UART_RX_TX_ENABLED,
						 UART_FIFO_RX_TX_ENABLED,
						 UART_ISR_PERIODIC};

	return uartInit(SERIAL_PORT, config);
}

bool serialWriteData (uchar_t* data, uint8_t len)
{
	return len == uartWriteMsg(SERIAL_PORT, data, len);
}

uchar_t* serialReadData (uint8_t* len)
{
	static unsigned char data[QUEUE_MAX_SIZE];

	*len = uartReadMsg(SERIAL_PORT, data, uartGetRxMsgLength(SERIAL_PORT));

	return data;
}

bool serialWriteStatus (void)
{
	return uartIsTxMsgComplete(SERIAL_PORT);
}

bool serialReadStatus (void)
{
	return uartIsRxMsg(SERIAL_PORT);
}

bool serialWriteDataBlocking (uchar_t* data, uint8_t len)
{
	uint8_t aux = uartWriteMsg(SERIAL_PORT, data, len);

	while (!uartIsTxMsgComplete(SERIAL_PORT));

	return len == aux;
}

uchar_t* serialReadDataBlocking (uint8_t* len)
{
	static unsigned char data[QUEUE_MAX_SIZE];

	*len = 0;
	while (!uartIsRxMsg(SERIAL_PORT))
		*len += uartReadMsg(SERIAL_PORT, data, uartGetRxMsgLength(SERIAL_PORT));

	return data;
}

/******************************************************************************/
