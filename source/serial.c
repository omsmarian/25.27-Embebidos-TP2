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
#define DEBUG_TP					1											// Debugging test points to measure ISR time

#define SERIAL_PORT					UART0_ID


/*******************************************************************************
 * STATIC VARIABLES AND CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

static unsigned char data[QUEUE_MAX_SIZE];


/*******************************************************************************
 *******************************************************************************
						GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

// Main Services ///////////////////////////////////////////////////////////////

bool serialInit (void)
{
	uart_cfg_t config = {9600, UART_MODE_8,
						 UART_PARITY_NONE,
						 UART_STOPS_1,
						 UART_RX_TX_ENABLED,
						 UART_FIFO_RX_TX_ENABLED};

	return uartInit(SERIAL_PORT, config);
}

bool serialSendData (unsigned char* data, uint8_t len)
{
	bool status = false;

	if(uartIsTxMsgComplete(SERIAL_PORT))
		status = len == uartWriteMsg(SERIAL_PORT, data, len);

	return status;
}

unsigned char* serialReadData (uint8_t* len)
{
	if(uartIsRxMsg(SERIAL_PORT))
		*len = uartReadMsg(SERIAL_PORT, data, uartGetRxMsgLength(SERIAL_PORT));

	return data;
}


/******************************************************************************/
