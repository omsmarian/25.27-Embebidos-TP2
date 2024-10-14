/***************************************************************************//**
  @file     uart.h
  @brief    UART Driver for K64F. Blocking, Non-Blocking and using FIFO feature
  @author   Group 4: - Oms, Mariano
                     - Solari Raigoso, Agustín
                     - Wickham, Tomás
                     - Vieira, Valentin Ulises
  @note     Based on the work of Daniel Jacoby
 ******************************************************************************/

#ifndef _UART_H_
#define _UART_H_

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

#define UART_MAX_IDS		UART_CANT_IDS
#define UART_FREQUENCY_HZ	1500U

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

typedef unsigned char uchar_t;
typedef uint8_t uart_register_t;

typedef enum {
	UART_PARITY_EVEN,
	UART_PARITY_ODD,
	UART_PARITY_NONE
} uart_parity_t;

typedef enum {
	UART_MODE_8,
	UART_MODE_9
} uart_mode_t;

typedef enum {
	UART_STOPS_1,
	UART_STOPS_2
} uart_stops_t;

typedef enum {
	UART_RX_ENABLED,
	UART_TX_ENABLED,
	UART_RX_TX_ENABLED
} uart_rx_tx_t;

typedef enum {
	UART_FIFO_DISABLED,
	UART_FIFO_RX_ENABLED,
	UART_FIFO_TX_ENABLED,
	UART_FIFO_RX_TX_ENABLED
} uart_fifo_t;

typedef enum {
	UART_ISR_IRQ,
	UART_ISR_PERIODIC
} uart_isr_t;

typedef enum {
	UART0_ID,
	UART1_ID,
	UART2_ID,
	UART3_ID,
	UART4_ID,
	UART5_ID,

	UART_CANT_IDS
} uart_id_t;

typedef struct {
    uint32_t		baudrate;
	uart_mode_t		mode;
	uart_parity_t	parity;
	uart_stops_t	stops;
	uart_rx_tx_t	RxTx;
	uart_fifo_t		fifo;
	uart_isr_t		isr;
} uart_cfg_t;

/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/**
 * @brief Initialize UART driver
 * @param id UART's number
 * @param config UART's configuration (baudrate, parity, etc.)
 * @return Initialization succeed
*/
bool uartInit (uart_id_t id, uart_cfg_t config);

// Non-Blocking Services ///////////////////////////////////////////////////////

/**
 * @brief Check if a new byte was received
 * @param id UART's number
 * @return A new byte has being received
*/
uint8_t uartIsRxMsg (uart_id_t id);

/**
 * @brief Check how many bytes were received
 * @param id UART's number
 * @return Quantity of received bytes
*/
uint8_t uartGetRxMsgLength (uart_id_t id);

/**
 * @brief Read a received message. Non-Blocking
 * @param id UART's number
 * @param msg Buffer to paste the received bytes
 * @param cant Desired quantity of bytes to be pasted
 * @return Real quantity of pasted bytes
*/
uint8_t uartReadMsg (uart_id_t id, uchar_t* msg, uint8_t cant);

/**
 * @brief Write a message to be transmitted. Non-Blocking
 * @param id UART's number
 * @param msg Buffer with the bytes to be transfered
 * @param cant Desired quantity of bytes to be transfered
 * @return Real quantity of bytes to be transfered
*/
uint8_t uartWriteMsg (uart_id_t id, const uchar_t* msg, uint8_t cant);

/**
 * @brief Check if all bytes were transfered
 * @param id UART's number
 * @return All bytes were transfered
*/
uint8_t uartIsTxMsgComplete (uart_id_t id);

// Blocking Services ///////////////////////////////////////////////////////////

/*******************************************************************************
 ******************************************************************************/

#endif // _UART_H_
