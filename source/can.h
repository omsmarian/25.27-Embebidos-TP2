/*
 * can.h
 *
 *  Created on: 1 oct 2024
 *      Author: asolari
 */

#ifndef CAN_H_
#define CAN_H_

#include "stdbool.h"
#include "stdint.h"
#include "stdlib.h"
#include <MK64F12.h>
#include "gpio.h"
#include <board.h>


#define PIN_CAN0_TX PORTNUM2PIN(PB,18)
#define PIN_CAN0_RX PORTNUM2PIN(PB,19)

#define BUFFER_SIZE 10 // Define el tamaño del buffer circular
#define FRAME_SIZE 8 // Define el tamaño de los frames
#define TX_BUFFER 8

#define BASE_ID 0x100
#define MASK_ID 0b11100000000
#define MY_BOARD_ID (BASE_ID + 4)


typedef struct{
	uint32_t ID;
	uint32_t length;
	union{
		struct{
			uint32_t dataWord0;
			uint32_t dataWord1;
		};
		struct{
			uint8_t dataByte0;
			uint8_t dataByte1;
			uint8_t dataByte2;
			uint8_t dataByte3;
			uint8_t dataByte4;
			uint8_t dataByte5;
			uint8_t dataByte6;
			uint8_t dataByte7;
		};
		uint8_t data[FRAME_SIZE];
	};
}CAN_DataFrame;


typedef enum {	CAN_SUCCESS,
				CAN_FAILED,
				CAN_ERROR,
				CAN_NON_STD_BAUD,
				CAN_RX_BUSY,
				CAN_RX_OVERFLOW
				}CAN_Status;

typedef enum{RX_INACTIVE = 0b0000,
			 RX_EMPTY    = 0b0100,
		     RX_FULL     = 0b0010,
		     RX_OVERRUN  = 0b0011,
		     RX_RANSWER  = 0b1010,
			 RX_BUSY     = 0b0001
			}RxMBCode;

typedef enum{TX_INACTIVE = 0b1000,
			 TX_ABORT    = 0b1001,
			 TX_DATA     = 0b1100, // RTR = 0
			 TX_REMOTE   = 0b1100, // RTR = 1
			 TX_TANSWER  = 0b1110
			}TxMBCode;

typedef struct
{
    CAN_DataFrame buffer[BUFFER_SIZE]; // Buffer para almacenar los frames
    int start; // Índice del primer elemento en el buffer
    int end; // Índice donde se añadirá el próximo elemento
} CircularBuffer_t;


/**
 * @brief Circular buffer to store received frames.
 */
extern CircularBuffer_t cb;

/////////////////////////////////////////////////////////////////////////////////
//                         Global function prototypes                          //
/////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Circular buffer to store received frames.
 */
extern CircularBuffer_t cb;

/*
 * @brief Initialize CAN module
 * @param config Pointer to a struct containing module configuration.
 * @param clockHz Protocol Engine clock source frequency in Hz.
 */
CAN_Status CAN_Init(void);


/**
 * @brief Enable the CAN module.
 * @param base CAN peripheral base address.
 */
void CAN_Enable();

/**
 * @brief Disable the CAN module.
 * @param base CAN peripheral base address..
 */
void CAN_Disable();

/**
 * @brief Freeze the CAN module.
 * @param freeze True to freeze, false to unfreeze.
 */
void CAN_Freeze( bool freeze);


/**
 * @brief Configure a message buffer for receiving.
 * @param base CAN peripheral base address
 * @param index Number of message buffer.
 * @param config Pointer to struct containing MB Rx configuration
 */
void  CAN_ConfigureRxMB(uint8_t index,uint32_t ID);


/**
 * @breif Enable interrupt of a message buffer and attach a callback.
 * @param base CAN peripheral base address
 * @param index Number of message buffer.
 * @param config Pointer to struct containing MB Rx configuration
 */
void CAN_EnableMbInterrupts	(uint8_t index);


/**
 * @brief Sets the global mask for CAN message buffers during matching process.
 *
 * Note that the configuration is only effective when the Rx individual mask
 * is disabled when calling CAN_Init().
 *
 */
void CAN_SetRxMbGlobalMask	(uint32_t 	mask);
void CAN_SetRxIndividualMask (uint8_t index, uint32_t 	mask);

/**
 * @brief Poll the flag status of a message buffer.
 * @param base CAN peripheral base address
 * @param index Number of message buffer.
 * @return True if a message was sent/received, false if not.
 */
bool CAN_GetMbStatusFlag(uint8_t index);

/**
 * @brief Clear the interrupt flag of the indicated message buffer
 * @param base CAN peripheral base address
 * @param index Number of message buffer.
 */
void CAN_ClearMbStatusFlag(uint8_t index);


/*
 * @brief Read a frame from a message buffer.
 * @param base CAN peripheral base address
 * @param index Number of message buffer to read.
 * @param frame Pointer to frame to store received data.
 * @return
 */
CAN_Status CAN_ReadRxMB(uint8_t index, CAN_DataFrame * frame);

/*
 * @brief Write a frame to a message buffer to be sent.
 * @param base CAN peripheral base address
 * @param index Number of message buffer to write.
 * @param frame Pointer to frame to be sent.
 * @return
 */
CAN_Status CAN_WriteTxMB(uint8_t index, CAN_DataFrame * frame);


/////////////////////////////////////////////////////////////////////////////////
// Functions for circular buffer
void initBuffer(CircularBuffer_t *cb);
void addToBuffer(CircularBuffer_t *cb, CAN_DataFrame frame);
CAN_DataFrame getFromBuffer(CircularBuffer_t *cb);


#endif /* CAN_H_ */
