/*
 * can.h
 *
 *  Created on: 1 oct 2024
 *      Author: asolari
 */

#ifndef CAN_H_
#define CAN_H_

#include "stdbool.h"
#include "MK64F12.h"
#include "fsl_port.h"
#include "fsl_debug_console.h"

// Define the message buffer
#define RX_MB 10
#define TX_MB 11

//CAN message structure
typedef struct {
	uint16_t id :11; // 11 bits for CAN ID
	uint8_t len :4; // 4 bits for Data Length
	uint8_t data[8]; // 0 - 8 bytes for Data
} can_msg_t;

bool can_init(void);
bool can_rx_msg(can_msg_t *msg);
bool can_tx_msg(can_msg_t *msg);;



#endif /* CAN_H_ */
