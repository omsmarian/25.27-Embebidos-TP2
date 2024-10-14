/***************************************************************************//**
  @file     PDRV_I2C.h
  @brief    +Descripcion del archivo+
  @author   matia
 ******************************************************************************/

#ifndef PDRV_I2C_H_
#define PDRV_I2C_H_

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include <stdint.h>
#include "MK64F12_features.h"

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

#define I2C_NUMBER_OF_DEVICES FSL_FEATURE_SOC_I2C_COUNT

#define I2C_RESTART 1<<8
#define I2C_READ    2<<8

#define I2C_ADDR_W(addr)	(addr<<1 | 0x0)
#define I2C_ADDR_R(addr)	(addr<<1 | 0x1)


/* Channel status definitions */
#define I2C_AVAILABLE 	0
#define I2C_BUSY 		1
#define I2C_ERROR 		2

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

typedef enum{
	I2C_SPEED_156250HZ = 0x14,
	I2C_SPEED_97656HZ = 0x17,
	I2C_SPEED_78125HZ = 0x1D,
	I2C_SPEED_65104HZ = 0x1E,
	I2C_SPEED_48828HZ = 0x23,
	I2C_SPEED_32522HZ = 0x26,
	I2C_SPEED_24414HZ = 0x2B,
	I2C_SPEED_16276HZ = 0x2E,
	I2C_SPEED_12207HZ = 0x33,
	I2C_SPEED_8138HZ = 0x39,
	I2C_SPEED_4069HZ = 0x3E,
	I2C_SPEED_3255HZ = 0x3F
}i2c_speed_t;


/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

// +ej: extern unsigned int anio_actual;+


/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

void initI2C(uint8_t, i2c_speed_t);

/*
  Sends a command/data sequence that can include restarts, writes and reads. Every transmission begins with a START,
  and ends with a STOP so you do not have to specify that.

  sequence is the I2C operation sequence that should be performed. It can include any number of writes, restarts and
  reads. Note that the sequence is composed of uint16_t, not uint8_t. This is because we have to support out-of-band
  signalling of I2C_RESTART and I2C_READ operations, while still passing through 8-bit data.

  sequence_length is the number of sequence elements (not bytes). Sequences of arbitrary length are supported. The
  minimum sequence length is (rather obviously) 2.

  received_data should point to a buffer that can hold as many bytes as there are I2C_READ operations in the
  sequence. If there are no reads, 0 can be passed, as this parameter will not be used.

  callback_fn is a pointer to a function that will get called upon successful completion of the entire sequence. If 0 is
  supplied, no function will be called. Note that the function will be called from an interrupt handler, so it should do
  the absolute minimum possible (such as enqueue an event to be processed later, set a flag, exit sleep mode, etc.)

  user_data is a pointer that will be passed to the callback_fn.
*/
// No llamar a la funcion demasiado seguido!
int32_t i2cSendSequence(uint8_t i2c_num, uint16_t *sequence, uint32_t sequence_len, uint8_t* received_data,
						  void (*callback_fn)(void*), void *user_data);

uint8_t i2cGetStatus(uint8_t i2c_num);

#endif /* I2C_H */
