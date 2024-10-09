/***************************************************************************//**
  @file     I2C.h
  @brief    +Descripcion del archivo+
  @author   Mariano Oms
 ******************************************************************************/

#ifndef _I2C_H_
#define _I2C_H_

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/


#include <stdint.h>

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/


/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

typedef enum{
	I2C_Read,
	I2C_Write,
}I2C_Mode_t;

typedef enum {
  I2C_Idle,			    // Idle
  I2C_Busy,	        // Currently transmitting or receiving
  I2C_Done,		      // Already finished communication
  I2C_Error			    // Some error occurred
} I2C_Status_t;

typedef enum {I2C0_M, I2C1_M, I2C2_M, I2C_COUNT} I2C_Module_t;

typedef uint8_t I2C_Address_t;

typedef struct{
	I2C_Status_t status;
	I2C_Mode_t mode;
	uint8_t * sequence_arr;
	size_t sequence_size;
	//uint8_t * write_arr;
	//size_t write_size;
	I2C_Address_t slave_address;
  I2C_Address_t reg_address;
	size_t index;
	bool reg_address_flag;
	//size_t W_index;
}I2C_Object_t;

/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

void I2C_Init(I2C_Module_t module);

I2C_Status_t I2C_Transmit(I2C_Module_t module, uint8_t * sequence_arr, size_t sequence_size, 
                          I2C_Address_t slave_address, I2C_Mode_t mode);


/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/


/*******************************************************************************
 ******************************************************************************/

#endif
