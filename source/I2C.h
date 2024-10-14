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
#include <stdbool.h>

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
  I2C_Busy,	        	// Currently transmitting or receiving
  I2C_Done,		      	// Already finished communication
  I2C_Error			    // Some error occurred
} I2C_Status_t;

typedef enum {I2C0_M, I2C1_M, I2C2_M, I2C_COUNT} I2C_Module_t;

typedef uint8_t I2C_Address_t;

typedef struct{
	I2C_Status_t status;
	I2C_Mode_t mode;
	uint8_t * sequence_arr;
	uint8_t sequence_size;
	I2C_Address_t slave_address;
  	I2C_Address_t reg_address;
	uint8_t index;
	bool reg_address_flag;
	bool repeated_start;
}I2C_Object_t;

/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/


/**
 * @brief Initialize I2C module
 * @param module I2C module to be initialized
 */
void I2C_Init(I2C_Module_t module);

/**
 * @brief Starts the transmition of data through I2C
 * @param module I2C module to be used
 * @param sequence_arr Array of data to be transmitted or array to store the received data
 * @param sequence_size Size of the array
 * @param slave_address Address of the slave device
 * @param reg_address Address of the register to be written or read
 * @param mode Mode of the transmission (I2C_Read or I2C_Write)
 * @return Status of the I2C module
 */
I2C_Status_t I2C_Transmit(I2C_Module_t module, uint8_t * sequence_arr, uint8_t sequence_size, 
                    I2C_Address_t slave_address, I2C_Address_t reg_address, I2C_Mode_t mode);

/**
 * @brief Gets the current state of the I2C module
 * 
 * @param module I2C module to get the status from 
 * @return Status of the I2C module 
 */
I2C_Status_t I2C_GetStatus(I2C_Module_t module);

#endif
