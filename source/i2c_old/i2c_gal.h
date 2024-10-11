/*
 * i2c.h
 */

#ifndef I2C_H_
#define I2C_H_


/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include <stdint.h>
#include <stddef.h>


/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/


/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/


typedef enum {I2C0_M, I2C1_M, I2C2_M, I2C_M_Count} I2C_Module_t;

/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/
typedef enum{
	I2C_Read,
	I2C_Write,
}I2C_Mode_t;


typedef enum {
  I2C_Idle,			// Idle
  I2C_Busy,	// Currently transmitting or receiving
  I2C_Done,		// Already finished communication
  I2C_Error			// Some error occurred
} I2C_Status_t;


typedef uint8_t I2C_Address_t;

typedef struct{
	I2C_Status_t status;
	I2C_Mode_t mode;
	uint8_t * read_buffer;
	size_t read_size;
	uint8_t * write_buffer;
	size_t write_size;
	I2C_Address_t slave_address;
	size_t R_index;
	size_t W_index;
}I2C_Object_t;


/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/


/**
 * @brief Initialize i2c driver
 * @param id i2c's number
 * @param config i2c's configuration (baudrate, parity, etc.)
*/
void I2C_InitModule (I2C_Module_t module);


I2C_Status_t I2C_InitObject(I2C_Module_t module, uint8_t * read_buffer, size_t read_size, uint8_t * write_buffer,
							size_t write_size, I2C_Address_t slave_address);


I2C_Status_t i2cTransactionState(I2C_Module_t module);




/*******************************************************************************
 ******************************************************************************/


#endif /* I2C_H_ */
