/***************************************************************************//**
  @file     serial.h
  @brief    Serial communication driver for K64F, using UART
  @author   Group 4: - Oms, Mariano
                     - Solari Raigoso, Agustín
                     - Wickham, Tomás
                     - Vieira, Valentin Ulises
 ******************************************************************************/

#ifndef _SERIAL_H_
#define _SERIAL_H_

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

#define DEVELOPMENT_MODE			1

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

typedef unsigned char uchar_t;

/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/**
 * @brief Initialize the serial module
 * @return Initialization succeed
 */
bool serialInit (void);

/**
 * @brief Send data through serial port
 * @param data Data to be sent
 * @param len Number of bytes to be sent
 * @return Data was sent
 */
bool serialWriteData (uchar_t* data, uint8_t len);

/**
 * @brief Read data from serial port
 * @param len Number of bytes read
 * @return Data read
 */
uchar_t* serialReadData (uint8_t* len);

/**
 * @brief Check if there is data to be written
 * @return true if all data was sent
 */
bool serialWriteStatus (void);

/**
 * @brief Check if there is data to be read
 * @return true if a message was received
 */
bool serialReadStatus (void);

/**
 * @brief Send data through serial port (blocking)
 * @param data Data to be sent
 * @param len Number of bytes to be sent
 * @return Data was sent
 */
bool serialWriteDataBlocking (uchar_t* data, uint8_t len);

/**
 * @brief Read data from serial port (blocking)
 * @return Data read
 */
uchar_t* serialReadDataBlocking (uint8_t* len);

/*******************************************************************************
 ******************************************************************************/

#endif // _SERIAL_H_
