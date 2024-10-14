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
#define DEBUG_TP					1											// Debugging test points to measure ISR time


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

bool serialWriteStatus (void);
bool serialReadStatus (void);

/*******************************************************************************
 ******************************************************************************/

#endif // _SERIAL_H_
