/***************************************************************************//**
  @file     sensor.h
  @brief    FXOS8700CQ accelerometer and magnetometer driver for K64F
  @author   Group 4
 ******************************************************************************/

#ifndef _SENSOR_H_
#define _SENSOR_H_

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>


/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

// #define FXOS8700CQ_STATUS	0x00
// #define FXOS8700CQ_OUT_X_MSB	0x01
// #define FXOS8700CQ_OUT_X_LSB	0x02
// #define FXOS8700CQ_OUT_Y_MSB	0x03
// #define FXOS8700CQ_OUT_Y_LSB	0x04
// #define FXOS8700CQ_OUT_Z_MSB	0x05
// #define FXOS8700CQ_OUT_Z_LSB	0x06

// #define FXOS8700CQ_XYZ_DATA_CFG	0x0E
// #define FXOS8700CQ_CTRL_REG1	0x2A
// #define FXOS8700CQ_CTRL_REG2	0x2B
// #define FXOS8700CQ_CTRL_REG3	0x2C
// #define FXOS8700CQ_CTRL_REG4	0x2D
// #define FXOS8700CQ_CTRL_REG5	0x2E

// FXOS8700CQ I2C address
#define FXOS8700CQ_SLAVE_ADDR	0x1D											// With pins SA0 = 0, SA1 = 0

// FXOS8700CQ internal register addresses
#define FXOS8700CQ_STATUS		0x00
#define FXOS8700CQ_WHOAMI		0x0D
#define FXOS8700CQ_XYZ_DATA_CFG	0x0E
#define FXOS8700CQ_CTRL_REG1	0x2A
#define FXOS8700CQ_M_CTRL_REG1	0x5B
#define FXOS8700CQ_M_CTRL_REG2	0x5C
#define FXOS8700CQ_F_SETUP		0x09

#define FXOS8700CQ_WHOAMI_VAL	0xC7											// Production devices

// Number of bytes to be read from the FXOS8700CQ
#define FXOS8700CQ_READ_LEN		13												// Status + 6 channels (13 bytes)

#define FXOS8700CQ_ACCEL_SENS	0.000244f										// Sensitivity in g/LSB
#define FXOS8700CQ_MAGN_SENS	0.1f											// Sensitivity in uT/LSB

#define FXOS8700CQ_ACCEL_RANGE	4												// Accelerometer range in g
#define FXOS8700CQ_MAGN_RANGE	120												// Magnetometer range in uT

#define FXOS8700CQ_ACCEL_LSB	(FXOS8700CQ_ACCEL_SENS * FXOS8700CQ_ACCEL_RANGE)	// Accelerometer LSB in g
#define FXOS8700CQ_MAGN_LSB		(FXOS8700CQ_MAGN_SENS * FXOS8700CQ_MAGN_RANGE)	// Magnetometer LSB in uT

#define FXOS8700CQ_ACCEL_LSB_2G	0.000244f										// Accelerometer LSB in g for 2g range
#define FXOS8700CQ_ACCEL_LSB_4G	0.000488f										// Accelerometer LSB in g for 4g range
#define FXOS8700CQ_ACCEL_LSB_8G	0.000976f										// Accelerometer LSB in g for 8g range

#define ANGLE_THRESHOLD			2												// Threshold for angle change detection


/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

typedef uint8_t byte_t;
typedef uint8_t angle_t;

typedef struct
{
	angle_t roll,
	angle_t pitch,
	angle_t yaw
} sensor_t;

typedef enum
{
	ROLL,
	PITCH,
	YAW
} sensor_axis_t;


/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/**
 * @brief  Get roll, pitch and yaw angles from the sensor
 * @return Sensor data
 */
sensor_t * sensorGetData (void);


/*******************************************************************************
 ******************************************************************************/

#endif // _SENSOR_H_
