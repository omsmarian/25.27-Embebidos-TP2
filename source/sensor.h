/***************************************************************************//**
  @file     sensor.h
  @brief    FXOS8700CQ accelerometer and magnetometer driver for K64F
  @author   Group 4: - Oms, Mariano
                     - Solari Raigoso, Agustín
                     - Wickham, Tomás
                     - Vieira, Valentin Ulises
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

#define FXOS8700CQ_ACCEL_SENS	0.000244f										// Sensitivity in g/LSB
#define FXOS8700CQ_MAGN_SENS	0.1f											// Sensitivity in uT/LSB

#define FXOS8700CQ_ACCEL_RANGE	4												// Accelerometer range in g
#define FXOS8700CQ_MAGN_RANGE	120												// Magnetometer range in uT

#define FXOS8700CQ_ACCEL_LSB	(FXOS8700CQ_ACCEL_SENS * FXOS8700CQ_ACCEL_RANGE)// Accelerometer LSB in g
#define FXOS8700CQ_MAGN_LSB		(FXOS8700CQ_MAGN_SENS * FXOS8700CQ_MAGN_RANGE)	// Magnetometer LSB in uT

#define FXOS8700CQ_ACCEL_LSB_2G	0.000244f										// Accelerometer LSB in g for 2g range
#define FXOS8700CQ_ACCEL_LSB_4G	0.000488f										// Accelerometer LSB in g for 4g range
#define FXOS8700CQ_ACCEL_LSB_8G	0.000976f										// Accelerometer LSB in g for 8g range

#define FXOS8700CQ_OUT_LEN		2												// Bytes
#define FXOS8700CQ_M_OUT_LEN	2
#define FXOS8700CQ_AXIS_CANT	3												// X, Y, Z
#define FXOS8700CQ_M_AXIS_CANT	3												// X, Y, Z
#define FXOS8700CQ_DATA_LEN		(FXOS8700CQ_AXIS_CANT * FXOS8700CQ_OUT_LEN + \
								  FXOS8700CQ_M_AXIS_CANT * FXOS8700CQ_M_OUT_LEN)

// Number of bytes to be read from the FXOS8700CQ in a single I2C transaction
#define FXOS8700CQ_READ_LEN		(1 + FXOS8700CQ_DATA_LEN)						// Status + 6 channels (13 bytes)

#define ANGLE_THRESHOLD			5												// Threshold for angle change detection in degrees
#define SENSOR_FREQUENCY_HZ		200												// Sensor data rate in Hz
#define SENSOR_PERIOD_MS		(2000 / SENSOR_FREQUENCY_HZ)					// Sensor data rate in ms (hybrid mode)

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

typedef uint8_t byte_t;
typedef int16_t angle_t;

typedef struct {
	angle_t roll;
	angle_t pitch;
	angle_t yaw;
} sensor_t;

typedef struct {
	int16_t x;
	int16_t y;
	int16_t z;
} raw_data_t;

typedef enum {
	ROLL,
	PITCH,
	YAW,

	AXIS_CANT
} sensor_axis_t;

typedef struct {
	bool roll;
	bool pitch;
	bool yaw;
} sensor_status_t;

/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/**
 * @brief Initialize the sensor module
 * @return Initialization succeed
 * @note Interrupts should be disabled when calling this function
 */
bool sensorInit (void);

/**
 * @brief Check if the sensor has been configured
 */
bool sensorConfigStatus (void);

/**
 * @brief Check if new data is available
 * @return true if any angle has exceeded ANGLE_THRESHOLD
 */
sensor_status_t* sensorGetStatus (void);

/**
 * @brief Get raw data from the accelerometer
 * @return x, y and z values in g
 */
raw_data_t* sensorGetAccelRawData (void);

/**
 * @brief Get raw data from the magnetometer
 * @return x, y and z values in uT
 */
raw_data_t* sensorGetMagnRawData (void);

/**
 * @brief  Get roll, pitch and yaw angles from the sensor
 * @return Absolute angles in degrees
 */
sensor_t* sensorGetAngleData (void);

/*******************************************************************************
 ******************************************************************************/

#endif // _SENSOR_H_
