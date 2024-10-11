/***************************************************************************//**
  @file     sensor.c
  @brief    FXOS8700CQ accelerometer and magnetometer driver for K64F, using I2C
  @author   Group 4: - Oms, Mariano
                     - Solari Raigoso, Agustín
                     - Wickham, Tomás
                     - Vieira, Valentin Ulises
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include <math.h>
#include <string.h>

#include "board.h"
#include "i2c.h"
// #include "PDRV_I2C.h"
#include "pisr.h"
#include "timer.h"
#include "sensor.h"


/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

#define DEVELOPMENT_MODE			1
#define DEBUG_TP					1											// Debugging test points to measure ISR time

#define CMP_IN(min, val, max)		(((min) < (val)) && ((val) < (max)))		// In:  (min, max)
#define CMP_OUT(min, val, max)		(((val) <= (min)) || ((max) <= (val)))		// Out: (-inf, min] U [max, +inf)

//#define M_PI						3.14159265358979323846


/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

// typedef enum
// {
// 	NONE,
// 	STATUS,
// 	DATA
// } reading_state_t;

typedef enum
{
	IDLE,
	READING,
} state_t;


/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/

/**
 * @brief Read data from the sensor
 */
static void readData (void);

/**
 * @brief Process raw data to get angles
 */
static void processData (void);

// Helper Functions ////////////////////////////////////////////////////////////

/**
 * @brief Read data from the sensor
 * @param reg Register to read
 * @param data Buffer to store data
 * @param len Data length
 * @return I2C bus status
 */
static I2C_Status_t readRegs (uint8_t reg, uint8_t * data, size_t len);

/**
 * @brief Write data to the sensor
 * @param reg Register to write
 * @param data Data to write
 * @param len Data length
 * @return I2C bus status
 */
static I2C_Status_t writeRegs (uint8_t reg, uint8_t * data, size_t len);

/**
 * @brief Check I2C transmission state
 * @return I2C bus status
 */
static I2C_Status_t transmitState (void);


/*******************************************************************************
 * STATIC VARIABLES AND CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

static raw_data_t accel_data, magn_data;
static sensor_t data, data_prev;


/*******************************************************************************
 *******************************************************************************
						GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

// Main Services ///////////////////////////////////////////////////////////////

bool sensorInit (void)
{
	bool status = true;
	uint8_t databyte;

	// I2C_InitModule(0);

	status = (writeRegs(FXOS8700CQ_WHOAMI, &databyte, 1) == 1) &&
			 (databyte == FXOS8700CQ_WHOAMI_VAL); // Read and check the ID

	databyte = 0x00; // Place into standby
	if (status) status = writeRegs(FXOS8700CQ_CTRL_REG1,	&databyte, 1) == 1;

	databyte = 0x1F; // No auto calibration, one-shot magn reset or measurement, 8x os and hybrid mode
	if (status) status = writeRegs(FXOS8700CQ_M_CTRL_REG1,	&databyte, 1) == 1;

	databyte = 0x20; // Map magn registers to follow accel, retain min/max latching and enable magn reset each cycle
	if (status) status = writeRegs(FXOS8700CQ_M_CTRL_REG2,	&databyte, 1) == 1;

	databyte = 0x01; //No filter and accel range of +/-4g range with 0.488mg/LSB
	if (status) status = writeRegs(FXOS8700CQ_XYZ_DATA_CFG,	&databyte, 1) == 1;

	databyte = 0x0D; // 200Hz data rate, low noise, 16 bit reads, out of standby and enable sampling
	if (status) status = writeRegs(FXOS8700CQ_CTRL_REG1,	&databyte, 1) == 1;

	databyte = 0x00; // Disable FIFO
	if (status) status = writeRegs(FXOS8700CQ_F_SETUP,		&databyte, 1) == 1;

	// Configure PISR for the sensor
	pisrRegister(readData, PISR_FREQUENCY_HZ / SENSOR_FREQUENCY_HZ);

	return status;
}

bool sensorGetStatus (sensor_axis_t axis)
{
	bool update_roll, update_pitch, update_yaw;

	processData();

	update_roll		= CMP_OUT(-ANGLE_THRESHOLD, data_prev.roll	- data.roll,	ANGLE_THRESHOLD);
	update_pitch	= CMP_OUT(-ANGLE_THRESHOLD, data_prev.pitch	- data.pitch,	ANGLE_THRESHOLD);
	update_yaw		= CMP_OUT(-ANGLE_THRESHOLD, data_prev.yaw	- data.yaw,		ANGLE_THRESHOLD);

	return (axis == ROLL)	? update_roll	:
		   (axis == PITCH)	? update_pitch	:
		   (axis == YAW)	? update_yaw	:
		   (axis == ALL)	? (update_roll || update_pitch || update_yaw) : false;
}

raw_data_t* sensorGetAccelRawData (void)
{
	return &accel_data;
}

raw_data_t* sensorGetMagnRawData (void)
{
	return &magn_data;
}

sensor_t* sensorGetAngleData (void)
{
	processData();
	data_prev = data;

	return &data;
}


/*******************************************************************************
 *******************************************************************************
						LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

// static void readData (void)
// {
// 	static reading_state_t state = NONE;
// 	static uint8_t status = 0, buffer[FXOS8700CQ_READ_LEN];

// 	switch (state)
// 	{
// 		case NONE:																// Wait for the bus to be free and read status
// 			if((transmitState() == I2C_Idle) && (readRegs(FXOS8700CQ_STATUS, &status, 1) == I2C_Busy))
// 				state = STATUS;
// 			break;

// 		case STATUS:
// 			if (transmitState() == I2C_Done)
// 			{
// 				if ((status &= 0x02) && (readRegs(FXOS8700CQ_OUT_X_MSB, buffer, FXOS8700CQ_READ_LEN) == I2C_Busy))											// Check if data is ready
// 					state = DATA;
// 				else
// 					state = NONE;
// 			}
// 			else if (transmitState() == I2C_Error)
// 				state = NONE;
// 			break;
		
// 		case DATA:
// 			if (transmitState() == I2C_Done)
// 			{
// 				accel_data.x = (int16_t)((buffer[0] << 8) | buffer[1]) >> 2;
// 				accel_data.y = (int16_t)((buffer[2] << 8) | buffer[3]) >> 2;
// 				accel_data.z = (int16_t)((buffer[4] << 8) | buffer[5]) >> 2;

// 				magn_data.x = (int16_t)((buffer[6]  << 8) | buffer[7]);
// 				magn_data.y = (int16_t)((buffer[8]  << 8) | buffer[9]);
// 				magn_data.z = (int16_t)((buffer[10] << 8) | buffer[11]);

// 				if(readRegs(FXOS8700CQ_STATUS, &status, 1) == I2C_Busy)
// 					state = STATUS;
// 				else
// 					state = NONE;
// 			}
// 			else if (transmitState() == I2C_Error)
// 				state = NONE;
// 			break;

// 		default:
// 			state = NONE;
// 			break;
// 	}
// }

static void readData (void)
{
	static state_t state = IDLE;
	static uint8_t buffer[FXOS8700CQ_READ_LEN];

	switch (state)
	{
		case IDLE:																// Wait for the bus to be free to read data
			if((transmitState() == I2C_Idle) && (readRegs(FXOS8700CQ_STATUS, buffer, FXOS8700CQ_READ_LEN) == I2C_Busy))
				state = READING;
			break;

		case READING:
			if (transmitState() != I2C_Busy)
				state = IDLE;

			if ((transmitState() == I2C_Done) && (buffer[0] &= 0x02))			// Check if data is ready
			{
				accel_data.x = (int16_t)((buffer[0] << 8) | buffer[1]) >> 2;
				accel_data.y = (int16_t)((buffer[2] << 8) | buffer[3]) >> 2;
				accel_data.z = (int16_t)((buffer[4] << 8) | buffer[5]) >> 2;

				magn_data.x = (int16_t)((buffer[6]  << 8) | buffer[7]);
				magn_data.y = (int16_t)((buffer[8]  << 8) | buffer[9]);
				magn_data.z = (int16_t)((buffer[10] << 8) | buffer[11]);
			}
			break;

		default:
			state = IDLE;
			break;
	}
}

static void processData (void)
{
	data.roll	= (angle_t) (atan2(accel_data.y,	accel_data.z)	* 180 / M_PI);
	data.pitch	= (angle_t) (atan2(accel_data.x,	accel_data.z)	* 180 / M_PI);
	data.yaw	= (angle_t) (atan2(magn_data.z,		magn_data.x)	* 180 / M_PI);
}

// Helper Functions ////////////////////////////////////////////////////////////

// static I2C_Status_t readRegs ( uint8_t reg, uint8_t * data, size_t len)
// {
// 	uint8_t buffer[1] = {reg};

// 	return I2C_InitObject(I2C0_M, data, len, buffer, 1, FXOS8700CQ_SLAVE_ADDR);
// }

// static I2C_Status_t writeRegs (uint8_t reg, uint8_t * data, size_t len)
// {
// 	uint8_t buffer[len + 1];

// 	buffer[0] = reg;
// 	memcpy(&buffer[1], data, len);

// 	return I2C_InitObject(I2C0_M, NULL, 0, buffer, len + 1, FXOS8700CQ_SLAVE_ADDR);
// }

// static I2C_Status_t transmitState (void)
// {
// 	return i2cTransactionState(I2C0_M);
// }

// static I2C_Status_t readRegs ( uint8_t reg, uint8_t * data, size_t len)
// {
// 	uint8_t buffer[1] = {reg};

// 	return i2cSendSequence(0, FXOS8700CQ_SLAVE_ADDR, buffer, 1, data, len);

// }


/******************************************************************************/
