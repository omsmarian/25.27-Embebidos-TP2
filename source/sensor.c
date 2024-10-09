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

#include "board.h"
// #include "cqueue.h"
#include "i2c.h"
#include "pisr.h"
#include "timer.h"
#include "sensor.h"


/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

#define DEVELOPMENT_MODE			1
#define DEBUG_TP					1											// Debugging test points to measure ISR time

#define CMP_IN(min, val, max)		(((min) <= (val)) && ((val) <= (max)))
#define CMP_OUT(min, val, max)		(((val) < (min)) || ((max) < (val)))


/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;
} raw_data_t;


/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/

static bool sensorGetStatus (sensor_axis_t axis);
static angle_t sensorGetAngle (sensor_axis_t axis);
static sensor_t* sensorGetAllData (void);


/*******************************************************************************
 * STATIC VARIABLES AND CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

static raw_data_t accel_data, magn_data;
// static angle_t angle;
static sensor_t data;


/*******************************************************************************
 *******************************************************************************
						GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

// Main Services ///////////////////////////////////////////////////////////////

void sensorInit (void)
{
	i2c_cfg_t config = { I2C0_BAUDRATE, I2C0_MASTER_ADDR, I2C0_MASTER_MODE };
	i2cInit(I2C0_ID, config);

	bool status = true;
	uint8_t databyte;

	status = (readRegs(FXOS8700CQ_SLAVE_ADDR, FXOS8700CQ_WHOAMI, &databyte, 1) == 1) ||
			 (databyte == FXOS8700CQ_WHOAMI_VAL) // Read and check the ID

	databyte = 0x00; // Place into standby
	if (status) status = writeRegs(FXOS8700CQ_SLAVE_ADDR, FXOS8700CQ_CTRL_REG1,		&databyte, 1) == 1;

	databyte = 0x1F; // No auto calibration, one-shot magn reset or measurement, 8x os and hybrid mode
	if (status) status = writeRegs(FXOS8700CQ_SLAVE_ADDR, FXOS8700CQ_M_CTRL_REG1,	&databyte, 1) == 1;

	databyte = 0x20; // Map magn registers to follow accel, retain min/max latching and enable magn reset each cycle
	if (status) status = writeRegs(FXOS8700CQ_SLAVE_ADDR, FXOS8700CQ_M_CTRL_REG2,	&databyte, 1) == 1;

	databyte = 0x01; //No filter and accel range of +/-4g range with 0.488mg/LSB
	if (status) status = writeRegs(FXOS8700CQ_SLAVE_ADDR, FXOS8700CQ_XYZ_DATA_CFG,	&databyte, 1) == 1;

	databyte = 0x0D; // 200Hz data rate, low noise, 16 bit reads, out of standby and enable sampling
	if (status) status = writeRegs(FXOS8700CQ_SLAVE_ADDR, FXOS8700CQ_CTRL_REG1,		&databyte, 1) == 1;

	databyte = 0x00; // Disable FIFO
	if (status) status = writeRegs(FXOS8700CQ_SLAVE_ADDR, FXOS8700CQ_F_SETUP,		&databyte, 1) == 1;

	// Enable clock to I2C module
	// SIM->SCGC4 |= SIM_SCGC4_I2C0_MASK;

	// // Configure SDA and SCL pins
	// PORT_Type * port = PORT_BASE_PTRS[FXOS8700CQ_I2C_SCL_PORT];
	// port->PCR[FXOS8700CQ_I2C_SCL_PIN] = 0x0;
	// port->PCR[FXOS8700CQ_I2C_SCL_PIN] |= PORT_PCR_MUX(PORT_mAlt5);
	// port->PCR[FXOS8700CQ_I2C_SCL_PIN] |= PORT_PCR_IRQC(PORT_eDisabled);
	// port = PORT_BASE_PTRS[FXOS8700CQ_I2C_SDA_PORT];
	// port->PCR[FXOS8700CQ_I2C_SDA_PIN] = 0x0;
	// port->PCR[FXOS8700CQ_I2C_SDA_PIN] |= PORT_PCR_MUX(PORT_mAlt5);
	// port->PCR[FXOS8700CQ_I2C_SDA_PIN] |= PORT_PCR_IRQC(PORT_eDisabled);

	// Configure PISR for the sensor
	pisrRegister(readData, PISR_FREQUENCY_HZ / SENSOR_FREQUENCY_HZ);

	// // Create queues for sensor data
	// int sensor_accel_queue = queueInit();
	// int sensor_magn_queue = queueInit();

	return status;
}

bool sensorGetStatus (sensor_axis_t axis)
{
	// processData();

	data.roll	= (angle_t) (atan2(accelData->y,	accelData->z)	* 180 / M_PI);
	data.pitch	= (angle_t) (atan2(accelData->x,	accelData->z)	* 180 / M_PI);
	data.yaw	= (angle_t) (atan2(magnData->z,		magnData->x)	* 180 / M_PI);

	return (axis == ROLL)	? CMP_OUT(-1, (float)(data.roll		/ ANGLE_THRESHOLD), 1) :
	 	   (axis == PITCH)	? CMP_OUT(-1, (float)(data.pitch	/ ANGLE_THRESHOLD), 1) :
	 	   (axis == YAW)	? CMP_OUT(-1, (float)(data.yaw		/ ANGLE_THRESHOLD), 1) : false;

	// return CMP_OUT(-1, (float)(data.roll	/ ANGLE_THRESHOLD), 1) ||
	// 	   CMP_OUT(-1, (float)(data.pitch	/ ANGLE_THRESHOLD), 1) ||
	// 	   CMP_OUT(-1, (float)(data.yaw		/ ANGLE_THRESHOLD), 1);
}

angle_t sensorGetAngle (sensor_axis_t axis)
{
	return (axis == ROLL)	? data.roll		:
		   (axis == PITCH)	? data.pitch	:
		   (axis == YAW)	? data.yaw		: 0;
}

sensor_t* sensorGetAllData (void)
{
	accel_data = magn_data = (raw_data_t) { 0, 0, 0 };
	data = (sensor_t) { 0, 0, 0 };

	return &data;
}


/*******************************************************************************
 *******************************************************************************
						LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

void readData (void)
{
	uint8_t status = false, buffer[FXOS8700CQ_READ_LEN];
	// raw_data_t accel_data, magn_data;
	// sensor_t* data;

	if ((readRegs(FXOS8700CQ_SLAVE_ADDR, FXOS8700CQ_STATUS, &status, 1) == 1) &&
		(status &= 0x02)) // Check if data is ready (faster if status is read first)
	{
		if (status = readRegs(FXOS8700CQ_SLAVE_ADDR, FXOS8700CQ_OUT_X_MSB, buffer, FXOS8700CQ_READ_LEN) == FXOS8700CQ_READ_LEN)
		{
			accel_data.x += (int16_t)((buffer[0] << 8) | buffer[1]) >> 2;
			accel_data.y += (int16_t)((buffer[2] << 8) | buffer[3]) >> 2;
			accel_data.z += (int16_t)((buffer[4] << 8) | buffer[5]) >> 2;

			magn_data.x += (int16_t)((buffer[6] << 8) | buffer[7]);
			magn_data.y += (int16_t)((buffer[8] << 8) | buffer[9]);
			magn_data.z += (int16_t)((buffer[10] << 8) | buffer[11]);

			// accel_data = { (int16_t)(((buffer[0] << 8) | buffer[1])) >> 2,
			// 			   (int16_t)(((buffer[2] << 8) | buffer[3])) >> 2,
			// 			   (int16_t)(((buffer[4] << 8) | buffer[5])) >> 2 };

			// magn_data = { (int16_t)((buffer[6]	<< 8) | buffer[7]),
			// 			  (int16_t)((buffer[8]	<< 8) | buffer[9]),
			// 			  (int16_t)((buffer[10] << 8) | buffer[11]) };

			// data = processData(&accel_data, &magn_data);
			// queuePush(queue, data->roll);
			// queuePush(queue, data->pitch);
			// queuePush(queue, data->yaw);

			// queuePush(accel_queue, buffer[1]);
			// queuePush(accel_queue, buffer[3]);
			// queuePush(accel_queue, buffer[5]);

			// queuePush(magn_queue, buffer[7]);
			// queuePush(magn_queue, buffer[9]);
			// queuePush(magn_queue, buffer[11]);
		}
	}
}

// void processData (void)
// {
// 	data.roll	= (angle_t) (atan2(accelData->y,	accelData->z)	* 180 / M_PI);
// 	data.pitch	= (angle_t) (atan2(accelData->x,	accelData->z)	* 180 / M_PI);
// 	data.yaw	= (angle_t) (atan2(magnData->z,		magnData->x)	* 180 / M_PI);
// }


/******************************************************************************/
