/***************************************************************************//**
  @file     sensor.c
  @brief    FXOS8700CQ accelerometer and magnetometer driver for K64F, using I2C
  @author   Group 4
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include <math.h>

#include "board.h"
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

static bool readRegs	(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len);
static bool writeRegs	(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len);

static bool sensorGetStatus	(void);
static bool sensorReadData	(raw_data_t *accel_data, raw_data_t *magn_data);


/*******************************************************************************
 * STATIC VARIABLES AND CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

static s_raw_data_t accelData, magnData;
// static angle_t angle;
static sensor_t data, data_prev;


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

	// Configure SDA and SCL pins
	PORT_Type * port = PORT_BASE_PTRS[FXOS8700CQ_I2C_SCL_PORT];
	port->PCR[FXOS8700CQ_I2C_SCL_PIN] = 0x0;
	port->PCR[FXOS8700CQ_I2C_SCL_PIN] |= PORT_PCR_MUX(PORT_mAlt5);
	port->PCR[FXOS8700CQ_I2C_SCL_PIN] |= PORT_PCR_IRQC(PORT_eDisabled);
	port = PORT_BASE_PTRS[FXOS8700CQ_I2C_SDA_PORT];
	port->PCR[FXOS8700CQ_I2C_SDA_PIN] = 0x0;
	port->PCR[FXOS8700CQ_I2C_SDA_PIN] |= PORT_PCR_MUX(PORT_mAlt5);
	port->PCR[FXOS8700CQ_I2C_SDA_PIN] |= PORT_PCR_IRQC(PORT_eDisabled);

	return status;
}

// angle_t sensorGetAngle (sensor_axis_t axis)
// {
// 	if(sensorGetStatus() && (sensorRead() & 0x02))
// 	{
// 		// byte_t data[6];
// 		// data = ReadAccel();
// 		// int16_t x = (data[0] << 8) | data[1];
// 		// int16_t y = (data[2] << 8) | data[3];
// 		// int16_t z = (data[4] << 8) | data[5];
// 		if(axis == ROLL)
// 			angle = (angle_t) (atan2(accelData.y, accelData.z) * 180 / M_PI);
// 		else if(axis == PITCH)
// 			angle = (angle_t) (atan2(accelData.x, accelData.z) * 180 / M_PI);
// 		else if(axis == YAW)
// 			angle = (angle_t) (atan2(magnData.z, magnData.x) * 180 / M_PI);
// 	}

// 	return angle;
// }

// angle_t sensorGetData (void)
// {
// 	if(sensorGetStatus() && sensorReadData(&accelData, &magnData))
// 		angle = (angle_t) (atan2(accelData.y, accelData.z) * 180 / M_PI);

// 	return angle;
// }

sensor_t* sensorGetData (void)
{
	if(sensorGetStatus() && sensorReadData(&accelData, &magnData))
	{
		data.roll	= (angle_t) (atan2(accelData.y,	accelData.z)	* 180 / M_PI);
		data.pitch	= (angle_t) (atan2(accelData.x,	accelData.z)	* 180 / M_PI);
		data.yaw	= (angle_t) (atan2(magnData.z,	magnData.x)		* 180 / M_PI);

		data_prev = data;
	}

	return &data;
}

bool sensorGetStatus (void)
{
	return CMP_OUT(-1, (data_prev.roll	- data.roll)	/ ANGLE_THRESHOLD, 1) ||
		   CMP_OUT(-1, (data_prev.pitch	- data.pitch)	/ ANGLE_THRESHOLD, 1) ||
		   CMP_OUT(-1, (data_prev.yaw	- data.yaw)		/ ANGLE_THRESHOLD, 1);
	
		//    (data_prev.roll	> data.roll		+ ANGLE_THRESHOLD)		||
		//    (data_prev.roll	< data.roll		- ANGLE_THRESHOLD)		||
	   	//    (data_prev.pitch	> data.pitch	+ ANGLE_THRESHOLD)		||
		//    (data_prev.pitch	< data.pitch	- ANGLE_THRESHOLD)		||
	   	//    (data_prev.yaw	> data.yaw		+ ANGLE_THRESHOLD)		||
		//    (data_prev.yaw	< data.yaw		- ANGLE_THRESHOLD);
}


/*******************************************************************************
 *******************************************************************************
						LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

// static bool 		IsRxMsg		(void) { return i2cIsRxMsg(I2C0_ID); }
// static byte_t	Read		(void) { return i2cReadMsg(I2C0_ID, FXOS8700CQ_STATUS, 1); }
// static byte_t*	ReadAccel	(void) { return i2cReadMsg(I2C0_ID, FXOS8700CQ_OUT_X_MSB, 6); }
// static byte_t*	ReadMag		(void) { return i2cReadMsg(I2C0_ID, FXOS8700CQ_M_OUT_X_MSB, 6); }

bool ReadAccelMagnData (raw_data_t *accel_data, raw_data_t *magn_data)
{
	bool status = false;
	uint8_t buffer[FXOS8700CQ_READ_LEN];

	if ((readRegs(FXOS8700CQ_SLAVE_ADDR, FXOS8700CQ_STATUS, buffer, FXOS8700CQ_READ_LEN) == FXOS8700CQ_READ_LEN) &&
		(buffer[0] & 0x02))														// Check if data is ready
	{
		accel_data->x = (int16_t)(((buffer[1] << 8) | buffer[2])) >> 2;
		accel_data->y = (int16_t)(((buffer[3] << 8) | buffer[4])) >> 2;
		accel_data->z = (int16_t)(((buffer[5] << 8) | buffer[6])) >> 2;

		magn_data->x = (buffer[7]  << 8) | buffer[8];
		magn_data->y = (buffer[9]  << 8) | buffer[10];
		magn_data->z = (buffer[11] << 8) | buffer[12];

		status = true;
	}

	return status;
} // Faster if status is read first
  // Separate reads?

// Helper functions ////////////////////////////////////////////////////////////

uint8_t readRegs (uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len)
{
	uint8_t buffer = reg_addr, count = 0;
	count = i2cWriteMsg(dev_addr, &buffer, 1);
	count = (count == 1) ? i2cReadMsg(dev_addr, data, len) : 0;

	return count;
}

uint8_t writeRegs (uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len)
{
	uint8_t buffer[len + 1];
	buffer[0] = reg_addr;
	for (uint8_t i = 0; i < len; i++)
		buffer[i + 1] = data[i];

	return i2cWriteMsg(dev_addr, buffer, len + 1);
}


/******************************************************************************/
