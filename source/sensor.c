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

#define DEBUG_TP					1											// Debugging Test Points to measure ISR time


/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;
} s_raw_data_t;


/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/

static bool s_i2c_read_regs (i2c_cfg_t i2c, uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len);
static bool s_i2c_write_regs (i2c_cfg_t i2c, uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len);

static bool sensorIsRxMsg (void);
static bool sensorReadData (s_raw_data_t *pAccelData, s_raw_data_t *pMagnData);


/*******************************************************************************
 * STATIC VARIABLES AND CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

static s_raw_data_t accelData, magnData;
// static angle_t angle;
static sensor_t data;


/*******************************************************************************
 *******************************************************************************
						GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

// Main Services ///////////////////////////////////////////////////////////////

// function configures FXOS8700CQ combination accelerometer and magnetometer sensor
void sensorInit (void)
{
	i2c_cfg_t config = {I2C0_BAUDRATE, I2C0_SLAVE_ADDR, I2C0_MASTER_MODE};
	i2cInit(I2C0_ID, config);

	bool status = true;
	uint8_t databyte;

	// read and check the FXOS8700CQ WHOAMI register
	if ((s_i2c_read_regs(aFP, FXOS8700CQ_SLAVE_ADDR, FXOS8700CQ_WHOAMI, &databyte, (uint8_t) 1) != 1) ||
		(databyte != FXOS8700CQ_WHOAMI_VAL))
		status = false;

	// write 0000 0000 = 0x00 to accelerometer control register 1 to place FXOS8700CQ into
	// standby
	// [7-1] = 0000 000
	// [0]: active=0
	databyte = 0x00;
	if (s_i2c_write_regs(aFP, FXOS8700CQ_SLAVE_ADDR, FXOS8700CQ_CTRL_REG1, &databyte, (uint8_t) 1) != 1)
		status = false;

	// write 0001 1111 = 0x1F to magnetometer control register 1
	// [7]: m_acal=0: auto calibration disabled
	// [6]: m_rst=0: no one-shot magnetic reset
	// [5]: m_ost=0: no one-shot magnetic measurement
	// [4-2]: m_os=111=7: 8x oversampling (for 200Hz) to reduce magnetometer noise
	// [1-0]: m_hms=11=3: select hybrid mode with accel and magnetometer active
	databyte = 0x1F;
	if (s_i2c_write_regs(aFP, FXOS8700CQ_SLAVE_ADDR, FXOS8700CQ_M_CTRL_REG1, &databyte, (uint8_t) 1) != 1)
		status = false;

	// write 0010 0000 = 0x20 to magnetometer control register 2
	// [7]: reserved
	// [6]: reserved
	// [5]: hyb_autoinc_mode=1 to map the magnetometer registers to follow the
	// accelerometer registers
	// [4]: m_maxmin_dis=0 to retain default min/max latching even though not used
	// [3]: m_maxmin_dis_ths=0
	// [2]: m_maxmin_rst=0
	// [1-0]: m_rst_cnt=00 to enable magnetic reset each cycle
	databyte = 0x20;
	if (s_i2c_write_regs(aFP, FXOS8700CQ_SLAVE_ADDR, FXOS8700CQ_M_CTRL_REG2, &databyte, (uint8_t) 1) != 1)
		status = false;

	// write 0000 0001= 0x01 to XYZ_DATA_CFG register
	// [7]: reserved
	// [6]: reserved
	// [5]: reserved
	// [4]: hpf_out=0
	// [3]: reserved
	// [2]: reserved
	// [1-0]: fs=01 for accelerometer range of +/-4g range with 0.488mg/LSB
	databyte = 0x01;
	if (s_i2c_write_regs(aFP, FXOS8700CQ_SLAVE_ADDR, FXOS8700CQ_XYZ_DATA_CFG, &databyte, (uint8_t) 1) != 1)
		status = false;

	// write 0000 1101 = 0x0D to accelerometer control register 1
	// [7-6]: aslp_rate=00
	// [5-3]: dr=001 for 200Hz data rate (when in hybrid mode)
	// [2]: lnoise=1 for low noise mode
	// [1]: f_read=0 for normal 16 bit reads
	// [0]: active=1 to take the part out of standby and enable sampling
	databyte = 0x0D;
	if (s_i2c_write_regs(aFP, FXOS8700CQ_SLAVE_ADDR, FXOS8700CQ_CTRL_REG1, &databyte, (uint8_t) 1) != 1)
		status = false;

	return status;
}

// angle_t sensorGetAngle (sensor_axis_t axis)
// {
// 	if(sensorIsRxMsg() && (sensorRead() & 0x02))
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
// 	if(sensorIsRxMsg() && sensorReadData(&accelData, &magnData))
// 	{
// 		angle = (angle_t) (atan2(accelData.y, accelData.z) * 180 / M_PI);
// 	}

// 	return angle;
// }

sensor_t * sensorGetData (void)
{
	if(sensorIsRxMsg() && sensorReadData(&accelData, &magnData))
	{
		data.roll = (angle_t) (atan2(accelData.y, accelData.z) * 180 / M_PI);
		data.pitch = (angle_t) (atan2(accelData.x, accelData.z) * 180 / M_PI);
		data.yaw = (angle_t) (atan2(magnData.z, magnData.x) * 180 / M_PI);
	}

	return &data;
}


/*******************************************************************************
 *******************************************************************************
						LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

static bool 	IsRxMsg		(void) { return i2cIsRxMsg(I2C0_ID); }
// static byte_t	Read		(void) { return i2cReadMsg(I2C0_ID, FXOS8700CQ_STATUS, 1); }
// static byte_t*	ReadAccel	(void) { return i2cReadMsg(I2C0_ID, FXOS8700CQ_OUT_X_MSB, 6); }
// static byte_t*	ReadMag		(void) { return i2cReadMsg(I2C0_ID, FXOS8700CQ_M_OUT_X_MSB, 6); }

bool ReadAccelMagnData (s_raw_data_t *pAccelData, s_raw_data_t *pMagnData)
{
	bool status = true;
	MQX_FILE_PTR fp;															// I2C file pointer
	uint8_t Buffer[FXOS8700CQ_READ_LEN];										// read buffer

	// read FXOS8700CQ_READ_LEN=13 bytes (status byte and the six channels of data)
	if (s_i2c_read_regs(fp, FXOS8700CQ_SLAVE_ADDR, FXOS8700CQ_STATUS, Buffer, FXOS8700CQ_READ_LEN) == FXOS8700CQ_READ_LEN)
	{
		// copy the 14 bit accelerometer byte data into 16 bit words
		pAccelData->x = (int16_t)(((Buffer[1] << 8) | Buffer[2]))>> 2;
		pAccelData->y = (int16_t)(((Buffer[3] << 8) | Buffer[4]))>> 2;
		pAccelData->z = (int16_t)(((Buffer[5] << 8) | Buffer[6]))>> 2;

		// copy the magnetometer byte data into 16 bit words
		pMagnData->x = (Buffer[7] << 8) | Buffer[8];
		pMagnData->y = (Buffer[9] << 8) | Buffer[10];
		pMagnData->z = (Buffer[11] << 8) | Buffer[12];

		status = true;
	}

	return status;
}

// Helper functions ////////////////////////////////////////////////////////////

void s_i2c_read_regs (i2c_cfg_t i2c, uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len)
{
	uint8_t buffer[1] = {reg_addr};
	i2cWriteMsg(i2c, dev_addr, buffer, 1);
	i2cReadMsg(i2c, dev_addr, data, len);
}

void s_i2c_write_regs (i2c_cfg_t i2c, uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len)
{
	uint8_t buffer[len + 1];
	buffer[0] = reg_addr;
	for (uint8_t i = 0; i < len; i++)
		buffer[i + 1] = data[i];
	i2cWriteMsg(i2c, dev_addr, buffer, len + 1);
}


/******************************************************************************/
