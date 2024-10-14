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
#include "debug.h"																// For measuring ISR time
#include "I2C.h"
#include "macros.h"
#include "pisr.h"
#include "sensor.h"

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

#define DEVELOPMENT_MODE			1

#define CONFIG_FREQUENCY_HZ			1000U

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

typedef enum { OFF, IDLE, READING } state_t;
typedef enum { BLOCKING, NON_BLOCKING } mode_t;
typedef enum { BUSY, DONE, ERROR } bus_status_t;

/*******************************************************************************
 * STATIC VARIABLES AND CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

static const bus_status_t bus_status[] = { BUSY, DONE, ERROR };

/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/

/**
 * @brief Called periodically to configure all registers (once each)
 * @note Status should be checked with sensorConfigStatus()
 */
static void config (void);

/**
 * @brief Read (store) data from the sensor periodically
 */
static void readData (void);

/**
 * @brief Process raw data to get angles
 */
static void processData (void);

// Helper Functions ////////////////////////////////////////////////////////////

/**
 * @brief Initialize bus
 */
static bool init(void);

/**
 * @brief Read data from the sensor
 * @param reg Register to read
 * @param data Buffer to store data
 * @param len Data length
 * @param mode Blocking or non-blocking
 * @return Bus status
 */
static bus_status_t readRegs (uint8_t reg, uint8_t* data, uint8_t len, mode_t mode);

/**
 * @brief Write data to the sensor
 * @param reg Register to write
 * @param data Data to write
 * @param len Data length
 * @param mode Blocking or non-blocking
 * @return Bus status
 */
static bus_status_t writeRegs (uint8_t reg, uint8_t* data, uint8_t len, mode_t mode);

/**
 * @brief Check transmission state
 * @return Bus status
 */
static bus_status_t transmitState (void);

/**
 * @brief Wait for the bus to be free
 * @return Bus status after waiting
 * @note Blocking function
 */
static bus_status_t wait (void);

/**
 * @brief Check (store) bus status
 */
static void checkBus (void);

/*******************************************************************************
 * STATIC VARIABLES AND CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

static raw_data_t accel_data, magn_data;
static sensor_t data, data_prev;
static bool status_bus, status_config;

/*******************************************************************************
 *******************************************************************************
						GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

// Main Services ///////////////////////////////////////////////////////////////

bool sensorInit (void)
{
	static bool status = false;

	if(!status)
	{
		init();
		pisrRegister(readData, PISR_FREQUENCY_HZ / SENSOR_FREQUENCY_HZ); // Configure PISR for the sensor
		pisrRegister(config, PISR_FREQUENCY_HZ / CONFIG_FREQUENCY_HZ);
		status = true;
	}

	return status;
}

bool sensorConfigStatus (void)
{
	return status_config;
}

sensor_status_t* sensorGetStatus ()
{
	static sensor_status_t status;
	status = (sensor_status_t){ false, false, false };

	processData();

	status.roll		= CMP_OUT(-ANGLE_THRESHOLD, data_prev.roll	- data.roll,	ANGLE_THRESHOLD);
	status.pitch	= CMP_OUT(-ANGLE_THRESHOLD, data_prev.pitch	- data.pitch,	ANGLE_THRESHOLD);
	status.yaw		= CMP_OUT(-ANGLE_THRESHOLD, data_prev.yaw	- data.yaw,		ANGLE_THRESHOLD);

	return &status;
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
	data_prev = data;

	return &data;
}

/*******************************************************************************
 *******************************************************************************
						LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

static void config (void)
{
#if DEBUG_SENSOR
P_DEBUG_TP_SET
#endif
	static bool status[7] = { false, false, false, false, false, false, false };
	static uint8_t name = FXOS8700CQ_WHOAMI_VAL - 1;
	uint8_t databyte;

	if (!status_config && (transmitState() == DONE))
	{
		if (!status[0])
			status[0] = readRegs(FXOS8700CQ_WHOAMI, &name, 1, BLOCKING) == BUSY; // Read and check the ID

		if (name == FXOS8700CQ_WHOAMI_VAL)
		{
			if (!status[1])
			{
				databyte = 0x00; // Place into standby
				status[1] = writeRegs(FXOS8700CQ_CTRL_REG1, &databyte, 1, BLOCKING) == BUSY;
			}
			else if (!status[2])
			{
				databyte = 0x1F; // No auto calibration, one-shot magn reset or measurement, 8x oversampling and hybrid mode
				status[2] = writeRegs(FXOS8700CQ_M_CTRL_REG1, &databyte, 1, BLOCKING) == BUSY;
			}
			else if (!status[3])
			{
				databyte = 0x20; // Map magn registers to follow accel, retain min/max latching and enable magn reset each cycle
				status[3] = writeRegs(FXOS8700CQ_M_CTRL_REG2, &databyte, 1, BLOCKING) == BUSY;
			}
			else if (!status[4])
			{
				databyte = 0x01; //No filter and accel range of +/-4g range with 0.488mg/LSB
//				status[4] = writeRegs(FXOS8700CQ_XYZ_DATA_CFG, &databyte, 1, BLOCKING) == BUSY;
				status[4] = readRegs(FXOS8700CQ_M_CTRL_REG2, &databyte, 1, BLOCKING) == BUSY;
			}
			else if (!status[5])
			{
				databyte = 0x00; // Disable FIFO
				status[5] = writeRegs(FXOS8700CQ_F_SETUP, &databyte, 1, BLOCKING) == BUSY;
			}
			else if (!status[6])
			{
				databyte = 0x0D; // 200Hz data rate, low noise, 16 bit reads, out of standby and enable sampling
				status[6] = writeRegs(FXOS8700CQ_CTRL_REG1, &databyte, 1, BLOCKING) == BUSY;
			}
			else
				status_config = true;
		}
	}
#if DEBUG_SENSOR
P_DEBUG_TP_CLR
#endif
}

static void readData (void)
{
#if DEBUG_SENSOR
P_DEBUG_TP_SET
#endif
	static state_t state = OFF;
	static uint8_t buffer[FXOS8700CQ_READ_LEN];

	/* Small FSM to read data from the sensor only when the bus is free */
	switch (state)
	{
		case OFF:
			if (status_config)
				state = IDLE;
			break;
		case IDLE:																// Wait for the bus to be free to read data
			for (uint8_t i = 0; i < FXOS8700CQ_READ_LEN; i++)
				buffer[i] = 0;
			if((transmitState() != BUSY) && (readRegs(FXOS8700CQ_STATUS, buffer, FXOS8700CQ_READ_LEN, NON_BLOCKING) == BUSY))
				state = READING;
			break;

		case READING:
			if (transmitState() != BUSY)
				state = IDLE;

			if ((transmitState() == DONE) && (buffer[0] &= 0x08))				// Check if data is ready
			{
				accel_data.x = (int16_t)((buffer[1] << 8) | buffer[2]) >> 2;
				accel_data.y = (int16_t)((buffer[3] << 8) | buffer[4]) >> 2;
				accel_data.z = (int16_t)((buffer[5] << 8) | buffer[6]) >> 2;

				magn_data.x = (int16_t)((buffer[7]  << 8) | buffer[8]);
				magn_data.y = (int16_t)((buffer[9]  << 8) | buffer[10]);
				magn_data.z = (int16_t)((buffer[11] << 8) | buffer[12]);
			}
			break;

		default:
			state = IDLE;
			break;
	}
#if DEBUG_SENSOR
P_DEBUG_TP_CLR
#endif
}

static void processData (void)
{
	data.roll	= (angle_t)(atan2(accel_data.x,	accel_data.z)	* 180 / M_PI);
	data.pitch	= (angle_t)(atan2(accel_data.y,	accel_data.z)	* 180 / M_PI);
	data.yaw	= (angle_t)(atan2(magn_data.z,	magn_data.x)	* 180 / M_PI);
}

// Helper Functions ////////////////////////////////////////////////////////////

static bool init (void)
{
	static bool status = false;

	if (!status)
	{
		I2C_Init(I2C0_M);
		status = true;
	}

	return status;
}

static bus_status_t readRegs (uint8_t reg, uint8_t* data, uint8_t len, mode_t mode)
{
	bus_status_t status = ERROR;

//	if (mode == BLOCKING) { wait(); }
	status = bus_status[I2C_Transmit(I2C0_M, data, len, FXOS8700CQ_SLAVE_ADDR, reg, I2C_Read)];
//	if (mode == BLOCKING) { status = wait(); }

	return status;
}

static bus_status_t writeRegs (uint8_t reg, uint8_t* data, uint8_t len, mode_t mode)
{
	bus_status_t status = ERROR;

//	if (mode == BLOCKING) { wait(); }
	status = bus_status[I2C_Transmit(I2C0_M, data, len, FXOS8700CQ_SLAVE_ADDR, reg, I2C_Write)];
//	if (mode == BLOCKING) { status = wait(); }

	return status;
}

static bus_status_t transmitState (void)
{
	return bus_status[I2C_GetStatus(I2C0_M)];
}

static bus_status_t wait (void)
{
	bus_status_t status;

	while ((status = transmitState()) == BUSY);

	return status;
}

static void checkBus (void)
{
	status_bus = transmitState();
}

/******************************************************************************/
