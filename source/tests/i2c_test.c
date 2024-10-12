/***************************************************************************//**
  @file     App.c
  @brief    Application functions
  @author   Nicolás Magliola
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include "board.h"
#include "gpio.h"
// #include "Systick.h"
#include "I2C.h"

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/
// FXOS8700CQ I2C address
#define FXOS8700CQ_SLAVE_ADDR	0x1D // with pins SA0=1, SA1=0

// FXOS8700CQ internal register addresses
#define FXOS8700CQ_STATUS		0x00
#define FXOS8700CQ_WHOAMI		0x0D
#define FXOS8700CQ_XYZ_DATA_CFG 0x0E
#define FXOS8700CQ_CTRL_REG1	0x2A
#define FXOS8700CQ_M_CTRL_REG1 	0x5B
#define FXOS8700CQ_M_CTRL_REG2 	0x5C
#define FXOS8700CQ_WHOAMI_VAL 	0xC7
#define FXOS8700CQ_WHO_AM_I		  0x0D

#define FXOS8700CQ_OUT_X_MSB	0x33
#define FXOS8700CQ_OUT_Y_MSB	0x03
#define FXOS8700CQ_OUT_Z_MSB	0x05

#define UINT14_MAX				16383

/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/

static const char seq_standby[] = {0x04, 0x00, 0xD2};
static const char seq_xyz_config[] = {0x08, 0xAA};
static const char seq_odr_config[] = {0x1D};
int i = 0;
unsigned char received_data[3];
// static const int seq_status[] = {I2C_ADD_WRITE(FXOS8700CQ_SLAVE_ADDR),FXOS8700CQ_STATUS,  I2C_RESTART_SET(1) | (FXOS8700CQ_SLAVE_ADDR << 1 | 1), I2C_READ_MASK};

// static const int seq_escritura[] = {I2C_ADD_WRITE(0x22), 0x1, 0x2, 0x3, 0x4, 0x5};

/*******************************************************************************
 *******************************************************************************
                        GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

/* Función que se llama 1 vez, al comienzo del programa */
void App_Init (void)
{
//    gpioMode(PIN_TEST_INIT, OUTPUT);
//    gpioMode(PIN_TEST_TRANSMIT, OUTPUT);
//    gpioMode(PIN_TEST_INTERRUPT, OUTPUT);
    I2C_Init(I2C0_M);
}

/* Función que se llama constantemente en un ciclo infinito */
void App_Run (void)
{
  // if((i == 0 && I2C_GetStatus(I2C0_M) == I2C_Idle) || (I2C_GetStatus(I2C0_M) == I2C_Done && i == 0))
  // {
	I2C_Status_t a = I2C_Transmit(I2C0_M, received_data, 3, FXOS8700CQ_SLAVE_ADDR, FXOS8700CQ_OUT_X_MSB, I2C_Read);
    i++;
    // gpioWrite(PIN_TEST_INIT, LOW);
  // }
  // else if(I2C_GetStatus(I2C0_M) == I2C_Done && i == 1)
  // {
  //   I2C_Transmit(I2C0_M, seq_xyz_config, sizeof(seq_xyz_config)/sizeof(char), FXOS8700CQ_SLAVE_ADDR, FXOS8700CQ_XYZ_DATA_CFG, I2C_Write);
  //   i++;
  //   gpioWrite(PIN_TEST_INIT, LOW);
  // }
  // else if(I2C_GetStatus(I2C0_M) == I2C_Done && i == 2)
  // {
  //   I2C_Transmit(I2C0_M, seq_odr_config, sizeof(seq_odr_config)/sizeof(char), FXOS8700CQ_SLAVE_ADDR, FXOS8700CQ_M_CTRL_REG2, I2C_Write);
  //   i = 0;
  //   gpioWrite(PIN_TEST_INIT, LOW);
  // }
  for(int j = 0; j < 1000000; j++);
  a = I2C_GetStatus(I2C0_M);
    // gpioWrite(PIN_TEST_INIT, HIGH);
}


/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/



/*******************************************************************************
 ******************************************************************************/
