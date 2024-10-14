/***************************************************************************//**
  @file     I2C.c
  @brief    +Descripcion del archivo+
  @author   Mariano Oms
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include <gpio.h>
#include "I2C.h"
#include "hardware.h"
#include "board.h"
#include "debug.h"
#include <stdlib.h>


/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

//I2C Timings
#define I2C_BUS_CLOCK		50000000U
#define I2C_BAUD_RATE   100000U


//Pines
#define I2C0_SCL_PIN 24
#define I2C0_SDA_PIN 25

//Mux
#define PORT_mAlt5 5


//I2C Register Macros
#define I2C_WRITE_DATA(data)    	(I2C_arr[module]->D = data)
#define I2C_ADDRESS_MASK			    ((I2C_Objects[module].slave_address & I2C_D_DATA_MASK)<<1)
#define I2C_SET_NACK	         	  (I2C_arr[module]->C1 |= I2C_C1_TXAK_MASK)
#define I2C_CLEAR_NACK      	 	  (I2C_arr[module]->C1 &= ~I2C_C1_TXAK_MASK)
#define I2C_START_SIGNAL        	(I2C_arr[module]->C1 |= I2C_C1_MST_MASK)
#define I2C_STOP_SIGNAL          	(I2C_arr[module]->C1 &= ~I2C_C1_MST_MASK)
#define I2C_REPEAT_START_SIGNAL 	(I2C_arr[module]->C1 |= I2C_C1_RSTA_MASK)
#define I2C_READ_DATA           	(I2C_arr[module]->D)
#define I2C_STATUS_BYTE           (I2C_arr[module]->S)
#define I2C_GET_TX_MODE				    (I2C_arr[module]->C1 & I2C_C1_TX_MASK)
#define I2C_GET_IRQ_FLAG        	(I2C_arr[module]->S & I2C_S_IICIF_MASK)
#define I2C_CLEAR_IRQ_FLAG      	(I2C_arr[module]->S |= I2C_S_IICIF_MASK)
#define I2C_GET_RX_ACK				    (I2C_arr[module]->S & I2C_S_RXAK_MASK)    // 0 = ACK received, 1 = no ACK received
#define I2C_SET_RX_MODE         	(I2C_arr[module]->C1 &= ~I2C_C1_TX_MASK)
#define I2C_SET_TX_MODE         	(I2C_arr[module]->C1 |= I2C_C1_TX_MASK)
#define I2C_SET_NACK	         	  (I2C_arr[module]->C1 |= I2C_C1_TXAK_MASK)
#define I2C_CLEAR_NACK      	 	  (I2C_arr[module]->C1 &= ~I2C_C1_TXAK_MASK)


//I2C Object Macros
#define I2C_INDEX                 (I2C_Objects[module].index)
#define I2C_STATUS                (I2C_Objects[module].status)
#define I2C_MODE                  (I2C_Objects[module].mode)
#define I2C_SLAVE_ADDRESS         (I2C_Objects[module].slave_address)
#define I2C_REG_ADDRESS           (I2C_Objects[module].reg_address)
#define I2C_SEQUENCE_SIZE         (I2C_Objects[module].sequence_size)
#define I2C_REG_ADDRESS_SENT      (I2C_Objects[module].reg_address_flag)
#define I2C_REPEATED_START_SENT   (I2C_Objects[module].repeated_start)
#define I2C_SEQUENCE_ARR          (I2C_Objects[module].sequence_arr)


/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

// Declaring the baud rate configuration structure
typedef struct {
  uint8_t mul; 	    // Multiplier Factor, register value
  uint8_t icr;	    // ClockRate, register value
} BaudRate_t;


/*******************************************************************************
 * VARIABLES WITH GLOBAL SCOPE
 ******************************************************************************/



/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/

static BaudRate_t SetBaudRate (uint32_t desiredBaudRate);

/*******************************************************************************
 * ROM CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/



/*******************************************************************************
 * STATIC VARIABLES AND CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

static I2C_Type* const I2C_arr[] = I2C_BASE_PTRS;           // Array of pointers to I2C modules
static PORT_Type* const Port_arr[] = PORT_BASE_PTRS;        // Array of pointers to PORT modules
static IRQn_Type const I2C_IRQn[] = I2C_IRQS;               // Array of IRQs for I2C modules
static I2C_Object_t I2C_Objects[I2C_COUNT];                 // Array of I2C objects


static uint32_t ICR2SCLDivider[] = {
	20, 22, 24, 26, 28, 30, 34, 40, 28, 32, 36, 40, 44, 48, 56,
	68, 48, 56, 64, 72, 80, 88, 104, 128, 80, 96, 112, 128, 144,
	160, 192, 240, 160, 192, 224, 256, 288, 320, 384, 480, 320,
	384, 448, 512, 576, 640, 768, 960, 640, 768, 896, 1024, 1152,
	1280, 1536, 1920, 1280, 1536, 1792, 2048, 2304, 2560, 3072, 3840
};

uint8_t dummy;

/*******************************************************************************
 *******************************************************************************
                        GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

void I2C_Init(I2C_Module_t module)
{
//  gpioWrite(PIN_TEST_INIT, HIGH);
  // Clock gating
  if(module == I2C0_M)
    SIM->SCGC4 |= SIM_SCGC4_I2C0(HIGH);
  else if(module == I2C1_M)
    SIM->SCGC4 |= SIM_SCGC4_I2C1(HIGH);
  else if(module == I2C2_M)
    SIM->SCGC1 |= SIM_SCGC1_I2C2(HIGH);


  // Port configuration
  if(module == I2C0_M)
    SIM->SCGC5 |= SIM_SCGC5_PORTE(HIGH);
  else if(module == I2C1_M)
    SIM->SCGC5 |= SIM_SCGC5_PORTE(HIGH);
  else if(module == I2C2_M)
    SIM->SCGC5 |= SIM_SCGC5_PORTA(HIGH);


  I2C_Type* I2C = I2C_arr[module];


  // Baude rate configuration
  BaudRate_t F = SetBaudRate(I2C_BAUD_RATE);
  I2C->F = I2C_F_MULT(F.mul) | I2C_F_ICR(F.icr);

  // Enable I2C and its interrupts
  I2C->C1 = I2C_C1_IICIE(HIGH) | I2C_C1_IICEN(HIGH);

  // Enable I2C interuptiom
  I2C->S = I2C_S_IICIF(HIGH);


  if(module == I2C0_M)
  {
    // SCL config
    Port_arr[PE]->PCR[I2C0_SCL_PIN] = LOW;            // Clear all bits
    Port_arr[PE]->PCR[I2C0_SCL_PIN] = PORT_PCR_MUX(PORT_mAlt5) | PORT_PCR_ODE(HIGH);
    
    // SDA config
    Port_arr[PE]->PCR[I2C0_SDA_PIN] = LOW;            // Clear all bits
    Port_arr[PE]->PCR[I2C0_SDA_PIN] = PORT_PCR_MUX(PORT_mAlt5) | PORT_PCR_ODE(HIGH);
  }
  // Enable I2C interuptiom
  NVIC_EnableIRQ(I2C_IRQn[module]);

  I2C_STATUS = I2C_Done;
//  gpioWrite(PIN_TEST_INIT, LOW);
}


I2C_Status_t I2C_Transmit(I2C_Module_t module, uint8_t * sequence_arr, uint8_t sequence_size, 
                    I2C_Address_t slave_address, I2C_Address_t reg_address, I2C_Mode_t mode)
{
  if(sequence_arr != NULL && sequence_size)
    {
      I2C_Objects[module].slave_address = slave_address;
      I2C_Objects[module].reg_address = reg_address;
      I2C_Objects[module].sequence_arr = sequence_arr;
      I2C_Objects[module].sequence_size = sequence_size;
      I2C_Objects[module].index = 0;
      I2C_Objects[module].reg_address_flag = false;
      I2C_Objects[module].repeated_start = false;
      I2C_Objects[module].mode = mode;
//      gpioWrite(PIN_TEST_TRANSMIT, HIGH);
    }


  I2C_SET_TX_MODE;
  I2C_START_SIGNAL;
  // 1 es R, 0 es W
  I2C_WRITE_DATA(I2C_ADDRESS_MASK);   // always start in write mode
  
  I2C_STATUS = I2C_Busy;
  
//  gpioWrite(PIN_TEST_TRANSMIT, LOW);

  return I2C_Objects[module].status;
}


I2C_Status_t I2C_GetStatus(I2C_Module_t module)
{
  return I2C_STATUS;
}

/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

// Busco el BaudRate mas cercano al deseado 
BaudRate_t SetBaudRate(uint32_t desiredBaudRate)
{
  BaudRate_t settings = {.mul = 0, .icr = 0};
  uint32_t bestBaudRate = 0;
  uint32_t error = 0;
  uint32_t currentBaudRate = 0;
  uint8_t mul;
  uint8_t icr;

  for(mul = 1 ; mul <= 4 ; mul = mul << 1)
  {
    for(icr = 0 ; icr <= 0x3F ; icr++)
    {
      currentBaudRate = I2C_BUS_CLOCK / (mul * ICR2SCLDivider[icr]);
      if(bestBaudRate == 0 || error > ((currentBaudRate > desiredBaudRate) ? 
      currentBaudRate - desiredBaudRate : desiredBaudRate - currentBaudRate))
      {
        bestBaudRate = currentBaudRate;
        error = currentBaudRate > desiredBaudRate ? 
        currentBaudRate - desiredBaudRate : desiredBaudRate - currentBaudRate;
        settings.mul = mul;
        settings.icr = icr;
      }
    }
  }
  return settings;
}


void I2C_IRQHandler(I2C_Module_t module)
{
#if DEBUG_GPIO
D_DEBUG_TP_SET
#endif
  I2C_CLEAR_IRQ_FLAG;

  // En modo transmision
  if(I2C_GET_TX_MODE)
  {
    // If write
    if(I2C_MODE == I2C_Write)
    {
      // si me queda por mandar
      if(I2C_INDEX < I2C_SEQUENCE_SIZE)
      {
        // RXAK = 0 ta bien, RXAK = 1 ta mal
        if(!I2C_GET_RX_ACK)
        {
          // si no mande el reg_address
          if(!I2C_REG_ADDRESS_SENT)
          {
            I2C_WRITE_DATA(I2C_REG_ADDRESS);
            I2C_REG_ADDRESS_SENT = true;
          }
          // si ya mande el addr mando el siguiente byte
          else
          {
            I2C_WRITE_DATA(I2C_SEQUENCE_ARR[I2C_INDEX]);
            I2C_INDEX++;
          }
        }
        // si no me mandaron el ACK
        else
        {
          I2C_STATUS = I2C_Error;
          I2C_STOP_SIGNAL;
        }
      }
      // si ya mande todo
      else if(I2C_INDEX == I2C_SEQUENCE_SIZE)
      {
        I2C_STATUS = I2C_Done;
        I2C_STOP_SIGNAL;
      }
    }
    // If read
    else if(I2C_MODE == I2C_Read)
    {
      // RXAK = 0 ta bien, RXAK = 1 ta mal
      if(!I2C_GET_RX_ACK)
      {
        // si no mande el reg_address
        if(!I2C_REG_ADDRESS_SENT)
        {
          I2C_WRITE_DATA(I2C_REG_ADDRESS);
          I2C_REG_ADDRESS_SENT = true;
        }
        // si ya mande el register address mando el repeated start
        else
        {
          // Si no mande el repeated start
          if(!I2C_REPEATED_START_SENT)
          {
            I2C_REPEAT_START_SIGNAL;
            I2C_WRITE_DATA(I2C_ADDRESS_MASK | 0x00000001); // despues ponerle una macro
            I2C_REPEATED_START_SENT = true;
          }
          else
          {
            I2C_SET_RX_MODE;

            // Si el tamaño es chico ya mando el NACK
            if(I2C_SEQUENCE_SIZE == 1)
            {
              I2C_SET_NACK;
              // gpioWrite(PIN_TEST_INIT, LOW);
            }
            else
              I2C_CLEAR_NACK;

            // Dummy read
            dummy = I2C_READ_DATA;
          }
        }
      }
      else
      {
        I2C_STATUS = I2C_Error;
        I2C_STOP_SIGNAL;
      }
    }
  }
  // en modo lectura
  else
  {
    // Si le resto 1 a el size me da el ultimo
    // Estoy en el ultimo para leer
    if(I2C_INDEX == I2C_SEQUENCE_SIZE - 1)
    {
      I2C_SET_TX_MODE;
      I2C_SEQUENCE_ARR[I2C_INDEX] = I2C_READ_DATA;
      I2C_STOP_SIGNAL;
      I2C_INDEX++;
//      gpioWrite(PIN_TEST_INTERRUPT, LOW);
      I2C_STATUS = I2C_Done;
    }
    // No estoy en el ultimo para leer
    else if(I2C_INDEX < (I2C_SEQUENCE_SIZE - 1))
    {
      // Si le resto 2 me da el anteultimo
      if(I2C_INDEX == (I2C_SEQUENCE_SIZE - 2))
      {
        I2C_SET_NACK;         // Es el NACK, toco TXAK
      }
      else
        I2C_CLEAR_NACK;
      
        I2C_SEQUENCE_ARR[I2C_INDEX] = I2C_READ_DATA;
        I2C_INDEX++;
    }
  }
#if DEBUG_GPIO
D_DEBUG_TP_CLR
#endif
}


__ISR__ I2C0_IRQHandler(void)
{
	I2C_IRQHandler(I2C0_M);
}
