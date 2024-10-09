/***************************************************************************//**
  @file     I2C.c
  @brief    +Descripcion del archivo+
  @author   Mariano Oms
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include "I2C.h"
#include "hardware.h"
#include "gpio.h"

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
#define I2C_SET_NACK	         	  (I2C_array[module]->C1 |= I2C_C1_TXAK_MASK)
#define I2C_CLEAR_NACK      	 	  (I2C_array[module]->C1 &= ~I2C_C1_TXAK_MASK)
#define I2C_START_SIGNAL        	(I2C_array[module]->C1 |= I2C_C1_MST_MASK)
#define I2C_STOP_SIGNAL          	(I2C_array[module]->C1 &= ~I2C_C1_MST_MASK)
#define I2C_REPEAT_START_SIGNAL 	(I2C_array[module]->C1 |= I2C_C1_RSTA_MASK)
#define I2C_READ_DATA           	(I2C_array[module]->D)
#define I2C_STATUS_BYTE           (I2C_array[module]->S)
#define I2C_GET_TX_MODE				    (I2C_array[module]->C1 & I2C_C1_TX_MASK)
#define I2C_GET_IRQ_FLAG        	(I2C_array[module]->S & I2C_S_IICIF_MASK)
#define I2C_CLEAR_IRQ_FLAG      	(I2C_array[module]->S |= I2C_S_IICIF_MASK)
#define I2C_GET_RX_ACK				    (I2C_array[module]->S & I2C_S_RXAK_MASK)    // 0 = ACK received, 1 = no ACK received
#define I2C_SET_RX_MODE         	(I2C_array[module]->C1 &= ~I2C_C1_TX_MASK)
#define I2C_SET_TX_MODE         	(I2C_array[module]->C1 |= I2C_C1_TX_MASK)
#define I2C_SET_NACK	         	  (I2C_array[module]->C1 |= I2C_C1_TXAK_MASK)
#define I2C_CLEAR_NACK      	 	  (I2C_array[module]->C1 &= ~I2C_C1_TXAK_MASK)



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



/*******************************************************************************
 *******************************************************************************
                        GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

void I2C_Init(I2C_Module_t module)
{

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


  /*if(module == I2C0_M)
  {
    // SCL config
    Port_arr[PE]->PCR[I2C0_SCL_PIN] = LOW;            // Clear all bits
    Port_arr[PE]->PCR[I2C0_SCL_PIN] = PORT_PCR_MUX(PORT_mAlt5) | PORT_PCR_ODE(HIGH);
    Port_arr[PE]->PCR[I2C0_SCL_PIN] = PORT_PCR_IRQC(PORT_eDisabled);
    
    // SDA config
    Port_arr[PE]->PCR[I2C0_SDA_PIN] = LOW;            // Clear all bits
    Port_arr[PE]->PCR[I2C0_SDA_PIN] = PORT_PCR_MUX(PORT_mAlt5) | PORT_PCR_ODE(HIGH);
    Port_arr[PE]->PCR[I2C0_SDA_PIN] = PORT_PCR_IRQC(PORT_eDisabled);
  }
*/
  // Enable I2C interuptiom
  NVIC_EnableIRQ(I2C_IRQn[module]);
}


I2C_Status_t I2C_Transmit(I2C_Module_t module, uint8_t * sequence_arr, size_t sequence_size, 
                    I2C_Address_t slave_address, I2C_Address_t reg_address, I2C_Mode_t mode);
{
  if(sequence_arr != NULL && sequence_size)
    {
      I2C_Objects[module].slave_address = slave_address;
      I2C_Objects[module].reg_address = reg_address;
      I2C_Objects[module].sequence_arr = sequence_arr;
      I2C_Objects[module].sequence_size = sequence_size;
      I2C_Objects[module].index = 0;
      I2C_Objects[module].mode = mode;
    }


  // 1 es R, 0 es W
  I2C_SET_TX_MODE;
  I2C_START_SIGNAL;
  I2C_WRITE_DATA(I2C_ADDRESS_MASK);   // always start in write mode
  I2C_Objects[module].status = I2C_Busy;

  return I2C_Objects[module].status;
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
}


void I2C_IRQHandler(I2C_Module_t module)
{
  I2C_CLEAR_IRQ_FLAG;

  if(I2C_GET_TX_MODE)
  {
    
  }
  else
}