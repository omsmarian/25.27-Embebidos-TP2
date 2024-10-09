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

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/




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

static uint32_t ICR2SCLDivider[] = {
	20, 22, 24, 26, 28, 30, 34, 40, 28, 32, 36, 40, 44, 48, 56,
	68, 48, 56, 64, 72, 80, 88, 104, 128, 80, 96, 112, 128, 144,
	160, 192, 240, 160, 192, 224, 256, 288, 320, 384, 480, 320,
	384, 448, 512, 576, 640, 768, 960, 640, 768, 896, 1024, 1152,
	1280, 1536, 1920, 1280, 1536, 1792, 2048, 2304, 2560, 3072, 3840
};

// Declaring the baud rate configuration structure
typedef struct {
  uint8_t mul; 	// Multiplier Factor, register value
  uint8_t icr;	// ClockRate, register value
} BaudRate_t;

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
  SIM->SCG5 |= SIM_SCG5_PORTA(HIGH) | SIM_SCG5_PORTB(HIGH) | 
              SIM_SCG5_PORTC(HIGH) | SIM_SCG5_PORTD(HIGH) | SIM_SCG5_PORTE(HIGH);

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


int32_t I2C_Trasmit()
{
  
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
 