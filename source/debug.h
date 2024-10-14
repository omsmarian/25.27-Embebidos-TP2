/***************************************************************************//**
  @file     protocol.h
  @brief    Data frame packer and unpacker, using protocol '{Id}[ValSign]{Val}'
  @author   Group 4: - Oms, Mariano
                     - Solari Raigoso, Agustín
                     - Wickham, Tomás
                     - Vieira, Valentin Ulises
 ******************************************************************************/

#ifndef _PROTOCOL_H_
#define _PROTOCOL_H_

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

/* Manual write for debugging (measure ISR time) */

// Periodic ISR ////////////////////////////////////////////////////////////////

#define P_SET_ADDRS			(0x400FF084u) // PTB9
#define P_SET_PTR			((uint32_t *)P_SET_ADDRS)
#define P_SET_VAL			(1<<9)

#define P_CLR_ADDRS			(0x400FF088u)
#define P_CLR_PTR			((uint32_t *)P_CLR_ADDRS)
#define P_CLR_VAL			(1<<9)
#define P_DEBUG_TP_SET		*(P_SET_PTR) = P_SET_VAL
#define P_DEBUG_TP_CLR		*(P_CLR_PTR) = P_CLR_VAL

// Dedicated ISR ///////////////////////////////////////////////////////////////

#define D_SET_ADDRS			(0x400FF084u) // PTB5
#define D_SET_PTR			((uint32_t *)D_SET_ADDRS)
#define D_SET_VAL			(1<<5)

#define D_CLR_ADDRS			(0x400FF088u)
#define D_CLR_PTR			((uint32_t *)D_CLR_ADDRS)
#define D_CLR_VAL			(1<<5)
#define D_DEBUG_TP_SET		*(D_SET_PTR) = D_SET_VAL
#define D_DEBUG_TP_CLR		*(D_CLR_PTR) = D_CLR_VAL

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

// Debugging test points ///////////////////////////////////////////////////////

enum {
	DEBUG_CAN		= 0,
	DEBUG_GPIO		= 0,
	DEBUG_I2C		= 0,
	DEBUG_PISR		= 0,
	DEBUG_SENSOR	= 0,
	DEBUG_TIMER		= 0,
	DEBUG_UART		= 0,
};

/*******************************************************************************
 ******************************************************************************/

#endif // _PROTOCOL_H_
