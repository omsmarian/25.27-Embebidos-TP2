/***************************************************************************//**
  @file     magcard.c
  @brief    UART Driver for K64F. Blocking, Non-Blocking and using FIFO feature
  @author   Group 4
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include <stdbool.h>

#include "hardware.h"
#include "MK64F12.h"

#include "board.h"
#include "cqueue.h"
#include "gpio.h"
#include "pisr.h"
#include "timer.h"
#include "uart.h"


/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

#define DEVELOPMENT_MODE			1

#define PORT_REG(port, reg)			(PORT_Ptrs[port]->reg)

#define UART_MAX_BAUDRATE			(__CORE_CLOCK__ / 16)
#define UART_HAL_DEFAULT_BAUDRATE	9600
#define UART_REG(id, reg)			(UART_Ptrs[id]->reg)

#define DEBUG_TP					1											// Debugging Test Points to measure ISR time


/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/

void uartHandler (void);
void uartUpdate (uart_id_t id);
void uartSetBaudRate (uart_id_t id, uint32_t br);


/*******************************************************************************
 * STATIC VARIABLES AND CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

static PORT_Type * const	PORT_Ptrs[]		=   PORT_BASE_PTRS;
static UART_Type * const	UART_Ptrs[]		=   UART_BASE_PTRS;
static uint32_t				UART_Clks[]		= { SIM_SCGC4_UART0_MASK,
												SIM_SCGC4_UART1_MASK,
												SIM_SCGC4_UART2_MASK,
												SIM_SCGC4_UART3_MASK,
												SIM_SCGC1_UART4_MASK,
												SIM_SCGC1_UART5_MASK };
static uint8_t const        UART_IRQn[]		=   UART_RX_TX_IRQS;
static pin_t const			UARTx_PINS[]	= { UART0_RX_PIN, UART0_TX_PIN,
											 	UART1_RX_PIN, UART1_TX_PIN,
											 	UART2_RX_PIN, UART2_TX_PIN,
											 	UART3_RX_PIN, UART3_TX_PIN,
											 	UART4_RX_PIN, UART4_TX_PIN,
											 	UART5_RX_PIN, UART5_TX_PIN };

static bool uart_init[UART_CANT_IDS];
static queue_id_t uart_rx_queue[UART_CANT_IDS];
static queue_id_t uart_tx_queue[UART_CANT_IDS];

// static tim_id_t uart_timers[UART_CANT_IDS];

static uart_id_t uart_irq = UART_CANT_IDS;


/*******************************************************************************
 *******************************************************************************
						GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

bool uartInit (uart_id_t id, uart_cfg_t config)
{
	if(!uart_init[id])
	{
		PINData_t pinRx = { PIN2PORT(UARTx_PINS[id * 2]), PIN2NUM(UARTx_PINS[id * 2]) };
		PINData_t pinTx = { PIN2PORT(UARTx_PINS[id * 2 + 1]), PIN2NUM(UARTx_PINS[id * 2 + 1]) };
		uint32_t * pinRxPCR = &(PORT_Ptrs[pinRx.port]->PCR[pinRx.num]);
		uint32_t * pinTxPCR = &(PORT_Ptrs[pinTx.port]->PCR[pinTx.num]);

		/* Enable the clock for UARTx */
		(id < 4) ? (SIM->SCGC4 |= UART_Clks[id]) : (SIM->SCGC1 |= UART_Clks[id]);

		/* Disable Rx and Tx while settings are changed */
		UART_REG(id, C2) &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK);

		/* Configure the mode and parity for UARTx */
		UART_REG(id, C1) = UART_C1_M(config.mode) |
						   UART_C1_PE(config.parity != UART_PARITY_NONE) | UART_C1_PT(config.parity);

		/* Set UARTx to default speed */
		uartSetBaudRate(id, UART_HAL_DEFAULT_BAUDRATE);

		/* Configure UARTx Rx and Tx pins */
		*pinRxPCR = 0x0;															//Clear all bits
		*pinTxPCR = 0x0;
		*pinRxPCR |= PORT_PCR_MUX(PORT_mAlt3); 										//Set MUX to UARTx
		*pinTxPCR |= PORT_PCR_MUX(PORT_mAlt3);
		*pinRxPCR |= PORT_PCR_IRQC(PORT_eDisabled);									//Disable interrupts (UART IRQs will handle them)
		*pinTxPCR |= PORT_PCR_IRQC(PORT_eDisabled);

		/* Baudrate Setup */
		uartSetBaudRate(id, config.baudrate);

		/* Enable FIFO and configure watermarks */
		UART_REG(id, CFIFO) |= UART_CFIFO_RXFLUSH_MASK | UART_CFIFO_TXFLUSH_MASK;
		if(config.fifo == UART_FIFO_RX_ENABLED || config.fifo == UART_FIFO_RX_TX_ENABLED)
		{
			UART_REG(id, PFIFO) |= UART_PFIFO_RXFE_MASK;
			uint8_t depth = (UART_REG(id, PFIFO) & UART_PFIFO_RXFIFOSIZE_MASK) >> UART_PFIFO_RXFIFOSIZE_SHIFT;
			UART_REG(id, RWFIFO) = UART_RWFIFO_RXWATER((85 * (depth ? (2 << depth) : 1)) / 100);
		}
		if(config.fifo == UART_FIFO_TX_ENABLED || config.fifo == UART_FIFO_RX_TX_ENABLED)
		{
			UART_REG(id, PFIFO) |= UART_PFIFO_TXFE_MASK;
			uint8_t depth = (UART_REG(id, PFIFO) & UART_PFIFO_TXFIFOSIZE_MASK) >> UART_PFIFO_TXFIFOSIZE_SHIFT;
			UART_REG(id, TWFIFO) = UART_TWFIFO_TXWATER(((15 * (depth ? (2 << depth) : 1)) / 100) + 1);
		}
		if(config.fifo == UART_FIFO_DISABLED)
			UART_REG(id, PFIFO) &= ~(UART_PFIFO_RXFE_MASK | UART_PFIFO_TXFE_MASK);

		/* Enable Tx and Rx IRQs for UARTx */
		NVIC_EnableIRQ(UART_IRQn[id]);

		/* Enable UARTx Rx and Tx (and interrupts) */
		if(config.RxTx != UART_RX_ENABLED)
			UART_REG(id, C2) |= (UART_C2_TE_MASK); // | UART_C2_TIE_MASK);
		if(config.RxTx != UART_TX_ENABLED)
			UART_REG(id, C2) |= (UART_C2_RE_MASK); // | UART_C2_RIE_MASK);

		/* Create queues for UARTx */
		uart_rx_queue[id] = queueInit();
		uart_tx_queue[id] = queueInit();

		/* Register PISR */
		pisrRegister(uartHandler, PISR_FREQUENCY_HZ / UART_FREQUENCY_HZ);

		/* Set up a timer to update the queues */
		// timerInit();
		// timerStart(timerGetId(), TIMER_MS2TICKS(1), TIMER_MODE_PERIODIC, uartHandler);

		uart_init[id] = true;
	}

	return uart_init[id];
}

// Main Services ///////////////////////////////////////////////////////////////

uint8_t uartIsRxMsg (uart_id_t id)
{
	return !queueIsEmpty(uart_rx_queue[id]);
}

uint8_t uartGetRxMsgLength (uart_id_t id)
{
	return queueSize(uart_rx_queue[id]);
}

uint8_t uartReadMsg (uart_id_t id, char* msg, uint8_t cant)
{
	uint8_t i = 0;
	while (i < cant && !queueIsEmpty(uart_rx_queue[id]))
		msg[i++] = queuePop(uart_rx_queue[id]);

	return i;
}

uint8_t uartWriteMsg (uart_id_t id, const char* msg, uint8_t cant)
{
	uint8_t i = 0;
	while (i < cant && !queueIsFull(uart_tx_queue[id]))
		queuePush(uart_tx_queue[id], msg[i++]);

	return i;
}

uint8_t uartIsTxMsgComplete (uart_id_t id)
{
	return queueIsEmpty(uart_tx_queue[id]);
}


/*******************************************************************************
 *******************************************************************************
						LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

// ISR Functions ///////////////////////////////////////////////////////////////

__ISR__ UART0_RX_TX_IRQHandler (void) {	uart_irq = UART0_ID; uartHandler(); }
__ISR__ UART1_RX_TX_IRQHandler (void) {	uart_irq = UART1_ID; uartHandler(); }
__ISR__ UART2_RX_TX_IRQHandler (void) {	uart_irq = UART2_ID; uartHandler(); }
__ISR__ UART3_RX_TX_IRQHandler (void) {	uart_irq = UART3_ID; uartHandler(); }
__ISR__ UART4_RX_TX_IRQHandler (void) {	uart_irq = UART4_ID; uartHandler(); }
__ISR__ UART5_RX_TX_IRQHandler (void) {	uart_irq = UART5_ID; uartHandler(); }

void uartHandler (void)															// Separate so that it can be called from the PISR/timer as well
{
	if(uart_irq != UART_CANT_IDS)
	{
		uartUpdate(uart_irq);
		uart_irq = UART_CANT_IDS;
	}
	else
		for(uint8_t id = 0; id < UART_CANT_IDS; id++)
			if(uart_init[id])
				uartUpdate(id);
}

void uartUpdate (uart_id_t id)
{
	uint8_t status = UART_REG(id, S1);											// Always needed (clears status register)

	uint8_t count = UART_REG(id, RCFIFO);
	while(count-- && !queueIsFull(uart_rx_queue[id]))
		queuePush(uart_rx_queue[id], UART_REG(id, D));

	count = UART_REG(id, TCFIFO);
	while((count++ != ((UART_REG(id, PFIFO) & UART_PFIFO_TXFIFOSIZE_MASK) >> UART_PFIFO_TXFIFOSIZE_SHIFT))
		&& !queueIsEmpty(uart_tx_queue[id]))
		UART_REG(id, D) = queuePop(uart_tx_queue[id]);

	// // while(!(UART_REG(id, S1) & UART_SFIFO_RXEMPT_MASK) && !queueIsFull(uart_rx_queue[id]))
	// while((UART_REG(id, S1) & UART_S1_RDRF_MASK) && !queueIsFull(uart_rx_queue[id]))
	// 	queuePush(uart_rx_queue[id], UART_REG(id, D));

	// while((UART_REG(id, S1) & UART_S1_TDRE_MASK) && !queueIsEmpty(uart_tx_queue[id]))
	// 	UART_REG(id, D) = queuePop(uart_tx_queue[id]);
} // S1 could be read in the dedicated irq and set a flag checked by the periodic isr

////////////////////////////////////////////////////////////////////////////////

void uartSetBaudRate (uart_id_t id, uint32_t br)
{
	uint8_t brfa;
	uint16_t sbr;
	uint32_t clock;

	clock = ((id == UART0_ID) || (id == UART1_ID)) ? (__CORE_CLOCK__) : (__CORE_CLOCK__ >> 1);

	if ((br == 0) || (br > UART_MAX_BAUDRATE))
		br = UART_HAL_DEFAULT_BAUDRATE;

	/* Calculate baud settings */
	sbr = (uint16_t)(clock / (br << 4)); // sbr = clock / (baudrate * 16)

	/* Determine if a fractional divider is needed to get closer to the baud rate */
	brfa = (uint8_t)((uint16_t)((clock << 1) / br) - (sbr << 5)); // brfa = 2 * clock / baudrate - 32 * sbr

	/* Save off the current value of the UARTx_BDH except for the SBR */
	UART_REG(id, BDH) = (UART_REG(id, BDH) & ~UART_BDH_SBR_MASK) | UART_BDH_SBR(sbr >> 8);

	/* Write the sbr to the UARTx_BDL */
	UART_REG(id, BDL) = UART_BDL_SBR(sbr);

	/* Save off the current value of the UARTx_C4 register except for the BRFA */
	UART_REG(id, C4) = (UART_REG(id, C4) & ~UART_C4_BRFA_MASK) | UART_C4_BRFA(brfa);
}

/*
void uartSetParity (uart_id_t id, uart_parity_t parity)
{
	UART_REG(id, C1) = (UART_REG(id, C1) & ~UART_C1_PE_MASK) | UART_C1_PE(parity != UART_PARITY_NONE);
	UART_REG(id, C1) = (UART_REG(id, C1) & ~UART_C1_PT_MASK) | UART_C1_PT(parity);
}

void uartSetMode (uart_id_t id, uart_mode_t mode)
{
	UART_REG(id, C1) = (UART_REG(id, C1) & ~UART_C1_M_MASK) | UART_C1_M(mode);
}

void uartSetStops (uart_id_t id, uart_stops_t stops)
{
	UART_REG(id, BDH) = (UART_REG(id, BDH) & ~UART_BDH_SBNS_MASK) | UART_BDH_SBNS(stops);
}

void uartSetRxTx (uart_id_t id, uart_rx_tx_t RxTx)
{
	UART_REG(id, C2) = (UART_REG(id, C2) & ~UART_C2_TE_MASK) | UART_C2_TE(RxTx != UART_RX_ENABLED);
	UART_REG(id, C2) = (UART_REG(id, C2) & ~UART_C2_RE_MASK) | UART_C2_RE(RxTx != UART_TX_ENABLED);
}

void uartSetFIFO (uart_id_t id, uart_fifo_t fifo)
{
	UART_REG(id, PFIFO) = (UART_REG(id, PFIFO) & ~UART_PFIFO_TXFIFOSIZE_MASK) | UART_PFIFO_TXFIFOSIZE(fifo);
	UART_REG(id, PFIFO) = (UART_REG(id, PFIFO) & ~UART_PFIFO_RXFIFOSIZE_MASK) | UART_PFIFO_RXFIFOSIZE(fifo);
}

void uartSetWatermarks (uart_id_t id, uint8_t tx, uint8_t rx)
{
	UART_REG(id, PFIFO) = (UART_REG(id, PFIFO) & ~UART_PFIFO_TXWATER_MASK) | UART_PFIFO_TXWATER(tx);
	UART_REG(id, PFIFO) = (UART_REG(id, PFIFO) & ~UART_PFIFO_RXWATER_MASK) | UART_PFIFO_RXWATER(rx);
}

void uartSetIRQs (uart_id_t id, uint8_t tx, uint8_t rx)
{
	UART_REG(id, C2) = (UART_REG(id, C2) & ~UART_C2_TIE_MASK) | UART_C2_TIE(tx);
	UART_REG(id, C2) = (UART_REG(id, C2) & ~UART_C2_RIE_MASK) | UART_C2_RIE(rx);
}

void uartSetLoopback (uart_id_t id, bool enable)
{
	UART_REG(id, C1) = (UART_REG(id, C1) & ~UART_C1_LOOPS_MASK) | UART_C1_LOOPS(enable);
}

void uartSetSmartCard (uart_id_t id, bool enable)
{
	UART_REG(id, C3) = (UART_REG(id, C3) & ~UART_C3_SCEN_MASK) | UART_C3_SCEN(enable);
}

void uartSet9Bits (uart_id_t id, bool enable)
{
	UART_REG(id, C1) = (UART_REG(id, C1) & ~UART_C1_M_MASK) | UART_C1_M(enable);
}

void uartSetIrDA (uart_id_t id, bool enable)
{
	UART_REG(id, C3) = (UART_REG(id, C3) & ~UART_C3_IRTX_MASK) | UART_C3_IRTX(enable);
}

void uartSetHalfDuplex (uart_id_t id, bool enable)
{
	UART_REG(id, C1) = (UART_REG(id, C1) & ~UART_C1_LOOPS_MASK) | UART_C1_LOOPS(enable);
}

void uartSetWakeUp (uart_id_t id, bool enable)
{
	UART_REG(id, C1) = (UART_REG(id, C1) & ~UART_C1_WAKE_MASK) | UART_C1_WAKE(enable);
}

void uartSetIdleLine (uart_id_t id, bool enable)
{
	UART_REG(id, C1) = (UART_REG(id, C1) & ~UART_C1_ILT_MASK) | UART_C1_ILT(enable);
}

void uartSetRxDMA (uart_id_t id, bool enable)
{
	UART_REG(id, C5) = (UART_REG(id, C5) & ~UART_C5_RDMAE_MASK) | UART_C5_RDMAE(enable);
}

void uartSetTxDMA (uart_id_t id, bool enable)
{
	UART_REG(id, C5) = (UART_REG(id, C5) & ~UART_C5_TDMAE_MASK) | UART_C5_TDMAE(enable);
}

void uartSetOversampling (uart_id_t id, bool enable)
{
	UART_REG(id, C4) = (UART_REG(id, C4) & ~UART_C4_OSR_MASK) | UART_C4_OSR(enable);
}

void uartSetInfrared (uart_id_t id, bool enable)
{
	UART_REG(id, C4) = (UART_REG(id, C4) & ~UART_C4_IREN_MASK) | UART_C4_IREN(enable);
}

void uartSetIdleLineType (uart_id_t id, bool enable)
{
	UART_REG(id, C4) = (UART_REG(id, C4) & ~UART_C4_ILT_MASK) | UART_C4_ILT(enable);
}

void uartSetBreakChar (uart_id_t id, bool enable)
{
	UART_REG(id, C1) = (UART_REG(id, C1) & ~UART_C1_SBK_MASK) | UART_C1_SBK(enable);
}

void uartSetMatchChar (uart_id_t id, bool enable)
{
	UART_REG(id, C3) = (UART_REG(id, C3) & ~UART_C3_MAEN1_MASK) | UART_C3_MAEN1(enable);
}

void uartSetMatchChar2 (uart_id_t id, bool enable)
{
	UART_REG(id, C3) = (UART_REG(id, C3) & ~UART_C3_MAEN2_MASK) | UART_C3_MAEN2(enable);
}
*/


/******************************************************************************/
