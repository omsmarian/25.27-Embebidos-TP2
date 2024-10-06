/* Simple UART test program */

#include "hardware.h"
#include "uart.h"
#include "gpio.h"
#include "board.h"
#include "timer.h"

#define __FOREVER__ 	for(;;)

int main (void)
{
 	hw_Init ();

	uart_cfg_t config = {9600, UART_MODE_8, UART_PARITY_NONE, UART_STOPS_1, UART_RX_TX_ENABLED, UART_FIFO_RX_TX_ENABLED};
	uartInit(UART0_ID, config);

	char msg[] = "hello, world";
	uartWriteMsg(UART0_ID, msg, 12);

	timerInit();
	ticks_t timeout = timerStart(TIMER_MS2TICKS(10));

	// Enable interrupts
	hw_EnableInterrupts();

	__FOREVER__
	{
		// if(timerExpired(timeout))
			if(uartIsRxMsg(UART0_ID))
			{
				uint8_t length = uartGetRxMsgLength(UART0_ID);
				uint8_t a = uartReadMsg(UART0_ID, msg, length);
				uartWriteMsg(UART0_ID, msg, length);
				// timeout = timerStart(TIMER_MS2TICKS(10));
			}
	}
}
