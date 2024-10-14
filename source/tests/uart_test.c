/* Simple UART test program, based on the work of Daniel Jacoby */

#include "board.h"
#include "gpio.h"
#include "hardware.h"
#include "timer.h"
#include "uart.h"

#define __FOREVER__ 	for(;;)

int main3 (void)
{
 	hw_Init();
	hw_DisableInterrupts();

	uart_cfg_t config = {9600, UART_MODE_8, UART_PARITY_NONE, UART_STOPS_1, UART_RX_TX_ENABLED, UART_FIFO_RX_TX_ENABLED};
	uartInit(UART0_ID, config);

	uchar_t msg[] = "hello, world";
	uartWriteMsg(UART0_ID, msg, 12);

	timerInit();
	ticks_t timeout = timerStart(TIMER_MS2TICKS(10));
	ticks_t timeout2 = timerStart(TIMER_MS2TICKS(3000));

	hw_EnableInterrupts();

	__FOREVER__
	{
		 if(!timerExpired(timeout2))
			if(uartIsRxMsg(UART0_ID))
			{
				uint8_t length = uartGetRxMsgLength(UART0_ID);
				uint8_t tmp = uartReadMsg(UART0_ID, msg, length);
				uartWriteMsg(UART0_ID, msg, length);
				// timeout = timerStart(TIMER_MS2TICKS(10));
			}
	}
}
