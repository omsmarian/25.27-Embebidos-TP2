/* Simple Serial test program, based on the work of Daniel Jacoby */

#include "board.h"
#include "gpio.h"
#include "hardware.h"
#include "serial.h"
#include "timer.h"

#define __FOREVER__ 	for(;;)

int main (void)
{
	uchar_t start_msg[] = "hello";
	uchar_t* msg = start_msg;
//	uint8_t len = sizeof(start_msg) - 1;
	uint8_t len = 5;
	ticks_t timeout, timeout2;

 	hw_Init();
	hw_DisableInterrupts();

	serialInit();
	serialWriteData(msg, len);

	timerInit();
	timeout = timerStart(TIMER_MS2TICKS(100));
	timeout2 = timerStart(TIMER_MS2TICKS(1000));

//	while(!timerExpired(timeout));
//	msg = serialReadData(&len);

	hw_EnableInterrupts();

	__FOREVER__
	{
		 if(timerExpired(timeout) && !timerExpired(timeout2))
		 {
//			if(serialReadStatus() && serialWriteStatus())
			{
				msg = serialReadData(&len);
				serialWriteData(msg, len);
			}
//			timeout = timerStart(TIMER_MS2TICKS(100));
		 }
	}
}
