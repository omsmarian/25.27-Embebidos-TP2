//#include "fsl_debug_console.h"
#include <old/can_communication.h>
#include "MK64F12.h"
#include "hardware.h"

#include "timer.h"

int main(void)
{
    /* Init board hardware. */
//    BOARD_InitDebugConsole();

	timerInit();

	ticks_t timeout = timerStart(TIMER_MS2TICKS(1000));

    otherBoardCommunicationsInit();

    measurement_t m;

    m.boardID = 0x105;
    m.angleID = 'C';
    m.angleVal = 12;

    sendMeasurement2OtherBoards(m);

    timerDelay(TIMER_MS2TICKS(10));

//    read();

    while (1)
    {
//		if (timerExpired(timeout))
		{
			if (receiveOtherBoardsMeasurement(&m))
    		{
//				PRINTF("Received measurement from board %d: %c %d\n", m.boardID, m.angleID, m.angleVal);
    			m;
			}
//			read();
//    		timeout = timerStart(TIMER_MS2TICKS(10));
		    timerDelay(TIMER_MS2TICKS(10));
		}
	}
}
