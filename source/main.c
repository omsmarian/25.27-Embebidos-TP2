#include "fsl_debug_console.h"
#include "MK64F12.h"
#include "hardware.h"

#include "can_communication.h"





int main(void) {



    /* Init board hardware. */
    BOARD_InitDebugConsole();

    otherBoardCommunicationsInit();

    measurement_t m;

    m.boardID = 0x101;
    m.angleID = 'R';
    m.angleVal = -160;

    sendMeasurement2OtherBoards(m);


    while (1) {
		if (receiveOtherBoardsMeasurement(&m)) {
			PRINTF("Received measurement from board %d: %c %d\n", m.boardID,
					m.angleID, m.angleVal);
		}
	}

}
