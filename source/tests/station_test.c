/* Simple CAN_communications test program */

#include "board.h"
#include "can_communication.h"
#include "gpio.h"
#include "hardware.h"
#include "macros.h"
#include "protocol.h"
#include "serial.h"
#include "station.h"
#include "timer.h"

#define __FOREVER__ 	for(;;)

int main3 (void)
{
	uchar_t start_msg[] = "hello";
	uchar_t* msg = start_msg;
//	uint8_t len = sizeof(start_msg) - 1;
	uint8_t len = 5;
	ticks_t timeout, timeout2;
	measurement_t m = { 0x101, 'R', -160 };
	protocol_t p = { m.angleID, m.angleVal };
	uchar_t* p = protocolPack(&p);

 	hw_Init();
	hw_DisableInterrupts();

	otherBoardCommunicationsInit();
	sendMeasurement2OtherBoards(m);

	// stationInit();
	// stationSendAll(p, PROTOCOL_DIGS);

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
		if(!timerExpired(timeout2))
		{
			if(timerExpired(timeout))
			{
				sendMeasurement2OtherBoards(m);
				// stationSendAll(p, PROTOCOL_DIGS);
	//			if(serialReadStatus() && serialWriteStatus())
			
				if (receiveOtherBoardsMeasurement(&m))
				// if (stationReceive(&p) != STATIONS_CANT)
				{
//					uchar_t id = m.boardID;
//					serialWriteData(&id, 1);
					protocol_t p = { m.angleID, m.angleVal };
					serialWriteData(protocolPack(&p), PROTOCOL_DIGS);
				}
	//			timeout = timerStart(TIMER_MS2TICKS(100));
			}
		}
	}

	return 0;
}
