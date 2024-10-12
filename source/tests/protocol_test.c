/* Simple Serial test program, based on the work of Daniel Jacoby */

#include <stdio.h>

// #include "board.h"
// #include "gpio.h"
// #include "hardware.h"
#include "protocol.h"
// #include "sensor.h"
// #include "serial.h"
// #include "timer.h"

#define __FOREVER__ 	for(;;)

int main (void)
{
	uchar_t start_msg[] = "hello";
	uchar_t* msg = start_msg;
//	uint8_t len = sizeof(start_msg) - 1;
	uint8_t len = 5;
	// ticks_t timeout, timeout2;
	protocol_t angle_test = { PITCH, -15398 };

 	// hw_Init();
	// hw_DisableInterrupts();

	// serialInit();
	// serialWriteData(msg, len);

	// timerInit();
	// timeout = timerStart(TIMER_MS2TICKS(100));
	// timeout2 = timerStart(TIMER_MS2TICKS(1000));

//	sensorInit();

//	while(!timerExpired(timeout));
//	msg = serialReadData(&len);

	// hw_EnableInterrupts();

	printf("Original angles: %d, %d\n", angle_test.angleId, angle_test.angleVal);
	msg = protocolPack(&angle_test);
	printf("Packed message: %s\n", msg);
	angle_test = *protocolUnpack(msg);
	printf("Unpacked angles: %d, %d\n", angle_test.angleId, angle_test.angleVal);

	return 0;

	__FOREVER__
	{

		// sensor_t data = readAcceleration();
//		raw_data_t* accel = sensorGetAccelRawData();
//		raw_data_t* magn = sensorGetMagnRawData();

//		if(timerExpired(timeout) && !timerExpired(timeout2))
//		{
////			if(serialReadStatus() && serialWriteStatus())
//			{
//				serialWriteData(msg, len);
//				msg = serialReadData(&len);
//			}
////			timeout = timerStart(TIMER_MS2TICKS(100));
//		}
	}
}
