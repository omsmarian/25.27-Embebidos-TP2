/* Simple Serial test program, based on the work of Daniel Jacoby */

#include <stdio.h>

#include "board.h"
#include "gpio.h"
#include "hardware.h"
#include "protocol.h"
#include "sensor.h"
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

	sensorInit();

//	while(!timerExpired(timeout));
//	msg = serialReadData(&len);

	hw_EnableInterrupts();

	sensorConfig();

	__FOREVER__
	{

//		if (sensorGetStatus(AXIS_CANT))
		if (sensorGetStatus(ROLL))
		{
			sensor_t* data = sensorGetAngleData();
			protocol_t p[] = { { .angleId = ROLL,	.angleVal = data->roll },
							   { .angleId = PITCH,	.angleVal = data->pitch },
							   { .angleId = YAW,	.angleVal = data->yaw } };
//			raw_data_t* accel = sensorGetAccelRawData();
//			raw_data_t* magn = sensorGetMagnRawData();

			if(timerExpired(timeout))
			{
//				if(serialReadStatus() && serialWriteStatus())
				{
					*msg = ' ';
					serialWriteData(protocolPack(p), PROTOCOL_DIGS);
					serialWriteData(msg, 1);
					serialWriteData(protocolPack(p + 1), PROTOCOL_DIGS);
					serialWriteData(msg, 1);
					serialWriteData(protocolPack(p + 2), PROTOCOL_DIGS);
					*msg = 10;
					serialWriteData(msg, 1);
					// msg = serialReadData(&len);
				}
				timeout = timerStart(TIMER_MS2TICKS(20));
			}
		}
	}
}
