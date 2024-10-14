/*
 * can_communication.c
 *
 *  Created on: 10 oct 2024
 *      Author: asolari
 */

#include <old/can_communication.h>
#include <stdlib.h>

#include "macros.h"

typedef unsigned char uchar_t;

// Inicializa el buffer circular
CAN_DataFrame getFromBuffer(CircularBuffer *cb);
// Añade un frame al buffer circular
void addToBuffer(CircularBuffer *cb, CAN_DataFrame frame);
// Inicializa el buffer circular
void initBuffer(CircularBuffer *cb);

uchar_t* itos (int16_t num);

CAN_DataFrame frame;

void otherBoardCommunicationsInit(void)
{
    // Inicializar el módulo CAN
    if (!CAN_Init())
    {
//        PRINTF("Error al inicializar el módulo CAN\n");
        return;
    }

    // Inicializar los buffers de recepción
    for(int i=0; i<8; i++)
    {
        CAN_ConfigureRxMB(i, 0x100 + i);

        //Enable the interrupt
        CAN_EnableMbInterrupts(i);
    }

    // Inicializar el buffer de transmisión
    CAN_ConfigureTxMB(5);
}

void sendMeasurement2OtherBoards(measurement_t m)
{
    CAN_DataFrame frame;

    frame.ID = m.boardID;

    frame.data[0] = m.angleID;

    //Clean frame data
	for (int i = 1; i < 8; i++) {
		frame.data[i] = 0;
	}

    // Convertir angleVal a una cadena de caracteres
//    char str[5];
//    int len = sprintf(str, "%d", m.angleVal);
	uchar_t* str = itos(m.angleVal);

    // Copiar la cadena de caracteres al frame, sin el carácter de terminación
    for (int i = 0; i < 3; i++) {
        frame.data[i + 1] = str[i];
    }

    // Ajustar la longitud del frame
    frame.length = 4;

    // Enviar el frame
    CAN_WriteTxMB(5 , &frame);
}

uchar_t* itos (int16_t num)
{
	static uchar_t chars[3];

	for (uint8_t i = 0; i < 3; i++)
		chars[i] = NUM2ASCII(0);

	for (uint8_t i = 0; i < 3; i++)
	{
		chars[2 - i] = NUM2ASCII(ABS(num) % 10);
		num /= 10;
	}

	return chars;
}

bool receiveOtherBoardsMeasurement(measurement_t * m)
{
    // Leer el frame del buffer circular
//    CAN_DataFrame frame = getFromBuffer(&cb);
	CAN_DataFrame frame;
	CAN_ReadRxMB(3, &frame);

    // Si el frame está vacío, el buffer estaba vacío
    if (frame.ID == 0 && frame.length == 0)
    {
        return false;
    }

    m->boardID = frame.ID;
    m->angleID = frame.data[0];

    // Convertir el string a int
    char str[4];
    str[0] = frame.data[1];
    str[1] = frame.data[2];
    str[2] = frame.data[3];
    str[3] = frame.data[4];

    m->angleVal = atoi(str);

    return true;
}

bool read (void)
{
	return CAN_ReadRxMB(1, &frame);										// Read frame from reception buffer
}
