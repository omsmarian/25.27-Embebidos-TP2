/*
 * can_communication.h
 *
 *  Created on: 11 oct 2024
 *      Author: asolari
 */

#ifndef CAN_COMMUNICATION_H_
#define CAN_COMMUNICATION_H_

#include "can.h"
#include "stdbool.h"
#include "stdio.h"


typedef struct
{
    uint16_t boardID;
    uint8_t angleID;     ///    ID representing roll (rolido 'R') o pitch (cabeceo 'C')
    int16_t angleVal;
}measurement_t;


/**
 * @brief Inicializa el módulo CAN y configura los buffers
 * de recepción y transmisión.
 * Buffers de 0 a 7 para recepción y 8 para transmisión.
 * @return void
 */

void otherBoardCommunicationsInit(void);

/**
 * @brief Envía una medición a las otras placas.
 * @param m Medición a enviar.
 * @return void
 */
void sendMeasurement2OtherBoards(measurement_t m);

/**
 * @brief Recibe una medición de otra placa.
 * @param m Puntero a la estructura donde se almacenará la medición.
 * @return bool
 */
bool receiveOtherBoardsMeasurement(measurement_t * m);

bool read (void);

#endif /* CAN_COMMUNICATION_H_ */
