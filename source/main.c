#include "can.h"
#include "fsl_debug_console.h"
#include "MK64F12.h"
#include "hardware.h"


can_msg_t data;



int main(void) {
    /* Init board hardware. */
    BOARD_InitDebugConsole();

    // Inicializar el módulo CAN
    if (!can_init())
    {
        PRINTF("Error al inicializar el módulo CAN\n");
        return 1;
    }

    // Crear un mensaje CAN para enviar
    data.id = 0x123;
    data.len = 8;
    data.data[0] = 0x01;
    data.data[1] = 0x02;
    data.data[2] = 0x03;
    data.data[3] = 0x04;
    data.data[4] = 0x05;
    data.data[5] = 0x06;
    data.data[6] = 0x07;
    data.data[7] = 0x08;


    //Enviar un mensaje CAN
	if (can_tx_msg(&data)) {
		PRINTF("Mensaje enviado\n");
	} else {
		PRINTF("Error al enviar el mensaje\n");
	}

    return 0;
}

__ISR__ CAN0_ORed_Message_buffer_IRQHandler(void) {
    // Crear una variable para almacenar el mensaje recibido
    can_msg_t received_msg;
    PRINTF("ISR\n");

    // Leer el mensaje del buffer de recepción
    if (can_rx_msg(&received_msg)) {
        PRINTF("Mensaje recibido: ID = 0x%x, Datos = 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n",
            received_msg.id, received_msg.data[0], received_msg.data[1], received_msg.data[2], received_msg.data[3],
            received_msg.data[4], received_msg.data[5], received_msg.data[6], received_msg.data[7]);
    }

    // Limpiar la bandera de interrupción escribirnedo un 1 en el bit correspondiente
    CAN0->IFLAG1 = (1 << TX_MB);
}
