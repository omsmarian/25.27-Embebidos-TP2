/*
 * can.c
 *
 *  Created on: 1 oct 2024
 *      Author: asolari
 */


#include "can.h"



bool isCAN0 = false; // Set to true for CAN0

static CAN_Type *can_ptr = CAN0;



bool can_init(void)
{
	if (!isCAN0) {
		PRINTF("CAN0 inicializando...\n");
		// Step 1: Enable the clock to the FlexCAN module
		SIM->SCGC6 |= SIM_SCGC6_FLEXCAN0_MASK;

	    // Step 2: Set the clock source
		can_ptr->CTRL1 |= CAN_CTRL1_CLKSRC_MASK; // Set the clock source to the oscillator clock


		//Pin initialization
		// Enable the clock for the PORTs
		SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;

		// Configure the MUX settings for the CAN0 pins
		PORTB->PCR[18] = PORT_PCR_MUX(2); // CAN0_TX
		PORTB->PCR[19] = PORT_PCR_MUX(2); // CAN0_RX

	    // Step 3: Enable the CAN module
		//MDIS: Module Disable
		// 0 : Enable
		// 1 : Disable
		can_ptr->MCR &= ~CAN_MCR_MDIS_MASK; // Enable the FlexCAN module


	    // Step 4: Wait until FlexCAN module is out of low power mode (MCR[LPM_ACK] = 0).
	    // LPMACK: Low Power Mode Acknowledge
		// 0 : FlexCAN is not in a low power mode
		// 1 : FlexCAN is in a low power mode
		while (can_ptr->MCR & CAN_MCR_LPMACK_MASK);

	    // Step 5: Enter freeze mode
		//HALT: Halt FlexCAN
		// 0 : FlexCAN is not in Freeze mode
		// 1 : FlexCAN is in Freeze mode
		can_ptr->MCR |= CAN_MCR_FRZ_MASK; // Enter freeze mode

		// Wait until the FlexCAN module is in freeze mode
		//FRZACK: Freeze Mode Acknowledge
		// 0 : FlexCAN is not in Freeze mode
		// 1 : FlexCAN is in Freeze mode
		//Wait until the FlexCAN module is in freeze mode
		while (!(can_ptr->MCR & CAN_MCR_FRZACK_MASK));


	    // Step 6: Initialize other MCR bits as needed.
	    can_ptr->MCR |= CAN_MCR_IRMQ_MASK; // Enable individual filtering per MB and reception queue
	    can_ptr->MCR |= CAN_MCR_WRNEN_MASK; // Enable warning interrupts
	    can_ptr->MCR |= CAN_MCR_SRXDIS_MASK; // Disable self reception
	    can_ptr->MCR |= CAN_MCR_RFEN_MASK; // Enable RxFIFO
	    can_ptr->MCR |= CAN_MCR_AEN_MASK; // Enable abort mechanism
	    can_ptr->MCR |= CAN_MCR_LPRIOEN_MASK; // Enable local priority feature

	    // Set the number of message buffers
	    can_ptr->MCR = (can_ptr->MCR & ~CAN_MCR_MAXMB_MASK) | CAN_MCR_MAXMB(16); // Usar 16 MBs como ejemplo


	    // Step 7: Set the FlexCAN to 125 kbps
	    //Boundrate = Clock / (PRESDIV + 1) / (PROPSEG + PSEG1 + PSEG2 + 1)
	    can_ptr->CTRL1 |= CAN_CTRL1_PROPSEG(0x07) | CAN_CTRL1_PSEG2(0x10)
	    | CAN_CTRL1_PSEG1(0x10) | CAN_CTRL1_PRESDIV(0x13)
	    | CAN_CTRL1_RJW(0x03) | CAN_CTRL1_LPB_MASK; // Habilitar el modo loop-back

	    // Step 8: Initialize the message buffers

	    /* Inicializar Message Buffer 0  */
	    can_ptr->MB[TX_MB].CS = 0x00000000;   // Limpia el buffer 0
	    can_ptr->MB[TX_MB].ID = CAN_ID_STD(0x000);  // Configura el identificador (ID estándar 0x123)
	    can_ptr->MB[TX_MB].WORD0 = 0;          // Datos a transmitir, 1ª parte
	    can_ptr->MB[TX_MB].WORD1 = 0;          // Datos a transmitir, 2ª parte
	    can_ptr->MB[TX_MB].CS = CAN_CS_CODE(0b1000);  // Establece el código (0b1000: INACTIVE)

	    /* Inicializar Message Buffer 1 (ejemplo de recepción) */
	    can_ptr->MB[RX_MB].CS = 0x00000000;   // Limpia el buffer 1
	    can_ptr->MB[RX_MB].ID = CAN_ID_STD(0x000);  // Configura el identificador de filtro (ID estándar 0x456)
	    can_ptr->MB[RX_MB].CS = CAN_CS_CODE(0b0100);  // Establece el código 0b0100: EMPTY)

	    //Program CTRL2 register
	    can_ptr->CTRL2 = 0x00000000; // Clear the register
	    can_ptr->CTRL2 |= CAN_CTRL2_RRS_MASK; // Enable the RxFIFO
	    can_ptr->CTRL2 |= CAN_CTRL2_MRP_MASK; // Enable the most recent message priority feature
	    can_ptr->CTRL2 |= CAN_CTRL2_EACEN_MASK; // Enable the abort mechanism for individual MBs
	    can_ptr->CTRL2 |= CAN_CTRL2_TASD(0x03); // Set the time segment after sample point to 3
	    can_ptr->CTRL2 |= CAN_CTRL2_RFFN(0x03); // Set the number of Rx FIFO filters to 8


	    // Step 9: Initialize the ID filter table because Rx FIFO was enabled.
	    // 0 : Dont care
	    // 1 : Care
	    can_ptr->RXFGMASK = 0x00000000;  // Desactiva el filtrado en la FIFO
	    can_ptr->RXMGMASK = 0x00000000;  // Desactiva el filtrado en los buffers de mensajes

	    //Step 10: RXIMR: Initialize the Rx Individual Mask Registers
	    // 0 : Dont care
	    // 1 : Care
	    can_ptr->RXIMR[RX_MB] = 0; // Individual Mask for MB10
	    can_ptr->RXIMR[TX_MB] = 0; // Individual Mask for MB11

	    // Paso 11: Habilitar las interrupciones correspondientes
	    can_ptr->CTRL1 |= CAN_CTRL1_BOFFMSK_MASK | CAN_CTRL1_ERRMSK_MASK; // Habilitar interrupciones de error y bus off

	    // Habilitar la recuperación automática
	    can_ptr->CTRL1 |= CAN_CTRL1_BOFFREC_MASK;  // Permite la recuperación automática del Bus Off


	    // Habilitar interrupciones para los buffers de transmisión y recepción (por ejemplo, buffer 0 y 1)
	    can_ptr->IMASK1 |= (1 << RX_MB) | (1 << TX_MB); // Habilita las interrupciones para el buffer de recepción y transmisión



	    // Habilitar las interrupciones de mensaje para CAN0 en el NVIC
	    NVIC_EnableIRQ(CAN0_ORed_Message_buffer_IRQn);
	    // Verificar si la interrupción está habilitada
	    if (NVIC->ISER[CAN0_ORed_Message_buffer_IRQn / 32] & (1 << (CAN0_ORed_Message_buffer_IRQn % 32))) {
	        PRINTF("La interrupción CAN0 está habilitada\n");
	    } else {
	        PRINTF("La interrupción CAN0 no está habilitada\n");
	    }

	    // Paso 12 a 13: Salir del modo freeze y finalizar inicialización
	    can_ptr->MCR &= ~(CAN_MCR_HALT_MASK | CAN_MCR_FRZ_MASK);  // Deshabilitar HALT y FRZ juntos

	    while (can_ptr->MCR & CAN_MCR_FRZACK_MASK) {}  // Esperar a que FRZACK sea 0, indicando que se salió del freeze mode
	    PRINTF("Not in Freeze mode\n");

        while (can_ptr->MCR & CAN_MCR_NOTRDY_MASK) {}  // Esperar a que NOTRDY sea 0, indicando que el módulo está listo
        PRINTF("CAN Module Ready\n");


        // Verificar si hay errores en el CAN
		if (can_ptr->ESR1 & CAN_ESR1_ERRINT_MASK) {
			PRINTF("Error: CAN Error\n");
		}



	    isCAN0 = true;
	    return true;
	}
}


bool can_tx_msg(can_msg_t *msg) {
    // 1. Comprobar si hay una interrupción pendiente y limpiarla
    if (can_ptr->IFLAG1 & (1 << TX_MB)) {
        can_ptr->IFLAG1 |= (1 << TX_MB);  // Limpiar la interrupción si está activa
    }

    // 2. Configura el Message Buffer (MB) para transmitir:
    // Configura el campo de control (CS) del MB para indicar que se trata de un marco de datos.
    // Configura el campo de identificación (ID) del MB con el ID del mensaje CAN.
    // Configura el campo de longitud de datos (DLC) del MB con la longitud del mensaje CAN.
    // Copia los datos del mensaje CAN en el campo de datos del MB.
    can_ptr->MB[TX_MB].CS = CAN_CS_CODE(0b1100) | CAN_CS_DLC(msg->len);  // Activar MB para transmitir
    can_ptr->MB[TX_MB].ID = CAN_ID_STD(msg->id);
    can_ptr->MB[TX_MB].WORD0 = (msg->data[0] << 24) | (msg->data[1] << 16) | (msg->data[2] << 8) | msg->data[3];
    can_ptr->MB[TX_MB].WORD1 = (msg->data[4] << 24) | (msg->data[5] << 16) | (msg->data[6] << 8) | msg->data[7];

    // 3. Inicia la transmisión:
    // Escribe en el campo CODE del CS para iniciar la transmisión.
    can_ptr->MB[TX_MB].CS = CAN_CS_CODE(0b1100) | CAN_CS_DLC(msg->len);  // Activar MB para transmitir

    // 4. Espera hasta que el mensaje se envíe:
    // Consulta el campo CODE del CS hasta que indique que la transmisión se ha completado.
    int timeout = 10000000;  // Ajusta el tiempo de espera según sea necesario
    while (!(can_ptr->IFLAG1 & (1 << TX_MB))) {
        if (--timeout == 0) {
            PRINTF("Error: Timeout esperando a que IFLAG1 se active\n");
            return false;  // Salir si hay un timeout
        }
    }

    // 5. Limpia la bandera de interrupción del MB.
    can_ptr->IFLAG1 |= (1 << TX_MB);

    return true;  // Transmisión exitosa
}




bool can_rx_msg(can_msg_t *msg) {
    // Verifica si el CAN está en modo freeze o deshabilitado
    if (can_ptr->MCR & CAN_MCR_FRZACK_MASK) {
        PRINTF("CAN en freeze mode\n");
        return false;  // Error: CAN en freeze mode
    }


    // Lee el identificador CAN (ID estándar de 11 bits)
    msg->id = CAN_ID_STD(can_ptr->MB[RX_MB].ID);  // Asigna el ID del mensaje CAN

    // Lee los datos de WORD0 y WORD1
    msg->data[0] = (can_ptr->MB[RX_MB].WORD0 >> 24) & 0xFF;
    msg->data[1] = (can_ptr->MB[RX_MB].WORD0 >> 16) & 0xFF;
    msg->data[2] = (can_ptr->MB[RX_MB].WORD0 >> 8) & 0xFF;
    msg->data[3] = can_ptr->MB[RX_MB].WORD0 & 0xFF;
    msg->data[4] = (can_ptr->MB[RX_MB].WORD1 >> 24) & 0xFF;
    msg->data[5] = (can_ptr->MB[RX_MB].WORD1 >> 16) & 0xFF;
    msg->data[6] = (can_ptr->MB[RX_MB].WORD1 >> 8) & 0xFF;
    msg->data[7] = can_ptr->MB[RX_MB].WORD1 & 0xFF;

    // Lee la longitud de los datos (DLC)
    msg->len = (can_ptr->MB[RX_MB].CS & CAN_CS_DLC_MASK) >> CAN_CS_DLC_SHIFT;

    // Limpiar la bandera de interrupción del buffer de recepción
    can_ptr->IFLAG1 = (1 << RX_MB);

    // Retornar true si se recibió exitosamente
    return true;
}

