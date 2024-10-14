///*
// * can.c
// *
// *  Created on: 1 oct 2024
// *      Author: asolari
// */
//
//


#include <can.h>

// Definir cb en can.c
CircularBuffer_t cb;


CAN_Status CAN_Init( void )
{

	/// Enable clock for CAN module
	SIM->SCGC6 |= SIM_SCGC6_FLEXCAN0_MASK;

	/// Enable module
	CAN_Enable(CAN0);

	/// Soft reset
	CAN0->MCR |= CAN_MCR_SOFTRST(1);
	while((CAN0->MCR&CAN_MCR_SOFTRST_MASK)>> CAN_MCR_SOFTRST_SHIFT);

	/// Configure the TX and RC pins (B18 B19)
	PORT_Type * ports[] = PORT_BASE_PTRS;
	uint32_t PCR = PORT_PCR_MUX(2) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK; // REVISAR COMO CONFIGURAR BIEN EL PCR
	ports[PIN_CAN0_TX/32]->PCR[PIN_CAN0_TX%32] = PCR;
	ports[PIN_CAN0_RX/32]->PCR[PIN_CAN0_RX%32] = PCR;


	/// Disable CAN module in order to modify registers
	CAN_Disable(CAN0);

	/// Set the clock source for the CAN Protocol Engine.
	CAN0->CTRL1 = CAN_CTRL1_CLKSRC(0);

	/// Enable CAN module.
	CAN_Enable(CAN0);

	/// Module will enter freeze mode automatically.
	/// Wait until module goes into freeze mode (MCR[FRZ_ACK] = 1).
	while((CAN0->MCR & CAN_MCR_FRZACK_MASK)!=CAN_MCR_FRZACK_MASK);


	// Set the baud rate 125kbps
	CAN0->CTRL1 |= CAN_CTRL1_PRESDIV(0x13);
	CAN0->CTRL1 |= CAN_CTRL1_PROPSEG(0x07);
	CAN0->CTRL1 |= CAN_CTRL1_PSEG1(0x07);
	CAN0->CTRL1 |= CAN_CTRL1_PSEG2(0x02);

	CAN0->MCR |= CAN_MCR_SRXDIS_MASK;
	CAN0->MCR |= CAN_MCR_IRMQ_MASK;

	// Reset all message buffer
	for(int i=0; i<CAN_CS_COUNT; i++)
	{
		CAN0->MB[i].CS = ( CAN0->MB[i].CS &= ~CAN_CS_CODE_MASK ) | CAN_CS_CODE(RX_INACTIVE);
		CAN0->MB[i].ID = CAN_ID_STD(0);
		CAN0->RXIMR[i] = 0xFFFFFFFF;
	}

	// Exit Freeze mode
	CAN_Freeze(false);

	//Init circular buffer
	initBuffer(&cb);

	return CAN_SUCCESS;
}



void CAN_Enable()
{
	CAN0->MCR &= ~CAN_MCR_MDIS_MASK;
	// Wait until CAN module is out of low power mode (MCR[LPM_ACK] = 0).
	while(CAN0->MCR & CAN_MCR_LPMACK_MASK);
}



void CAN_Disable()
{
	CAN0->MCR |= CAN_MCR_MDIS_MASK;
	// Wait until module is in low power mode (MCR[LPM_ACK] = 1).
	while((CAN0->MCR & CAN_MCR_LPMACK_MASK)!=CAN_MCR_LPMACK_MASK);
}


void  CAN_ConfigureRxMB( uint8_t index, uint32_t ID)
{
	/// If the Mailbox is active (either Tx or Rx) inactivate the Mailbox.
	CAN0->MB[index].CS = ( CAN0->MB[index].CS &= ~CAN_CS_CODE_MASK ) | CAN_CS_CODE(RX_INACTIVE);

	/// Write the ID word.
	CAN0->MB[index].ID = CAN_ID_STD(ID);

	/// Write the EMPTY code to the CODE field of the Control and Status word to activate the Mailbox.
	CAN0->MB[index].CS = CAN_CS_CODE(RX_EMPTY) | CAN_CS_IDE(0);
}


void CAN_EnableMbInterrupts	( uint8_t index)
{
	NVIC_EnableIRQ(CAN0_ORed_Message_buffer_IRQn);
	// Enable the interrupt for the corresponding MB.
	CAN0->IMASK1 |= (1UL<<index);
}

void CAN_DisableMbInterrupts	( uint8_t index)
{
	CAN0->IMASK1 &= ~(1UL<<index);
}



void CAN_SetRxMbGlobalMask	( uint32_t 	mask)
{
	CAN_Freeze(true);
	CAN0->RXMGMASK = mask;
	CAN_Freeze(false);
}

void CAN_SetRxIndividualMask (uint8_t index, uint32_t 	mask)
{
	CAN_Freeze(true);
	CAN0->RXIMR[index] = mask;
	CAN_Freeze(false);
}

bool CAN_GetMbStatusFlag(uint8_t index)
{
	return (((CAN0->IFLAG1>>index)&1UL)==1UL);
}

void CAN_ClearMbStatusFlag(uint8_t index)
{
	CAN0->IFLAG1 |= (1<<index); // W1C
}

CAN_Status CAN_ReadRxMB(uint8_t index, CAN_DataFrame * frame)
{
	/// Check if the BUSY bit is deasserted.
	if(CAN0->MB[index].CS>>CAN_CS_CODE_SHIFT & 1UL)
		return CAN_RX_BUSY;

	uint32_t code = (CAN0->MB[index].CS & CAN_CS_CODE_MASK)>>CAN_CS_CODE_SHIFT;
	CAN_Status retVal;

	/// Check the CODE field of the Control and Status word.
	switch(code)
	{
	case RX_EMPTY:
		retVal = CAN_FAILED;
		break;

	case RX_FULL:
		/// Read the contents of the Mailbox.
		frame->ID = (CAN0->MB[index].ID & CAN_ID_STD_MASK)>> CAN_ID_STD_SHIFT;
		frame->length = (CAN0->MB[index].CS & CAN_CS_DLC_MASK) >> CAN_CS_DLC_SHIFT;

		frame->dataWord0 =  ((CAN0->MB[index].WORD0 & CAN_WORD0_DATA_BYTE_0_MASK)>>24)|
							((CAN0->MB[index].WORD0 & CAN_WORD0_DATA_BYTE_1_MASK)>>8)|
							((CAN0->MB[index].WORD0 & CAN_WORD0_DATA_BYTE_2_MASK)<<8)|
							((CAN0->MB[index].WORD0 & CAN_WORD0_DATA_BYTE_3_MASK)<<24);

		frame->dataWord1 =  ((CAN0->MB[index].WORD1 & CAN_WORD1_DATA_BYTE_4_MASK)>>24)|
							((CAN0->MB[index].WORD1 & CAN_WORD1_DATA_BYTE_5_MASK)>>8)|
							((CAN0->MB[index].WORD1 & CAN_WORD1_DATA_BYTE_6_MASK)<<8)|
							((CAN0->MB[index].WORD1 & CAN_WORD1_DATA_BYTE_7_MASK)<<24);
		/// Clean the flag
		CAN0->IFLAG1 |= (1<<index);

		/// Read the Free Running Timer to unlock Mailbox as soon as possible and make it available for reception.
		CAN0->TIMER;

		retVal = CAN_SUCCESS;
		break;

	case RX_OVERRUN:
		retVal =  CAN_RX_OVERFLOW;
		break;
	}
	return retVal;
}



CAN_Status  CAN_WriteTxMB(uint8_t index, CAN_DataFrame * frame)
{
	CAN_Status retVal;
	if(index < CAN_CS_COUNT) // CHEQUEAR QUE NO PERTENEZCA A LA FIFO SI ESTA ACTIVA
	{
		/// Check whether the respective interrupt bit is set and clear it.
		if(CAN0->IFLAG1& (1<<index))
			CAN0->IFLAG1 |= (1<<index); // W1C

		/// Write INACTIVE code to the CODE field (a pending frame may be transmitted without notification).
		CAN0->MB[index].CS = CAN_CS_CODE(TX_INACTIVE);

		/// Write the ID word.
		CAN0->MB[index].ID = CAN_ID_STD(frame->ID);

		//Cleanr the data bytes
		CAN0->MB[index].WORD0 = 0;
		CAN0->MB[index].WORD1 = 0;

		/// Write the data bytes in order using macros.
		CAN0->MB[index].WORD0 = CAN_WORD0_DATA_BYTE_0(frame->dataByte0) |
		                    	CAN_WORD0_DATA_BYTE_1(frame->dataByte1) |
		                    	CAN_WORD0_DATA_BYTE_2(frame->dataByte2) |
		                    	CAN_WORD0_DATA_BYTE_3(frame->dataByte3);
		CAN0->MB[index].WORD1 = CAN_WORD1_DATA_BYTE_4(frame->dataByte4) |
		                        CAN_WORD1_DATA_BYTE_5(frame->dataByte5) |
		                    	CAN_WORD1_DATA_BYTE_6(frame->dataByte6) |
		                    	CAN_WORD1_DATA_BYTE_7(frame->dataByte7);

		/// Write the DLC and CODE fields of the Control and Status word to activate the MB.
		CAN0->MB[index].CS = CAN_CS_CODE(TX_DATA) | CAN_CS_DLC(frame->length) | CAN_CS_SRR(1) | CAN_CS_IDE(0);


		retVal = CAN_SUCCESS;
	}
	else
		retVal = CAN_ERROR;

	return retVal;
}


void CAN0_ORed_Message_buffer_IRQHandler(void)
{
	CAN_DataFrame frame;
	for(int i=0; i<CAN_CS_COUNT; i++)
	{
		if( CAN0->IFLAG1 & (1<<i) )
		{
			if(((CAN0->MB[i].CS&CAN_CS_CODE_MASK)>>CAN_CS_CODE_SHIFT)==RX_FULL)
			{
				CAN_Status s = CAN_ReadRxMB(i,&frame);

                //If status is success, add the frame to the buffer
                if (s == CAN_SUCCESS)
                {
                    addToBuffer(&cb, frame);
                }
			}
		}
	}
}




void CAN_Freeze(bool freeze)
{
	if(freeze)
	{
		CAN0->MCR |= CAN_MCR_HALT_MASK;
		// Wait until module goes into freeze mode (MCR[FRZ_ACK] = 1).
		while((CAN0->MCR & CAN_MCR_FRZACK_MASK)!=CAN_MCR_FRZACK_MASK);
	}
	else
	{
		CAN0->MCR &= ~CAN_MCR_HALT_MASK;
		// Wait until module goes out freeze mode (MCR[FRZ_ACK] = 0).
		while((CAN0->MCR & CAN_MCR_FRZACK_MASK)==CAN_MCR_FRZACK_MASK);
	}
}

// Inicializa el buffer circular
void initBuffer(CircularBuffer_t *cb) {
    cb->start = 0;
    cb->end = 0;
}

void addToBuffer(CircularBuffer_t *cb, CAN_DataFrame frame) {
    // Verificar si el buffer está lleno
    if ((cb->end + 1) % BUFFER_SIZE == cb->start) {
        // Incrementar el índice de inicio para descartar el frame más antiguo
        cb->start = (cb->start + 1) % BUFFER_SIZE;
    }

    // Agregar el nuevo frame en la posición actual del índice de fin
    cb->buffer[cb->end] = frame;

    // Incrementar el índice de fin
    cb->end = (cb->end + 1) % BUFFER_SIZE;
}

// Obtiene un frame del buffer circular
CAN_DataFrame getFromBuffer(CircularBuffer_t *cb) {
    if (cb->start == cb->end) {
        // El buffer está vacío
        return (CAN_DataFrame){0};
    } else {
        CAN_DataFrame frame = cb->buffer[cb->start];
        cb->start = (cb->start + 1) % BUFFER_SIZE;
        return frame;
    }
}

