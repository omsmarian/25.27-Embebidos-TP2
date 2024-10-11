/*
 * can.c
 *
 *  Created on: 1 oct 2024
 *      Author: asolari
 */


#include "can.h"

// Definir cb en can.c
CircularBuffer cb;


bool isCAN0 = false; // Set to true for CAN0


void initBuffer(CircularBuffer *cb);
void addToBuffer(CircularBuffer *cb, CAN_DataFrame frame);
CAN_DataFrame getFromBuffer(CircularBuffer *cb);



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

static void CAN_Freeze( bool freeze)
{
	if(freeze)
	{
		CAN0->MCR |= CAN_MCR_HALT_MASK;
		//Wait until enter to freeze mode
		while((CAN0->MCR & CAN_MCR_FRZACK_MASK)!=CAN_MCR_FRZACK_MASK);
	}
	else
	{
		CAN0->MCR &= ~(CAN_MCR_HALT_MASK | CAN_MCR_FRZ_MASK);  // Deshabilitar HALT y FRZ juntos
	    // Wait until exit from freeze mode
		while (CAN0->MCR & CAN_MCR_FRZACK_MASK) {}
	}
}


bool CAN_Init(void)
{
	if (!isCAN0)
	{
		PRINTF("Initializing CAN...\n");

		//Enable the clock to the FlexCAN module
		SIM->SCGC6 |= SIM_SCGC6_FLEXCAN0_MASK;

	    //Set the clock source
		CAN0->CTRL1 |= CAN_CTRL1_CLKSRC_MASK; // Set the clock source to the oscillator clock



		CAN_Enable();

		//Pin initialization
		// Enable the clock for the PORTs
		SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;

		/// Configure the TX and RC pins (PB18 PB19)
		PORT_Type * ports[] = PORT_BASE_PTRS;
		uint32_t PCR = PORT_PCR_MUX(2) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK; // REVISAR COMO CONFIGURAR BIEN EL PCR
		ports[PIN_CAN0_TX/32]->PCR[PIN_CAN0_TX%32] = PCR;
		ports[PIN_CAN0_RX/32]->PCR[PIN_CAN0_RX%32] = PCR;




		//Enter Freeze mode
	    CAN_Freeze(true);


	    // Set the number of message buffers
	    CAN0->MCR = (CAN0->MCR & ~CAN_MCR_MAXMB_MASK) | CAN_MCR_MAXMB(15); // Usar 16 MBs como ejemplo



	    // Habilitar la recuperación automática
	    CAN0->CTRL1 |= CAN_CTRL1_BOFFREC_MASK;  // Permite la recuperación automática del Bus Off


	    //  Set the FlexCAN to 125 kbps
	    //Boundrate = Clock / (PRESDIV + 1) / (PROPSEG + PSEG1 + PSEG2 + 1)
	    CAN0->CTRL1 |= CAN_CTRL1_PROPSEG(6) | CAN_CTRL1_PRESDIV(9)
	    | CAN_CTRL1_PSEG1(1) | CAN_CTRL1_PSEG2(1) | CAN_CTRL1_RJW(0)
	    | CAN_CTRL1_LBUF(0) | CAN_CTRL1_LPB(0); //parametros para control de tiempo


	    CAN0->CTRL2 |= CAN_CTRL2_MRP_MASK;

	    // Initialize the message buffers with a default configuration
		for(int i=0; i<CAN_CS_COUNT; i++)
		{
			CAN0->MB[i].CS = ( CAN0->MB[i].CS &= ~CAN_CS_CODE_MASK ) | CAN_CS_CODE(RX_INACTIVE);
			CAN0->MB[i].ID = CAN_ID_STD(0);
			CAN0->RXIMR[i] = 0xFFFFFFFF;
		}


		// Exit Freeze mode after configuration
		CAN_Freeze(false);

		//Init circular buffer
		initBuffer(&cb);

	    isCAN0 = true;
	    return true;
	}
}

void  CAN_ConfigureRxMB( uint8_t index, uint16_t ID)
{
    CAN0->MB[index].ID = 0; //Enter to buffer and clean the ID
    CAN0->MB[index].ID |= CAN_ID_STD(ID); //Set the ID
    CAN0->MB[index].CS &= CAN_CS_CODE_MASK ; //Reset the code field
    CAN0->MB[index].CS |= CAN_CS_CODE(RX_EMPTY) | CAN_CS_IDE(0) | CAN_CS_RTR(0); //empty_rx, buffer vacio y ok

    //Clean the data
    CAN0->MB[index].WORD0 = 0;
    CAN0->MB[index].WORD1 = 0;

    PRINTF("Configured MB RX %d\n",index);
}

void CAN_ConfigureTxMB(uint8_t index) {
    CAN0->MB[index].CS &= CAN_CS_CODE_MASK ; //Reset the code
    CAN0->MB[index].CS |= ( CAN0->MB[index].CS &= ~CAN_CS_CODE_MASK ) | CAN_CS_CODE(TX_INACTIVE); // Set the code to INACTIVE
    CAN0->MB[index].ID = CAN_ID_STD(0); // Clean the ID

    PRINTF("Configured MB TX %d\n",index);
}



void CAN_EnableMbInterrupts	( uint8_t index)
{
	NVIC_EnableIRQ(CAN0_ORed_Message_buffer_IRQn); // Enable the NVIC interrupt
	CAN0->IMASK1 |= (1UL<<index); // Enable the interrupt with the respective index

}

void CAN_DisableMbInterrupts	( uint8_t index)
{
	CAN0->IMASK1 &= ~(1UL<<index); //Clean the respective bit for interrupt
}


void CAN_ClearMbStatusFlag(uint8_t index)
{
	CAN0->IFLAG1 |= (1<<index); //Clean the respective flag
}


CAN_Status CAN_ReadRxMB(uint8_t index, CAN_DataFrame * frame)
{

	/// Check if the BUSY bit is deasserted.
	if(CAN0->MB[index].CS>>CAN_CS_CODE_SHIFT & 1UL)
		return CAN_RX_BUSY;

	uint32_t code = (CAN0->MB[index].CS & CAN_CS_CODE_MASK)>>CAN_CS_CODE_SHIFT;
	CAN_Status retVal;


	switch(code)
	{
	case RX_EMPTY:
		retVal = CAN_FAILED;
		break;

	case RX_FULL:
		///Read the contents of the buffer
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
		CAN0->IFLAG1 |= (1<<index); // W1C

		/// Read the Free Running Timer to unlock Mailbox as soon as possible and make it available for reception.
		CAN0->TIMER;

		//Set the mailbox as empty
		CAN0->MB[index].CS = CAN_CS_CODE(RX_EMPTY);

		retVal = CAN_SUCCESS;
		break;

	case RX_OVERRUN:
		retVal =  CAN_RX_OVERFLOW;
		break;
	}


	return retVal;
}

//CAN_Status  CAN_WriteTxMB(uint8_t index, CAN_DataFrame * frame)
//{
//    CAN_Status retVal;
//    if(index < CAN_CS_COUNT)
//    {
//        /// Check whether the respective interrupt bit is set and clear it.
//        if(CAN0->IFLAG1& (1<<index))
//            CAN0->IFLAG1 |= (1<<index); //Clean the flag
//
//        /// Write INACTIVE code to the CODE field (a pending frame may be transmitted without notification).
//        CAN0->MB[index].CS = CAN_CS_CODE(TX_INACTIVE);
//
//        /// Write the ID word.
//        CAN0->MB[index].ID = CAN_ID_STD(frame->ID);
//
//        /// Reset the data bytes.
//        CAN0->MB[index].WORD0 = 0;
//        CAN0->MB[index].WORD1 = 0;
//
//        /// Write the data bytes in order using macros.
//        for (int i = 0; i < frame->length; i++) {
//            switch (i) {
//                case 0: CAN0->MB[index].WORD0 |= CAN_WORD0_DATA_BYTE_0(frame->data[i]); break;
//                case 1: CAN0->MB[index].WORD0 |= CAN_WORD0_DATA_BYTE_1(frame->data[i]); break;
//                case 2: CAN0->MB[index].WORD0 |= CAN_WORD0_DATA_BYTE_2(frame->data[i]); break;
//                case 3: CAN0->MB[index].WORD0 |= CAN_WORD0_DATA_BYTE_3(frame->data[i]); break;
//                case 4: CAN0->MB[index].WORD1 |= CAN_WORD1_DATA_BYTE_4(frame->data[i]); break;
//                case 5: CAN0->MB[index].WORD1 |= CAN_WORD1_DATA_BYTE_5(frame->data[i]); break;
//                case 6: CAN0->MB[index].WORD1 |= CAN_WORD1_DATA_BYTE_6(frame->data[i]); break;
//                case 7: CAN0->MB[index].WORD1 |= CAN_WORD1_DATA_BYTE_7(frame->data[i]); break;
//            }
//        }
//
//        /// Write the DLC and CODE fields of the Control and Status word to activate the MB.
//        CAN0->MB[index].CS = CAN_CS_CODE(TX_DATA) | CAN_CS_DLC(frame->length) | CAN_CS_SRR(1) | CAN_CS_IDE(0);
//
//        retVal = CAN_SUCCESS;
//    }
//    else
//        retVal = CAN_ERROR;
//
//    return retVal;
//}

CAN_Status  CAN_WriteTxMB(uint8_t index, CAN_DataFrame * frame)
{

	CAN_Status retVal;
	if(index < CAN_CS_COUNT)
	{
		/// Check whether the respective interrupt bit is set and clear it.
		if(CAN0->IFLAG1& (1<<index))
			CAN0->IFLAG1 |= (1<<index); //Clean the flag

		/// Write INACTIVE code to the CODE field (a pending frame may be transmitted without notification).
		CAN0->MB[index].CS = CAN_CS_CODE(TX_INACTIVE);

		/// Write the ID word.
		CAN0->MB[index].ID = CAN_ID_STD(frame->ID);

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
    PRINTF("ISR\n");
    CAN_DataFrame frame;
    for(int i=0; i<CAN_CS_COUNT; i++)
    {
        if( CAN0->IFLAG1 & (1<<i) )
        {
            //Clear the flag
            CAN_ClearMbStatusFlag(i);

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

// Inicializa el buffer circular
void initBuffer(CircularBuffer *cb) {
    cb->start = 0;
    cb->end = 0;
}

// Añade un frame al buffer circular
void addToBuffer(CircularBuffer *cb, CAN_DataFrame frame) {
    cb->buffer[cb->end] = frame;
    cb->end = (cb->end + 1) % BUFFER_SIZE;
    if (cb->end == cb->start) {
        // El buffer está lleno, por lo que avanzamos el inicio para hacer espacio
        cb->start = (cb->start + 1) % BUFFER_SIZE;
    }
}

// Obtiene un frame del buffer circular
CAN_DataFrame getFromBuffer(CircularBuffer *cb) {
    if (cb->start == cb->end) {
        // El buffer está vacío
        return (CAN_DataFrame){0};
    } else {
        CAN_DataFrame frame = cb->buffer[cb->start];
        cb->start = (cb->start + 1) % BUFFER_SIZE;
        return frame;
    }
}
