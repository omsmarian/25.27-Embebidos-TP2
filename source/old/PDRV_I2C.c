/***************************************************************************//**
  @file     PDRV_I2C.c
  @brief    +Descripcion del archivo+
  @author   matia
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include "MK64F12.h"
#include "PDRV_I2C.h"

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

#define SIM_SCGC5_PORT_MASK 0x3E00
#define SIM_SCGC5_PORT(port,b) (((uint32_t)(((uint32_t)(b)) << (9+port))) & SIM_SCGC5_PORT_MASK)

#define I2C0_PORT	4
#define I2C1_PORT	2
#define I2C2_PORT	0

#define I2C0_SDA_PIN	25	// PTE25
#define I2C0_SCL_PIN	24	// PTE24
#define I2C1_SDA_PIN	11	// PTC11
#define I2C1_SCL_PIN	10	// PTC10
#define I2C2_SDA_PIN	13	// PTA13	No está disponible en la FRDM
#define I2C2_SCL_PIN	14	// PTA14	No está disponible en la FRDM

#define I2C0_ALT	5
#define I2C1_ALT	2
#define I2C2_ALT	5

#define I2C_PORT(num)		((num == 0)?	I2C0_PORT: \
							((num == 1)?	I2C1_PORT: \
											I2C2_PORT))

#define I2C_ALT(num)		((num == 0)?	I2C0_ALT: \
							((num == 1)?	I2C1_ALT: \
											I2C2_ALT))

#define I2C_SDA_PIN(num)	((num == 0)?	I2C0_SDA_PIN: \
							((num == 1)?	I2C1_SDA_PIN: \
											I2C2_SDA_PIN))

#define I2C_SCL_PIN(num)	((num == 0)?	I2C0_SCL_PIN: \
							((num == 1)?	I2C1_SCL_PIN: \
											I2C2_SCL_PIN))

#define F_MUL_VALUE	0x2		// mul = 4

/* These correspond to bit 0 of the address byte. */
#define I2C_WRITING 0
#define I2C_READING 1

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

typedef struct {
  uint16_t *sequence;
  uint16_t *sequence_end;
  uint8_t *received_data;
  void (*callback_fn)(void*);
  void *user_data;
  uint8_t reads_ahead;
  uint8_t status;
  uint8_t txrx;
} I2C_Channel;

/*******************************************************************************
 * VARIABLES WITH GLOBAL SCOPE
 ******************************************************************************/


/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * ROM CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

static PORT_Type* const PORT_PTRS[FSL_FEATURE_SOC_PORT_COUNT] = PORT_BASE_PTRS;
static I2C_Type* const I2C_PTRS[] = I2C_BASE_PTRS;
static IRQn_Type const I2C_IRQn[] = I2C_IRQS;

/*******************************************************************************
 * STATIC VARIABLES AND CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

static I2C_Channel i2c_channels[I2C_NUMBER_OF_DEVICES];


/*******************************************************************************
 *******************************************************************************
                        GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

void initI2C(uint8_t i2c_num, i2c_speed_t i2c_speed) {
	I2C_Type* i2c = I2C_PTRS[i2c_num];

	// 1) Clock enable
	SIM->SCGC5 |= SIM_SCGC5_PORT(I2C_PORT(i2c_num),1);

	switch(i2c_num){
	case 0: SIM->SCGC4 |= SIM_SCGC4_I2C0(1); break;
	case 1: SIM->SCGC4 |= SIM_SCGC4_I2C1(1); break;
	case 2: SIM->SCGC1 |= SIM_SCGC1_I2C2(1); break;
	}

	// 2) Port configuration
	PORT_PTRS[I2C_PORT(i2c_num)]->PCR[I2C_SDA_PIN(i2c_num)] = PORT_PCR_MUX(I2C_ALT(i2c_num)) | PORT_PCR_ODE(1); // Enable open drain
	PORT_PTRS[I2C_PORT(i2c_num)]->PCR[I2C_SCL_PIN(i2c_num)] = PORT_PCR_MUX(I2C_ALT(i2c_num)) | PORT_PCR_ODE(1); // Enable open drain

	i2c->C1 = 0;
	i2c->C1 |= I2C_C1_IICEN_MASK;
	i2c->F &= ~0xf;
	i2c->F |= ((F_MUL_VALUE << I2C_F_MULT_SHIFT) | i2c_speed);
	NVIC_EnableIRQ(I2C_IRQn[i2c_num]);
}

int32_t i2cSendSequence(uint8_t i2c_num, uint16_t* sequence, uint32_t sequence_len, uint8_t *received_data,
						  void (*callback_fn)(void*), void *user_data) {
	volatile I2C_Channel *channel = &(i2c_channels[i2c_num]);
	I2C_Type* i2c = I2C_PTRS[i2c_num];
	int32_t result = 0;
	uint8_t status;

	if(channel->status == I2C_BUSY) {
		return -1;
	}

	channel->sequence = sequence;
	channel->sequence_end = sequence + sequence_len;
	channel->received_data = received_data;
	channel->status = I2C_BUSY;
	channel->txrx = I2C_WRITING;
	channel->callback_fn = callback_fn;
	channel->user_data = user_data;

	/* reads_ahead does not need to be initialized */

	i2c->S |= I2C_S_IICIF_MASK; /* Acknowledge the interrupt request, just in case */
	i2c->C1 = (I2C_C1_IICEN_MASK | I2C_C1_IICIE_MASK);

	/* Generate a start condition and prepare for transmitting. */
	i2c->C1 |= (I2C_C1_MST_MASK | I2C_C1_TX_MASK);

	status = i2c->S;
	if(status & I2C_S_ARBL_MASK) {
		result = -1;
		i2c->C1 &= ~(I2C_C1_IICIE_MASK | I2C_C1_MST_MASK | I2C_C1_TX_MASK);
		channel->status = I2C_ERROR;
		return result;
	}

	/* Write the first (address) byte. */
	i2c->D = *channel->sequence++;

	return result;                /* Everything is OK. */
}

uint8_t i2cGetStatus(uint8_t i2c_num){
	return i2c_channels[i2c_num].status;
}


/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/


void i2c_irq_handler(uint32_t channel_number) {
	volatile I2C_Channel* channel = &i2c_channels[channel_number];
	I2C_Type* i2c = (I2C_Type*)I2C_PTRS[channel_number];
	uint16_t element;
	uint8_t status;

	status = i2c->S;

	/* Was the interrupt request from the current I2C module? */
	if(!(status & I2C_S_IICIF_MASK)) {
		return;						/* This should never happen, but... */
	}

	i2c->S |= I2C_S_IICIF_MASK;	/* Acknowledge the interrupt request. */

	if(status & I2C_S_ARBL_MASK) {
		i2c->S |= I2C_S_ARBL_MASK;
		i2c->C1 &= ~(I2C_C1_MST_MASK | I2C_C1_IICIE_MASK); /* Generate STOP and disable further interrupts. */
		channel->status = I2C_ERROR;
		return;
	}

	if(channel->txrx == I2C_READING) {

		switch(channel->reads_ahead) {
		case 0:
			/* All the reads in the sequence have been processed (but note that the final data register read still needs to
			 be done below! Now, the next thing is either a restart or the end of a sequence. In any case, we need to
			 switch to TX mode, either to generate a repeated start condition, or to avoid triggering another I2C read
			 when reading the contents of the data register. */
			i2c->C1 |= I2C_C1_TX_MASK;

			/* Perform the final data register read now that it's safe to do so. */
			*channel->received_data++ = i2c->D;

			/* Do we have a repeated start? */
			if((channel->sequence < channel->sequence_end) && (*channel->sequence == I2C_RESTART)) {

				i2c->C1 |= I2C_C1_RSTA_MASK; /* Generate a repeated start condition. */

				/* A restart is processed immediately, so we need to get a new element from our sequence. This is safe, because
				   a sequence cannot end with a RESTART: there has to be something after it. Note that the only thing that can
				   come after a restart is an address write. */
				channel->txrx = I2C_WRITING;
				channel->sequence++;
				element = *channel->sequence;
				i2c->D = element;
			} else {

				/* Generate STOP (set MST=0), switch to RX mode, and disable further interrupts. */
				i2c->C1 &= ~(I2C_C1_MST_MASK | I2C_C1_IICIE_MASK | I2C_C1_TXAK_MASK);

				/* Call the user-supplied callback function upon successful completion (if it exists). */
				if(channel->callback_fn) {
					(*channel->callback_fn)(channel->user_data);
				}
				channel->status = I2C_AVAILABLE;
				return;

			}
			break;

		case 1:
			i2c->C1 |= I2C_C1_TXAK_MASK; /* do not ACK the final read */
			*channel->received_data++ = i2c->D;
			break;

		default:
			*channel->received_data++ = i2c->D;
			break;
		}

		channel->reads_ahead--;

	} else { /* channel->txrx == I2C_WRITING */
        /* First, check if we are at the end of a sequence. */
        if(channel->sequence == channel->sequence_end) {
            /* Generate STOP (set MST=0), switch to RX mode, and disable further interrupts. */
            i2c->C1 &= ~(I2C_C1_MST_MASK | I2C_C1_IICIE_MASK | I2C_C1_TXAK_MASK);

            /* Call the user-supplied callback function upon successful completion (if it exists). */
            if(channel->callback_fn) {
            (*channel->callback_fn)(channel->user_data);
            }
            channel->status = I2C_AVAILABLE;
            return;
        }

        if(status & I2C_S_RXAK_MASK) {
            /* We received a NACK. Generate a STOP condition and abort. */
            i2c->C1 &= ~(I2C_C1_MST_MASK | I2C_C1_IICIE_MASK); /* Generate STOP and disable further interrupts. */
            channel->status = I2C_ERROR;
            return;
        }

        /* check next thing in our sequence */
        element = *channel->sequence;

        if(element == I2C_RESTART) {
            /* Do we have a restart? If so, generate repeated start and make sure TX is on. */

            i2c->C1 |= (I2C_C1_RSTA_MASK | I2C_C1_TX_MASK); /* Generate a repeated start condition and switch to TX */

            /* A restart is processed immediately, so we need to get a new element from our sequence. This is safe, because a
            sequence cannot end with a RESTART: there has to be something after it. */
            channel->sequence++;
            element = *channel->sequence;
            /* Note that the only thing that can come after a restart is a write. */

            i2c->D = element;

        } else {
            if(element == I2C_READ) {
                channel->txrx = I2C_READING;
                /* How many reads do we have ahead of us (not including this one)? For reads we need to know the segment length
                to correctly plan NACK transmissions. */
                channel->reads_ahead = 1;        /* We already know about one read */
                while(((channel->sequence + channel->reads_ahead) < channel->sequence_end) &&
                    (*(channel->sequence + channel->reads_ahead) == I2C_READ)) {
                    channel->reads_ahead++;
                }
                i2c->C1 &= ~I2C_C1_TX_MASK; /* Switch to RX mode. */

                if(channel->reads_ahead == 1) {
                    i2c->C1 |= I2C_C1_TXAK_MASK; /* do not ACK the final read */
                } else {
                    i2c->C1 &= ~(I2C_C1_TXAK_MASK);  /* ACK all but the final read */
                }
                /* Dummy read comes first, note that this is not valid data! This only triggers a read, actual data will come
                in the next interrupt call and overwrite this. This is why we do not increment the received_data
                pointer. */
                *channel->received_data = i2c->D;
                channel->reads_ahead--;
            } else {
                /* Not a restart, not a read, must be a write. */
                i2c->D = element;
            }
        }
    }

	channel->sequence++;
	return;

}

//void I2C0_IRQHandler(void) {i2c_irq_handler(0);}
//void I2C1_IRQHandler(void) {i2c_irq_handler(1);}
//void I2C2_IRQHandler(void) {i2c_irq_handler(2);}
