/***************************************************************************//**
  @file     cqueue.c
  @brief    Circular Queue / Ring Buffer / FIFO Implementation
  @author   Group 4: - Oms, Mariano
                     - Solari Raigoso, Agustín
                     - Wickham, Tomás
                     - Vieira, Valentin Ulises
  @note     Based on the work of Daniel Jacoby
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include "cqueue.h"

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

#define DEVELOPMENT_MODE	1

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

/**
 * @brief Queue structure
 * @param items Data, QUEUE_MAX_SIZE elements + 1 for overflow detection
 * @param front First element, equal to rear if empty
 * @param Rear Last element, points to next empty slot or QUEUE_OVERFLOW if full
 */
typedef struct {
	data_t items[QUEUE_OVERFLOW];
	count_t front;
	count_t rear;
} queue_t; // TODO: Use void* for data_t and count_t

/*******************************************************************************
 * STATIC VARIABLES AND CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

static queue_t queues[QUEUES_MAX_CANT];
static queue_id_t ids = 0;

/*******************************************************************************
 *******************************************************************************
						GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

queue_id_t queueInit (void)
{
	if (ids < QUEUES_MAX_CANT)
		queueClear(ids);
	else
		ids = QUEUES_MAX_CANT;

	return ids++;
}

count_t queuePush (queue_id_t id, data_t data) // Enqueue
{
	count_t count = queueSize(id);

	if (!queueIsFull(id))
	{
		queues[id].items[queues[id].rear] = data;
		queues[id].rear++;
		if (queues[id].rear >= QUEUE_OVERFLOW)
			queues[id].rear -= QUEUE_OVERFLOW;
	}

	return count + 1;
}

data_t queuePop (queue_id_t id) // Dequeue
{
	data_t data = queues[id].items[queues[id].front];

	if (!queueIsEmpty(id))
	{
		queues[id].front++;
		if (queues[id].front >= QUEUE_OVERFLOW)
			queues[id].front -= QUEUE_OVERFLOW;
	}
	else
		data = QUEUE_UNDERFLOW;

	return data;
}

count_t	queueSize (queue_id_t id)
{
	count_t size = (QUEUE_OVERFLOW + queues[id].rear) - queues[id].front;
	if (size >= QUEUE_OVERFLOW)
		size -= QUEUE_OVERFLOW;

	return size;
}

data_t	queueFront		(queue_id_t id) { return !queueIsEmpty(id) ? queues[id].items[queues[id].front] : QUEUE_UNDERFLOW; }
data_t	queueBack		(queue_id_t id) { return !queueIsEmpty(id) ? queues[id].items[queues[id].rear - 1] : QUEUE_UNDERFLOW; }
bool	queueIsEmpty	(queue_id_t id) { return !queueSize(id); }
bool	queueIsFull		(queue_id_t id) { return queueSize(id) == QUEUE_MAX_SIZE; }
void	queueClear		(queue_id_t id) { queues[id].front = queues[id].rear = 0; }

/******************************************************************************/
