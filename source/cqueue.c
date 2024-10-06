/***************************************************************************//**
  @file     cqueue.c
  @brief    Circular Queue / Ring Buffer / FIFO Implementation
  @author   Group 4
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include "cqueue.h"


/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

/*
 * Queue structure
 * items: data, QUEUE_MAX_SIZE elements + 1 for overflow detection
 * front: first element, equal to rear if empty
 * rear: last element, points to next empty slot or QUEUE_OVERFLOW if full
 */
typedef struct {
	data_t items[QUEUE_OVERFLOW];
	count_t front;
	count_t rear;
} queue_t;


/*******************************************************************************
 * STATIC VARIABLES AND CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

static queue_t queues[QUEUES_MAX_CANT];


/*******************************************************************************
 *******************************************************************************
						GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

queue_id_t queueInit (void)
{
	static queue_id_t id = 0;

	if (id < QUEUES_MAX_CANT)
		queueClear(id);
	else
		id = QUEUES_MAX_CANT;

	return id++;
}

count_t queuePush (queue_id_t id, data_t data) // Enqueue
{
	count_t count = queueSize(id);

	if (!queueIsFull(id))
	{
		queues[id].items[queues[id].rear] = data;
		queues[id].rear = (queues[id].rear + 1) % (QUEUE_OVERFLOW);
	}

	return count + 1;
}

data_t queuePop (queue_id_t id) // Dequeue
{
	data_t data = queues[id].items[queues[id].front];

	if (!queueIsEmpty(id))
		queues[id].front = (queues[id].front + 1) % (QUEUE_OVERFLOW);
	else
		data = QUEUE_UNDERFLOW;

	return data;
}

data_t	queueFront		(queue_id_t id) { return !queueIsEmpty(id) ? queues[id].items[queues[id].front] : QUEUE_UNDERFLOW; }
data_t	queueBack		(queue_id_t id) { return !queueIsEmpty(id) ? queues[id].items[queues[id].rear - 1] : QUEUE_UNDERFLOW; }
bool	queueIsEmpty	(queue_id_t id) { return !queueSize(id); }
bool	queueIsFull		(queue_id_t id) { return queueSize(id) == QUEUE_MAX_SIZE; }
count_t	queueSize		(queue_id_t id) { return (queues[id].rear - queues[id].front + QUEUE_OVERFLOW) % QUEUE_OVERFLOW; }
void	queueClear		(queue_id_t id) { queues[id].front = queues[id].rear = 0; }


/******************************************************************************/
