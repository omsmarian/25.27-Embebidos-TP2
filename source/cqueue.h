/***************************************************************************//**
  @file     cqueue.h
  @brief    Circular Queue / Ring Buffer / FIFO Implementation
  @author   Group 4: - Oms, Mariano
                     - Solari Raigoso, Agustín
                     - Wickham, Tomás
                     - Vieira, Valentin Ulises
  @note     Based on the work of Daniel Jacoby
 ******************************************************************************/

#ifndef _CQUEUE_H_
#define _CQUEUE_H_

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include <stdbool.h>
#include <stdint.h>

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

// Queues configuration ////////////////////////////////////////////////////////

#define QUEUE_MAX_SIZE		24													// Complete with desired queue size
#define QUEUES_MAX_CANT		2													// Complete with desired number of queues

// Invalid data for the queue //////////////////////////////////////////////////

#define QUEUE_INVALID_ID	255													// Complete according to QUEUES_MAX_CANT
#define INVALID_DATA		0xFF												// Complete with your invalid data, according to data_t

#define QUEUE_OVERFLOW		(QUEUE_MAX_SIZE + 1)
#define QUEUE_UNDERFLOW		(INVALID_DATA)

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

// Data types for the queue ////////////////////////////////////////////////////

typedef uint8_t queue_id_t;														// Complete according to QUEUES_MAX_CANT
typedef unsigned char data_t;													// Complete with your data type
typedef uint8_t count_t;														// Complete according to QUEUE_MAX_SIZE

/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/**
 * @brief Request a queue (initialize)
 * @return ID of the queue to use
 */
queue_id_t queueInit (void);

/**
 * @brief Inserts an element at the back of the queue
 * @param id Queue ID
 * @param data Data to be pushed
 * @return Number of elements in queue
 * @note If the queue is full, the data is not pushed and QUEUE_OVERFLOW is returned
 */
count_t queuePush (queue_id_t id, data_t data);

/**
 * @brief Removes an element from the front of the queue
 * @param id Queue ID
 * @return Data popped
 * @note If the queue is empty, the data is not pulled and QUEUE_UNDERFLOW is returned
 */
data_t queuePop (queue_id_t id);

/**
 * @brief Access next element
 * @param id Queue ID
 * @return Data at the front of the queue
 */
data_t queueFront (queue_id_t id);

/**
 * @brief Access last element
 * @param id Queue ID
 * @return Data at the back of the queue
 */
data_t queueBack (queue_id_t id);

/**
 * @brief Check if the queue is empty
 * @param id Queue ID
 * @return true if empty
 */
bool queueIsEmpty (queue_id_t id);

/**
 * @brief Check if the queue is full
 * @param id Queue ID
 * @return true if full
 */
bool queueIsFull (queue_id_t id);

/**
 * @brief Get the number of elements in the queue
 * @param id Queue ID
 * @return Remaining items count
 */
count_t queueSize (queue_id_t id);

/**
 * @brief Clear the queue
 * @param id Queue ID
 */
void queueClear (queue_id_t id);

/*******************************************************************************
 ******************************************************************************/

#endif // _CQUEUE_H_
