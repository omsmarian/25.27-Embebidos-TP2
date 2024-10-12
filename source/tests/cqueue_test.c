#include <stdio.h>
#include <time.h>
#include "cqueue.h"

int main (void)
{
	queue_id_t id = queueInit();

	struct timeval st, et;
	mingw_gettimeofday(&st,NULL);
	for(int i = 0; i < 1000; i++)
	{
		queuePush(id, i);
		queuePop(id);
	}
	mingw_gettimeofday(&et,NULL);
	double elapsed = ((double)(et.tv_sec - st.tv_sec) * 1000000) + (double)(et.tv_usec - st.tv_usec);
	printf("Time elapsed: %f micro seconds\n", elapsed);

    printf("Size: %d\n", queueSize(id));
    printf("Push: %d, Pull: %d\n", queuePush(id, 25), queuePop(id));

    count_t i = -1;
    while(++i <= QUEUE_OVERFLOW - 4)
// 	while(queueSize(id) != QUEUE_MAX_SIZE)
// 	while(!queueIsFull(id))
	{
    	if(queuePush(id, i) == QUEUE_OVERFLOW)
    		printf("Queue Overflowed, Size: %d\n", queueSize(id));
		else
		    printf("Data pushed: %d, Size: %d\n", i, queueSize(id));
    }

    printf("Front: %d, Back: %d\n", queueFront(id), queueBack(id));

    // queueClear(id);
    printf("Front: %d, Back: %d, Size: %d\n", queueFront(id), queueBack(id), queueSize(id));
    printf("Push: %d, Pull: %d\n", queuePush(id, i), queuePop(id));
    printf("Pull: %d, Push: %d\n", queuePop(id), queuePush(id, i));

    i = QUEUE_OVERFLOW + 1;
    while(i--)
// 	while(queueSize(id))
//  while(!queueIsEmpty(id))
	{
		data_t data = queuePop(id);
		if(data == QUEUE_UNDERFLOW)
    		printf("Queue Underflowed, Size %d\n", queueSize(id));
		else
		    printf("Data pulled: %d, Remaining: %d\n", data, queueSize(id));
	}

    printf("Front: %d, Back: %d\n", queueFront(id), queueBack(id));

	return 0;
}
