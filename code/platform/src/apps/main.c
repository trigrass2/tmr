/* Standard includes. */
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

unsigned int count = 0;

void vLoopTask( void *pvParameters );

int main(void)
{	
	unsigned int exit = 0;
	unsigned int p1=2,p2=10;
	
	xTaskCreate( vLoopTask, ( signed portCHAR * ) "LOOP-1", configMINIMAL_STACK_SIZE*2, (void*)&p1, 1, NULL );
	xTaskCreate( vLoopTask, ( signed portCHAR * ) "LOOP-2", configMINIMAL_STACK_SIZE*2, (void*)&p2, 2, NULL );
	/* Start the scheduler. */
	vTaskStartScheduler();
	
	while(exit == 1);

	return 0;
}

void vLoopTask( void *pvParameters )
{
    unsigned int delay = 0;

    if( pvParameters != NULL )
        delay = *((unsigned int*)pvParameters);
    else
        delay = 2;

	while(1)
	{
		count++;
		vTaskDelay(delay);
	}
}

