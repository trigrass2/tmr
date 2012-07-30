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
	
	xTaskCreate( vLoopTask, ( signed portCHAR * ) "LOOP-1", configMINIMAL_STACK_SIZE*2, NULL, 1, NULL );
	xTaskCreate( vLoopTask, ( signed portCHAR * ) "LOOP-2", configMINIMAL_STACK_SIZE*2, NULL, 2, NULL );
	/* Start the scheduler. */
	vTaskStartScheduler();
	
	while(exit == 1);

	return 0;
}

void vLoopTask( void *pvParameters )
{
	while(1)
	{
		count++;
		vTaskDelay(2);
	}
}
