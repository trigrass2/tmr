/* Standard includes. */
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* ST StdPeriph Driver includes. */
#include "stm32f4xx_conf.h"

unsigned int count = 0;
unsigned int p1=2,p2=10,p3=10000;

void vLoopTask( void *pvParameters );
void vDiagTask( void *pvParameters );

int main(void)
{	
	unsigned int exit = 0;

	xTaskCreate( vLoopTask, ( signed portCHAR * ) "LOOP-1", configMINIMAL_STACK_SIZE*2, (void*)&p1, 1, NULL );
	xTaskCreate( vLoopTask, ( signed portCHAR * ) "LOOP-2", configMINIMAL_STACK_SIZE*2, (void*)&p2, 2, NULL );
	xTaskCreate( vDiagTask, ( signed portCHAR * ) "DIAG-3", configMINIMAL_STACK_SIZE*2, (void*)&p3, 3, NULL );
	
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

void vDiagTask( void *pvParameters )
{
    GPIO_InitTypeDef  GPIO_BEEP;
    GPIO_InitTypeDef  GPIO_I2C;
    unsigned int delay = 0;
    //const portTickType xDelay = 5000 / portTICK_RATE_MS;

    if( pvParameters != NULL )
        delay = *((unsigned int*)pvParameters);
    else
        delay = 2;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_BEEP.GPIO_Pin = GPIO_Pin_12;
    GPIO_BEEP.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_BEEP.GPIO_OType = GPIO_OType_PP;
    GPIO_BEEP.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_BEEP.GPIO_PuPd = GPIO_PuPd_NOPULL;
    
    GPIO_Init(GPIOB, &GPIO_BEEP);

    GPIO_I2C.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
    GPIO_I2C.GPIO_Mode = GPIO_Mode_AF;
    GPIO_I2C.GPIO_OType = GPIO_OType_OD;
    GPIO_I2C.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_I2C.GPIO_PuPd = GPIO_PuPd_NOPULL;
    
    GPIO_Init(GPIOB, &GPIO_I2C);
    
	while(1)
	{
		GPIO_SetBits(GPIOB, GPIO_Pin_12);
		delay = 10 / portTICK_RATE_MS;
		vTaskDelay(delay);
		GPIO_ResetBits(GPIOB, GPIO_Pin_12);
		delay = 2000 / portTICK_RATE_MS;
		vTaskDelay(delay);
	}
}
