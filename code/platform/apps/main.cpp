#include "tmr_board.h"

/*
const unsigned int LKTB data[24]={
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
}
*/

class A {
    static const unsigned int masks[];
};

const unsigned int A::masks[] = {
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
};

void vLoopTask_ON( void *pvParameters );
void vLoopTask_OFF( void *pvParameters );
void System_Init(void);
void BEEP(void);

xSemaphoreHandle xSemaphore;

int main(void)
{
    unsigned int exit = 0;
    
    //class A test_data;
    //exit = (unsigned int)test_data.masks[1];

    System_Init();
    
    xTaskCreate( vLoopTask_ON, ( signed portCHAR * ) "LOOP-ON", configMINIMAL_STACK_SIZE*2, NULL, 1, NULL );
    xTaskCreate( vLoopTask_OFF, ( signed portCHAR * ) "LOOP-OFF", configMINIMAL_STACK_SIZE*2, NULL, 2, NULL );
    //xTaskCreate( vLoopTask, ( signed portCHAR * ) "LOOP-2", configMINIMAL_STACK_SIZE*2, NULL, 2, NULL );

    /* Start the scheduler. */
    vTaskStartScheduler();

    while(exit == 1);

    return 0;
}

void vLoopTask_ON( void *pvParameters )
{
    //unsigned int delay = 5 / portTICK_RATE_MS;
    
    xSemaphore = xSemaphoreCreateMutex();

    if( xSemaphore == NULL )
    {
        //BEEP_ON
    }
    
    while(1)
    {
        //BEEP();
        //BEEP_ON
        //vTaskDelay(delay);
        if( xSemaphore != NULL )
        {
            // See if we can obtain the semaphore.  If the semaphore is not available
            // wait 10 ticks to see if it becomes free.	
            if( xSemaphoreTake( xSemaphore, ( portTickType ) 10 ) == pdTRUE )
            {
                // We were able to obtain the semaphore and can now access the
                // shared resource.

                // ...

                // We have finished accessing the shared resource.  Release the 
                // semaphore.
                //xSemaphoreGive( xSemaphore );
                BEEP_ON
                //vTaskDelay(delay);
            }
        }
    }
}

void vLoopTask_OFF( void *pvParameters )
{
    //unsigned int delay = 1000 / portTICK_RATE_MS;
    
    while(1)
    {
        //BEEP();
        vTaskDelay(5 / portTICK_RATE_MS);
        BEEP_OFF
        vTaskDelay(5000 / portTICK_RATE_MS);
        xSemaphoreGive( xSemaphore );
    }
}

#define digitalHi(p, i)     { p->BSRRL = i; }
#define digitalLo(p, i)     { p->BSRRH = i; }
#define digitalToggle(p, i) { p->ODR ^= i; }

// Hardware definitions and GPIO

#define BEEP_GPIO   GPIOB
#define BEEP_PIN    GPIO_Pin_12

#define BEEP_TOGGLE digitalToggle(BEEP_GPIO, BEEP_PIN);
#define BEEP_ON    digitalHi(BEEP_GPIO, BEEP_PIN);
#define BEEP_OFF     digitalLo(BEEP_GPIO, BEEP_PIN);

void System_Init(void)
{
    GPIO_InitTypeDef  GPIO_PB;
    
    // BEEP
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_PB.GPIO_Pin = BEEP_PIN;
    GPIO_PB.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_PB.GPIO_OType = GPIO_OType_PP;
    GPIO_PB.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_PB.GPIO_PuPd = GPIO_PuPd_NOPULL;
    
    GPIO_Init(BEEP_GPIO, &GPIO_PB);
}

void BEEP(void)
{
    unsigned int delay = 5 / portTICK_RATE_MS;
    
    //GPIO_SetBits(GPIOB, GPIO_Pin_12);
    //BEEP_ON
    vTaskDelay(delay);
    BEEP_OFF
    //GPIO_ResetBits(GPIOB, GPIO_Pin_12);
}
