#include "tmr_board.h"

void System_Init(void);
void vLoopTask_ON( void *pvParameters );
void vLoopTask_OFF( void *pvParameters );
void vI2C_Task( void *pvParameters );
//void Beep(void);

xSemaphoreHandle xSemaphore;

int main(void)
{
    unsigned int exit = 0;
    
    System_Init();
    
    xTaskCreate( vLoopTask_ON, ( signed portCHAR * ) "beep_on", configMINIMAL_STACK_SIZE*2, NULL, 1, NULL );
    xTaskCreate( vLoopTask_OFF, ( signed portCHAR * ) "beep_off", configMINIMAL_STACK_SIZE*2, NULL, 2, NULL );
    xTaskCreate( vI2C_Task, ( signed portCHAR * ) "vI2C_Task", configMINIMAL_STACK_SIZE*2, NULL, 3, NULL );

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
    while(1)
    {
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

void Beep(void)
{
    unsigned int delay = 5 / portTICK_RATE_MS;

    //BEEP_ON
    vTaskDelay(delay);
    BEEP_OFF
}

#include "cpal_i2c.h"
#include "drv_mpu6050.h"
#include "drv_hmc5883l.h"

uint8_t I2C_ReadReg(uint8_t DeviceAddr, uint8_t RegisterAddr);
uint8_t I2C_WriteReg(uint8_t DeviceAddr, uint8_t RegisterAddr, uint8_t RegisterValue);

/* Select clock Speed */
/* To use the I2C at 400 KHz (in fast mode), the PCLK1 frequency (I2C peripheral
   input clock) must be a multiple of 10 MHz */

//#define I2C_IT
#define I2C_SPEED                   400000

#define TIMEOUT_MAX              0x3000 /*<! The value of the maximal timeout for I2C waiting loops */

uint32_t I2C_TimeOut = TIMEOUT_MAX; /*<! Value of Timeout when I2C communication fails */

CPAL_TransferTypeDef  I2C_RXTransfer = { 
                        /* Initialize TX Transfer structure */
                        pNULL,
                        0,
                        0,
                        0};

CPAL_TransferTypeDef  I2C_TXTransfer = {
                        /* Initialize RX Transfer structure */
                        pNULL,
                        0,
                        0,
                        0};
                        
uint8_t I2C_Buffer[2] = {0x00,0x00};

extern CPAL_InitTypeDef I2C2_DevStructure;

uint8_t I2C_ReadReg(uint8_t DeviceAddr, uint8_t RegisterAddr)
{
  I2C_Buffer[0] = 0xFF;
  
  /* Disable all options */
  I2C2_DevStructure.wCPAL_Options  = 0;
  
  /* point to CPAL_TransferTypeDef structure */
  I2C2_DevStructure.pCPAL_TransferRx = &I2C_RXTransfer;
  
  /* Configure transfer parameters */  
  I2C2_DevStructure.pCPAL_TransferRx->wNumData = 1;
  I2C2_DevStructure.pCPAL_TransferRx->pbBuffer = I2C_Buffer;
  I2C2_DevStructure.pCPAL_TransferRx->wAddr1   = (uint32_t)(DeviceAddr << 1); // this is very important!!
  I2C2_DevStructure.pCPAL_TransferRx->wAddr2   = (uint32_t)RegisterAddr;
  
  /* Read Operation */
  if(CPAL_I2C_Read(&I2C2_DevStructure) == CPAL_PASS)
  {
    while ((I2C2_DevStructure.CPAL_State != CPAL_STATE_READY) && (I2C2_DevStructure.CPAL_State != CPAL_STATE_ERROR) )
    { }
  }
  /* Return the read value */
  return I2C_Buffer[0];
}

uint8_t I2C_WriteReg(uint8_t DeviceAddr, uint8_t RegisterAddr, uint8_t RegisterValue)
{
  uint32_t read_verif = 0;
  
  I2C_Buffer[0] = RegisterValue;
  
  /* Disable all options */
  I2C2_DevStructure.wCPAL_Options  = 0;
  
  /* point to CPAL_TransferTypeDef structure */
  I2C2_DevStructure.pCPAL_TransferTx = &I2C_TXTransfer;
  
  /* Configure transfer parameters */  
  I2C2_DevStructure.pCPAL_TransferTx->wNumData = 1;
  I2C2_DevStructure.pCPAL_TransferTx->pbBuffer = I2C_Buffer ;
  I2C2_DevStructure.pCPAL_TransferTx->wAddr1   = (uint32_t)(DeviceAddr << 1);
  I2C2_DevStructure.pCPAL_TransferTx->wAddr2   = (uint32_t)RegisterAddr;
  
  /* Write Operation */
  if (CPAL_I2C_Write(&I2C2_DevStructure)== CPAL_PASS)
  {
    while ((I2C2_DevStructure.CPAL_State != CPAL_STATE_READY) && (I2C2_DevStructure.CPAL_State != CPAL_STATE_ERROR) )
    { }  
  }
  
#ifdef VERIFY_WRITTENDATA
  /* Verify (if needed) that the loaded data is correct  */
  
  /* Read the just written register*/
  read_verif = I2C_ReadReg(DeviceAddr, RegisterAddr);
  /* Load the register and verify its value  */
  if (read_verif != RegisterValue)
  {
    /* Control data wrongly transferred */
    read_verif = 0x01;
  }
  else
  {
    /* Control data correctly transferred */
    read_verif = 0;
  }
#endif
  
  /* Return the verifying value: 0 (Passed) or 1 (Failed) */
  return read_verif;
}

void vI2C_Task( void *pvParameters )
{ 
    /* Set CPAL structure parameters to their default values */  
    CPAL_I2C_StructInit(&I2C2_DevStructure);
      
    /* Set I2C clock speed */
    I2C2_DevStructure.pCPAL_I2C_Struct->I2C_ClockSpeed = I2C_SPEED;
  
    #ifdef I2C_IT
    /* Select Interrupt programming model and disable all options */
    I2C2_DevStructure.CPAL_ProgModel = CPAL_PROGMODEL_INTERRUPT;
    I2C2_DevStructure.wCPAL_Options  = 0;
    #else
    /* Select DMA programming model and activate TX_DMA_TC and RX_DMA_TC interrupts */
    I2C2_DevStructure.CPAL_ProgModel = CPAL_PROGMODEL_DMA;
    I2C2_DevStructure.wCPAL_Options  = 0;
    #endif /* IOE_IT */

    /* point to CPAL_TransferTypeDef structure */
    I2C2_DevStructure.pCPAL_TransferTx = &I2C_TXTransfer;
    I2C2_DevStructure.pCPAL_TransferRx = &I2C_RXTransfer;
  
    /* Deinitialize CPAL peripheral */
    CPAL_I2C_DeInit(&I2C2_DevStructure);

    /* Initialize CPAL peripheral */
    CPAL_I2C_Init(&I2C2_DevStructure);

    // MPU6050 bypass mode
    
    I2C_WriteReg(MPU6050_ADDRESS_AD0_HIGH, MPU6050_RA_PWR_MGMT_1, 1 << MPU6050_PWR1_DEVICE_RESET_BIT); // Reset

    
    vTaskDelay(100 / portTICK_RATE_MS);

    I2C_WriteReg(MPU6050_ADDRESS_AD0_HIGH, MPU6050_RA_PWR_MGMT_1, 0x0); // Leave sleep mpde

    vTaskDelay(100 / portTICK_RATE_MS);

    I2C_WriteReg(MPU6050_ADDRESS_AD0_HIGH, MPU6050_RA_INT_PIN_CFG, 1 << MPU6050_INTCFG_I2C_BYPASS_EN_BIT); // Disable MST I2C

    vTaskDelay(100 / portTICK_RATE_MS);
    
    while(1)
    {
        I2C_ReadReg(MPU6050_ADDRESS_AD0_HIGH,MPU6050_RA_WHO_AM_I);
        I2C_ReadReg(HMC5883L_ADDRESS,HMC5883L_RA_ID_A);
        I2C_ReadReg(HMC5883L_ADDRESS,HMC5883L_RA_ID_B);
        I2C_ReadReg(HMC5883L_ADDRESS,HMC5883L_RA_ID_C);

        vTaskDelay(10000 / portTICK_RATE_MS);

        #if 1
        BEEP_ON
        vTaskDelay(100 / portTICK_RATE_MS);
        BEEP_OFF
        vTaskDelay(100 / portTICK_RATE_MS);
        BEEP_ON
        vTaskDelay(100 / portTICK_RATE_MS);
        BEEP_OFF
        vTaskDelay(100 / portTICK_RATE_MS);
        #endif
    }
}

void vApplicationTickHook( void )
{
    CPAL_I2C_TIMEOUT_Manager();
}
