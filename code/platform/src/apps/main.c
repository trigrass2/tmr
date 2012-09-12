/* Standard includes. */
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* ST StdPeriph Driver includes. */
#include "stm32f4xx_conf.h"

#include "tmr_board.h"
#include "drv_mpu6050.h"
#include "drv_hmc5883l.h"
#include "drv_sdio_sd.h"


/** @addtogroup SDIO_uSDCard
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;

/* Private define ------------------------------------------------------------*/
#define BLOCK_SIZE            512 /* Block Size in Bytes */

#define NUMBER_OF_BLOCKS      100  /* For Multi Blocks operation (Read/Write) */
#define MULTI_BUFFER_SIZE    (BLOCK_SIZE * NUMBER_OF_BLOCKS)

#define SD_OPERATION_ERASE          0
#define SD_OPERATION_BLOCK          1
#define SD_OPERATION_MULTI_BLOCK    2 
#define SD_OPERATION_END            3

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t Buffer_Block_Tx[BLOCK_SIZE], Buffer_Block_Rx[BLOCK_SIZE];
uint8_t Buffer_MultiBlock_Tx[MULTI_BUFFER_SIZE], Buffer_MultiBlock_Rx[MULTI_BUFFER_SIZE];
volatile TestStatus EraseStatus = FAILED, TransferStatus1 = FAILED, TransferStatus2 = FAILED;
SD_Error Status = SD_OK;
__IO uint32_t SDCardOperation = SD_OPERATION_ERASE;

/* Private function prototypes -----------------------------------------------*/
void NVIC_Configuration(void);
void SD_EraseTest(void);
void SD_SingleBlockTest(void);
void SD_MultiBlockTest(void);
void Fill_Buffer(uint8_t *pBuffer, uint32_t BufferLength, uint32_t Offset);
TestStatus Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint32_t BufferLength);
TestStatus eBuffercmp(uint8_t* pBuffer, uint32_t BufferLength);

/* Private functions ---------------------------------------------------------*/


unsigned int count = 0;
unsigned int p1=5000,p2=1000,p3=10000;
unsigned int data[6] = {0};
u8 data_mpu[118] = {0};
u8 data_hmc[13] = {0};
unsigned int data_g[6] = {0};

//#define BEEP_ENABLE

// MS5611
unsigned long D1;    // ADC value of the pressure conversion
unsigned long D2;    // ADC value of the temperature conversion
unsigned int C[8];   // calibration coefficients
double P;   // compensated pressure value
double T;   // compensated temperature value
double dT;   // difference between actual and measured temperature
double OFF;   // offset at actual temperature
double SENS;   // sensitivity at actual temperature

unsigned char ms_crc4 = 0;
unsigned int nprom[] = {0x3132,0x3334,0x3536,0x3738,0x3940,0x4142,0x4344,0x450B}; // CRC =0x0B so..  0x4500 + 0x0B

// MPU6050
s16 temp_data = 0;
double temp_deg = 0.0;

#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

// BATT
__IO uint16_t ADCConvertedValue = 0;
__IO uint32_t VBATVoltage = 0;
#define ADC1_DR_ADDRESS    ((uint32_t)0x4001204C)

// RTC
RTC_InitTypeDef RTC_InitStruct;
RTC_TimeTypeDef RTC_TimeStruct;
RTC_DateTypeDef RTC_DateStruct;
RTC_TimeTypeDef  RTC_TimeStampStruct;
RTC_DateTypeDef  RTC_TimeStampDateStruct;
__IO uint32_t AsynchPrediv = 0, SynchPrediv = 0;



void vLoopTask( void *pvParameters );
void vDiagTask( void *pvParameters );
uint8_t I2C_ByteRead(u16 Addr, u8 Reg);
void I2C_ByteWrite(u16 Addr, u8 Reg,u8 Data,u8 Cmd);
void I2C_MutiRead(u8* pBuffer, u8 Addr, u8 Reg,u8 Count);
void I2C2_Configuration(void);
unsigned char crc4(unsigned int n_prom[]);
uint8_t USART_Scanf(uint32_t MinValue, uint32_t MaxValue);
void RTC_TimeShow(void);
void RTC_DateShow(void);
void RTC_TimeStampShow(void);
void RTC_TimeRegulate(void);
void RTC_Config(void);
void System_Init(void);

int main(void)
{
    unsigned int exit = 0;

    xTaskCreate( vLoopTask, ( signed portCHAR * ) "LOOP-1", configMINIMAL_STACK_SIZE*2, (void*)&p1, 3, NULL );
    xTaskCreate( vLoopTask, ( signed portCHAR * ) "LOOP-2", configMINIMAL_STACK_SIZE*2, (void*)&p2, 2, NULL );
    xTaskCreate( vDiagTask, ( signed portCHAR * ) "DIAG-3", configMINIMAL_STACK_SIZE*2, (void*)&p3, 1, NULL );

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

    //------------------------------------------------
    // LOOP
    //------------------------------------------------
    while(1)
    {
        count++;

        #if 0
        GPIO_SetBits(GPIOB, GPIO_Pin_12);
        vTaskDelay(20);
        GPIO_ResetBits(GPIOB, GPIO_Pin_12);
        #endif
        vTaskDelay((delay / portTICK_RATE_MS));
    }
    //------------------------------------------------
}

#define PCA9536DP_ADDRESS    0x41
#define PCA9533DP_ADDRESS    0x62
#define MPU6050_ADDRESS      0x69
#define HMC5883L_ADDRESS     0x1E
#define MS5611_ADDRESS       0x77
#define I2C_SPEED            400000

// MS5611
#define CMD_RESET            0x1E // ADC reset command
#define CMD_ADC_READ         0x00 // ADC read command
#define CMD_ADC_CONV         0x40 // ADC conversion command
#define CMD_ADC_D1           0x00 // ADC D1 conversion
#define CMD_ADC_D2           0x10 // ADC D2 conversion
#define CMD_ADC_256          0x00 // ADC OSR=256
#define CMD_ADC_512          0x02 // ADC OSR=512
#define CMD_ADC_1024         0x04 // ADC OSR=1024
#define CMD_ADC_2048         0x06 // ADC OSR=2056
#define CMD_ADC_4096         0x08 // ADC OSR=4096
#define CMD_PROM_RD          0xA0 // Prom read command

// PCA9536DP : 4-bit I2C-bus and SMBus I/O port
#define REG_I           0x00
#define REG_O           0x01
#define REG_N           0x02
#define REG_C           0x03

#define I0_BIT          0x00
#define I1_BIT          0x01
#define I2_BIT          0x02
#define I3_BIT          0x03

#define O0_BIT          0x00
#define O1_BIT          0x01
#define O2_BIT          0x02
#define O3_BIT          0x03

#define N0_BIT          0x00
#define N1_BIT          0x01
#define N2_BIT          0x02
#define N3_BIT          0x03

#define C0_BIT          0x00
#define C1_BIT          0x01
#define C2_BIT          0x02
#define C3_BIT          0x03



u8 pca9536dp[4]={0};




#define TRUE   (1)                      /* Boolean true value.   */
#define FALSE  (0)                      /* Boolean false value.  */


void vDiagTask( void *pvParameters )
{
    u16 delay=0, i=0;
    u32 count=0;
    u32 tick_s=0, tick_e=0;
    double vbatV=0;

    if( pvParameters != NULL )
        delay = *((unsigned int*)pvParameters);
    else
        delay = 2;

    System_Init();
    
    /*------------------------------------------------
         Test PCA9533
         ------------------------------------------------*/
    I2C_ByteWrite(PCA9533DP_ADDRESS, 0x01, 0xFF, FALSE); // PSC0 : P = 1.68
    I2C_ByteWrite(PCA9533DP_ADDRESS, 0x02, 0x40, FALSE); // PWM0 50%(0x80)
    I2C_ByteWrite(PCA9533DP_ADDRESS, 0x03, 0x97, FALSE); // PSC1 : P = 1.00
    I2C_ByteWrite(PCA9533DP_ADDRESS, 0x04, 0x40, FALSE); // PWM1 25%(0x40)

    I2C_ByteWrite(PCA9533DP_ADDRESS, 0x05, 0x03, FALSE); // LS0

    #ifdef BEEP_ENABLE
    GPIO_SetBits(GPIOB, GPIO_Pin_12);
    vTaskDelay(20);
    GPIO_ResetBits(GPIOB, GPIO_Pin_12);
    vTaskDelay(500);
    #endif
    //------------------------------------------------

    /*------------------------------------------------
        Test PCA9536 and APC250
        ------------------------------------------------*/
    I2C_ByteWrite(PCA9533DP_ADDRESS, 0x05, 0x0F, FALSE); // LS0

    #ifdef BEEP_ENABLE
    GPIO_SetBits(GPIOB, GPIO_Pin_12);
    vTaskDelay(20);
    GPIO_ResetBits(GPIOB, GPIO_Pin_12);
    vTaskDelay(500);
    #endif

    for(i=0 ; i<4 ; i++)
    {
        pca9536dp[i]=I2C_ByteRead(PCA9536DP_ADDRESS, i);
    }
    // Config pins with Output
    // IO0 : APC_EN  -> O
    // IO1 : APC_SET - > O
    // IO2 : APC_AUX -> I
    // IO3 : PWR_LED -> O
    //pca9536dp[REG_C] = 0xF4
    I2C_ByteWrite(PCA9536DP_ADDRESS, REG_C, 0xF4, NULL); // IO2 us input  pin
    I2C_ByteWrite(PCA9536DP_ADDRESS, REG_O, 0xF6, NULL); // 11110110 , APC250 EN = Low
    //------------------------------------------------

    //------------------------------------------------
    // Test MPU6050
    //------------------------------------------------
    I2C_ByteWrite(PCA9533DP_ADDRESS, 0x05, 0x3F, FALSE); // LS0

    #ifdef BEEP_ENABLE
    GPIO_SetBits(GPIOB, GPIO_Pin_12);
    vTaskDelay(20);
    GPIO_ResetBits(GPIOB, GPIO_Pin_12);
    vTaskDelay(500);
    #endif

    I2C_ByteWrite(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 1 << MPU6050_PWR1_DEVICE_RESET_BIT, FALSE); // Reset

    delay = 100 / portTICK_RATE_MS; // delay 100 ms
    vTaskDelay(delay);

    I2C_ByteWrite(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0x0, FALSE); // Leave sleep mpde

    delay = 100 / portTICK_RATE_MS; // delay 100 ms
    vTaskDelay(delay);

    I2C_ByteWrite(MPU6050_ADDRESS, MPU6050_RA_INT_PIN_CFG, 1 << MPU6050_INTCFG_I2C_BYPASS_EN_BIT, FALSE); // Disable MST I2C

    delay = 100 / portTICK_RATE_MS; // delay 100 ms
    vTaskDelay(delay);

    #if 1
    for(i=0 ; i<118 ; i++)
    {
        data_mpu[i]=I2C_ByteRead(MPU6050_ADDRESS, i);
    }
    #else
    I2C_MutiRead((u8*)data_mpu, MPU6050_ADDRESS, 0x0, 118);
    #endif
    temp_data = (data_mpu[MPU6050_RA_TEMP_OUT_H]<<8) | data_mpu[MPU6050_RA_TEMP_OUT_L];
    temp_deg = (double)(temp_data/340.0) + 36.53;
    //------------------------------------------------

    //------------------------------------------------
    // Test MS5611
    //------------------------------------------------
    I2C_ByteWrite(PCA9533DP_ADDRESS, 0x05, 0xFF, FALSE); // LS0

    #ifdef BEEP_ENABLE
    GPIO_SetBits(GPIOB, GPIO_Pin_12);
    vTaskDelay(20);
    GPIO_ResetBits(GPIOB, GPIO_Pin_12);
    vTaskDelay(500);
    #endif

    I2C_ByteWrite(MS5611_ADDRESS, CMD_RESET, NULL, TRUE); // Reset command

    delay = 100 / portTICK_RATE_MS; // delay 100 ms
    vTaskDelay(delay);

    // PROM READ SEQUENCE
    for(i=0 ; i<8 ; i++)
    {
        I2C_MutiRead((u8*)&(C[i]), MS5611_ADDRESS, (CMD_PROM_RD + (i<<1)), 2);
        C[i]= (C[i]>>8) | ((C[i]&0x00FF)<<8);
    }

    // CONVERSION SEQUENCE

    // D1
    I2C_ByteWrite(MS5611_ADDRESS, (CMD_ADC_CONV|CMD_ADC_D1|CMD_ADC_4096), NULL, TRUE); // ADC command for MS5611

    delay = 200 / portTICK_RATE_MS; // delay 200 ms
    vTaskDelay(delay);

    I2C_MutiRead((u8*)&(D1), MS5611_ADDRESS, CMD_ADC_READ, 3);

    // D2
    I2C_ByteWrite(MS5611_ADDRESS, (CMD_ADC_CONV|CMD_ADC_D2|CMD_ADC_4096), NULL, TRUE); // ADC command for MS5611

    delay = 200 / portTICK_RATE_MS; // delay 200 ms
    vTaskDelay(delay);

    I2C_MutiRead((u8*)&(D2), MS5611_ADDRESS, CMD_ADC_READ, 3);

    // CYCLIC REDUNDANCY CHECK (CRC)
    ms_crc4=crc4(C);

    //------------------------------------------------

    //------------------------------------------------
    // Test HMC5883L
    //------------------------------------------------
    I2C_ByteWrite(PCA9533DP_ADDRESS, 0x05, 0x55, FALSE); // LS0

    #ifdef BEEP_ENABLE
    GPIO_SetBits(GPIOB, GPIO_Pin_12);
    vTaskDelay(20);
    GPIO_ResetBits(GPIOB, GPIO_Pin_12);
    vTaskDelay(500);
    #endif

    for(i=0 ; i<13 ; i++)
    {
        data_hmc[i]=I2C_ByteRead(HMC5883L_ADDRESS, i); // to check HMC5883L
    }
    //------------------------------------------------

    //------------------------------------------------
    // LOOP
    //------------------------------------------------
    I2C_ByteWrite(PCA9533DP_ADDRESS, 0x05, 0xBB, FALSE); // LS0

    delay = 1500 / portTICK_RATE_MS;

    while(1)
    {
        #if 0
        GPIO_SetBits(GPIOB, GPIO_Pin_12);
        delay = 20 / portTICK_RATE_MS;
        vTaskDelay(delay);
        GPIO_ResetBits(GPIOB, GPIO_Pin_12);
        #endif

        printf("\n\r---------------------------------------------------------------------------------\n\r");
        vTaskDelay(100);
        printf("\n\rUSART Printf Example: retarget the C library printf function to the USART (%d)\n\r",count++);
        vTaskDelay(100);

        /*The VBAT pin is internally connected to a bridge divider by 2 */
        VBATVoltage = (ADCConvertedValue*2)*3300/0xFFF;
        vbatV=VBATVoltage/1000.0;
        printf("\n\rvbatV = %f\n\r",vbatV);
        vTaskDelay(100);

        /* Display the RTC Time/Date and TimeStamp Time/Date */
        RTC_TimeShow();
        RTC_DateShow();
        RTC_TimeStampShow();

        tick_s=xTaskGetTickCount();
        printf("\n\rEnter Sleep, xTickCount = %d\n\r",tick_s);
        vTaskDelay(100);

        I2C_ByteWrite(PCA9536DP_ADDRESS, 0x01, 0xFF, FALSE); // PWR_LED (ON), APC250 (OFF)
        vTaskDelay(delay);
        I2C_ByteWrite(PCA9536DP_ADDRESS, 0x01, 0xFE, FALSE); // PWR_LED (ON), APC250 (ON)
        vTaskDelay(100);

        tick_e=xTaskGetTickCount();
        printf("\n\rLeave Sleep, xTickCount = %d\n\r",tick_e);
        vTaskDelay(100);
        printf("\n\rTime Spent = %f ms\n\r",(float)(tick_e-tick_s)/portTICK_RATE_MS);
        vTaskDelay(200);

        if((Status == SD_OK) && (SDCardOperation != SD_OPERATION_END) && (SD_Detect()== SD_PRESENT))
        {
            printf("\n\rSD Function test!!\n\r");
            vTaskDelay(100);
            
            switch(SDCardOperation)
            {
                /*-------------------------- SD Erase Test ---------------------------- */
                case (SD_OPERATION_ERASE):
                {
                    printf("\n\rSD_EraseTest()!!\n\r");
                    vTaskDelay(100);
                    
                    SD_EraseTest();
                    SDCardOperation = SD_OPERATION_BLOCK;
                    break;
                }
                /*-------------------------- SD Single Block Test --------------------- */
                case (SD_OPERATION_BLOCK):
                {
                    printf("\n\rSD_SingleBlockTest()!!\n\r");
                    vTaskDelay(100);
                    
                    SD_SingleBlockTest();
                    SDCardOperation = SD_OPERATION_MULTI_BLOCK;
                    break;
                }       
                /*-------------------------- SD Multi Blocks Test --------------------- */
                case (SD_OPERATION_MULTI_BLOCK):
                {
                    printf("\n\rSD_MultiBlockTest()!!\n\r");
                    vTaskDelay(100);
                    
                    SD_MultiBlockTest();
                    SDCardOperation = SD_OPERATION_END;
                    break;
                }              
            }
        }

        
        tick_s=xTaskGetTickCount();
        printf("\n\rEnter Sleep, xTickCount = %d\n\r",tick_s);
        vTaskDelay(100);
        
        I2C_ByteWrite(PCA9536DP_ADDRESS, 0x01, 0xF7, FALSE); // PWR_LED (ON), APC250 (OFF)
        vTaskDelay(delay);
        I2C_ByteWrite(PCA9536DP_ADDRESS, 0x01, 0xF6, FALSE); // PWR_LED (ON), APC250 (ON)
        vTaskDelay(100);

        
        tick_e=xTaskGetTickCount();
        printf("\n\rLeave Sleep, xTickCount = %d\n\r",tick_e);
        vTaskDelay(100);
        printf("\n\rTime Spent = %f ms\n\r",(float)(tick_e-tick_s)/portTICK_RATE_MS);
        vTaskDelay(100);

    }
    //------------------------------------------------
}

/*******************************************************************************
* Function Name  : GPIO_I2C2_Configuration
* Description    :
* Input          :
* Output         :
* Return         :
*******************************************************************************/
void GPIO_I2C2_Configuration(void)
{
    GPIO_InitTypeDef  GPIO_I2C1_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2,ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
    /* Configure I2C1 pins: SCL and SDA PB10 & PB11*/
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_I2C2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_I2C2);

    GPIO_I2C1_InitStructure.GPIO_Pin =  GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_I2C1_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_I2C1_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_I2C1_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_I2C1_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_Init(GPIOB, &GPIO_I2C1_InitStructure);
}

/*******************************************************************************
* Function Name  : I2C2_Configuration
* Description    :
* Input          :
* Output         :
* Return         :
*******************************************************************************/
void I2C2_Configuration(void)
{
    I2C_InitTypeDef  I2C_InitStructure;
    I2C_DeInit(I2C2);
    I2C_Cmd(I2C2, DISABLE);

    /* I2C configuration */
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = I2C_SPEED;

    I2C_Cmd(I2C2, ENABLE);
    I2C_Init(I2C2, &I2C_InitStructure);

    I2C_AcknowledgeConfig(I2C2, ENABLE);
}

/*******************************************************************************
* Function Name  : I2C_ByteRead
* Description    :
* Input          :
* Output         :
* Return         :
*******************************************************************************/
uint8_t I2C_ByteRead(u16 Addr, u8 Reg)
{
    u16 tempADD;
    uint8_t rData;

    //I2C2_Configuration();

    while(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY));

    tempADD=Addr<<1;
    I2C_AcknowledgeConfig(I2C2, ENABLE);
    I2C_GenerateSTART(I2C2, ENABLE);

    /* Test on EV5 and clear it */
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));

    I2C_Send7bitAddress(I2C2, tempADD, I2C_Direction_Transmitter);
    /* Test on EV6 and clear it */
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

    I2C_SendData(I2C2, Reg);

    /* Test on EV8 and clear it */
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    I2C_GenerateSTART(I2C2, ENABLE);
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));

    I2C_Send7bitAddress(I2C2, tempADD, I2C_Direction_Receiver);
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED));

    rData= I2C_ReceiveData(I2C2);
    I2C_AcknowledgeConfig(I2C2, DISABLE);
    I2C_GenerateSTOP(I2C2, ENABLE);

    return rData;
}

/*******************************************************************************
* Function Name  : I2C_ByteWrite
* Description    :
* Input          :
* Output         :
* Return         :
*******************************************************************************/
void I2C_ByteWrite(u16 Addr, u8 Reg,u8 Data,u8 Cmd)
{
    u16 tempADD;

    //I2C2_Configuration();

    while(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY));

    tempADD=Addr<<1;
    I2C_AcknowledgeConfig(I2C2, ENABLE);
    I2C_GenerateSTART(I2C2, ENABLE);

    /* Test on EV5 and clear it */
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));
    I2C_Send7bitAddress(I2C2, tempADD, I2C_Direction_Transmitter);

    /* Test on EV6 and clear it */
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    //I2C_Cmd(I2C2, ENABLE); /* Clear EV6 by setting again the PE bit */ // to verify it

    /* Send the internal address to write to */
    I2C_SendData(I2C2, Reg);

    /* Test on EV8 and clear it */
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    if(Cmd != TRUE)
    {
        /* Send the byte to be written */
        I2C_SendData(I2C2, Data);

        /* Test on EV8 and clear it */
        while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    }

    /* Send STOP condition */
    I2C_GenerateSTOP(I2C2, ENABLE);
    I2C_AcknowledgeConfig(I2C2, ENABLE); // test
}

/*******************************************************************************
* Function Name  : I2C_MutiRead
* Description    :
* Input          :
* Output         :
* Return         :
*******************************************************************************/
void I2C_MutiRead(u8* pBuffer, u8 Addr, u8 Reg,u8 Count)
{
    u8 tempADD;

    //I2C1_Configuration();

    while(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY));
    I2C_AcknowledgeConfig(I2C2, ENABLE);

    /* Send START condition */
    I2C_GenerateSTART(I2C2, ENABLE);
    tempADD=Addr<<1;

    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));    /* Test on EV5 and clear it */
    I2C_Send7bitAddress(I2C2, tempADD, I2C_Direction_Transmitter);

    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));    /* Test on EV6 and clear it */
    //  I2C_Cmd(I2C1, ENABLE); /* Clear EV6 by setting again the PE bit */

    I2C_SendData(I2C2, Reg);
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));    /* Test on EV8 and clear it */

    I2C_GenerateSTART(I2C2, ENABLE);
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));    /* Test on EV5 and clear it */

    I2C_Send7bitAddress(I2C2, tempADD, I2C_Direction_Receiver);
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));    /* Test on EV6 and clear it */

    /* While there is data to be read */
    while(Count)
    {
        if(Count == 1)
        {
            I2C_AcknowledgeConfig(I2C2, DISABLE);    /* Disable Acknowledgement */
            I2C_GenerateSTOP(I2C2, ENABLE);    /* Send STOP Condition */
        }

        /* Test on EV7 and clear it */
        if(I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED))
        // while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
        {
            /* Read a byte from the EEPROM */
            *pBuffer = I2C_ReceiveData(I2C2);
            pBuffer++;    /* Point to the next location where the byte read will be saved */
            Count--;    /* Decrement the read bytes counter */
        }
    }

    /* Enable Acknowledgement to be ready for another reception */
    I2C_AcknowledgeConfig(I2C2, ENABLE);
}

//********************************************************
//! @brief calculate the CRC code for details look into CRC CODE NOTES
//!
//! @return crc code
//********************************************************
unsigned char crc4(unsigned int n_prom[])
{
    int cnt;    // simple counter
    unsigned int n_rem;    // crc reminder
    unsigned int crc_read;    // original value of the crc
    unsigned char  n_bit;
    n_rem = 0x00;
    crc_read=n_prom[7];    //save read CRC
    n_prom[7]=(0xFF00 & (n_prom[7]));    //CRC byte is replaced by 0

    for (cnt = 0; cnt < 16; cnt++)    // operation is performed on bytes
    {
        // choose LSB or MSB
        if (cnt%2==1) n_rem ^= (unsigned short) ((n_prom[cnt>>1]) & 0x00FF);
        else n_rem ^= (unsigned short) (n_prom[cnt>>1]>>8);

        for (n_bit = 8; n_bit > 0; n_bit--)
        {
            if (n_rem & (0x8000))
            {
                n_rem = (n_rem << 1) ^ 0x3000;
            }
            else
            {
                n_rem = (n_rem << 1);
            }
        }
    }

    n_rem=  (0x000F & (n_rem >> 12));    // final 4-bit reminder is CRC code
    n_prom[7]=crc_read;    // restore the crc_read to its original place

    return (n_rem ^ 0x00);
}

#define RTC_CLOCK_SOURCE_LSE

void System_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;
    I2C_InitTypeDef   I2C_InitStruct;
    ADC_InitTypeDef       ADC_InitStruct;
    ADC_CommonInitTypeDef ADC_CommonInitStruct;
    DMA_InitTypeDef       DMA_InitStruct;

    /*------------------------------------------------
          BEEP & GPIO Configuration
     ------------------------------------------------*/
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;

    GPIO_Init(GPIOB, &GPIO_InitStruct);
    //------------------------------------------------

    /*------------------------------------------------
         I2C & GPIO Configuration
     ------------------------------------------------*/
    /*!< I2C Periph clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);

    /*!< I2C_SCL_GPIO_CLK and I2C_SDA_GPIO_CLK Periph clock enable */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    /* Reset I2C IP */
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C2, ENABLE);

    /* Release reset signal of I2C IP */
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C2, DISABLE);

    /*!< GPIO configuration */
    /* Connect PXx to I2C_SCL*/
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_I2C2);
    /* Connect PXx to I2C_SDA*/
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_I2C2);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;

    GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*!< I2C configuration */
    /* I2C configuration */
    I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStruct.I2C_OwnAddress1 = 0;
    I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStruct.I2C_ClockSpeed = I2C_SPEED;

    /* I2C Peripheral Enable */
    I2C_Cmd(I2C2, ENABLE);
    /* Apply I2C configuration after enabling it */
    I2C_Init(I2C2, &I2C_InitStruct);
    //------------------------------------------------

    /*------------------------------------------------
         UART & GPIO Configuration
     ------------------------------------------------*/
    USART_InitStruct.USART_BaudRate = 115200;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    /* Enable GPIO clock */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOC, ENABLE);

    /* Enable UART clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

    /* Connect PXx to USARTx_Tx*/
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);

    /* Connect PXx to USARTx_Rx*/
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);

    /* Configure USART Tx as alternate function  */
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* Configure USART Rx as alternate function  */
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7;
    GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* USART configuration */
    USART_Init(USART6, &USART_InitStruct);

    /* Enable USART */
    USART_Cmd(USART6, ENABLE);

    /*------------------------------------------------
         ADC & VBAT Configuration
     ------------------------------------------------*/
    /* Enable peripheral clocks *************************************************/
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    /* DMA2_Stream0 channel0 configuration **************************************/
    DMA_DeInit(DMA2_Stream0);
    DMA_InitStruct.DMA_Channel = DMA_Channel_0;
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR_ADDRESS;
    DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)&ADCConvertedValue;
    DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStruct.DMA_BufferSize = 1;
    DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Disable;
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStruct.DMA_Priority = DMA_Priority_High;
    DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
    DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream0, &DMA_InitStruct);
    /* DMA2_Stream0 enable */
    DMA_Cmd(DMA2_Stream0, ENABLE);

    /* ADC Common Init **********************************************************/
    ADC_CommonInitStruct.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStruct.ADC_Prescaler = ADC_Prescaler_Div2;
    ADC_CommonInitStruct.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStruct.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStruct);

    /* ADC1 Init ****************************************************************/
    ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStruct.ADC_ScanConvMode = DISABLE;
    ADC_InitStruct.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStruct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStruct.ADC_NbrOfConversion = 1;
    ADC_Init(ADC1, &ADC_InitStruct);

    /* Enable ADC1 DMA */
    ADC_DMACmd(ADC1, ENABLE);

    /* ADC1 regular channel18 (VBAT) configuration ******************************/
    ADC_RegularChannelConfig(ADC1, ADC_Channel_Vbat, 1, ADC_SampleTime_15Cycles);

    /* Enable VBAT channel */
    ADC_VBATCmd(ENABLE);

    /* Enable DMA request after last transfer (Single-ADC mode) */
    ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);

    /* Enable ADC1 **************************************************************/
    ADC_Cmd(ADC1, ENABLE);

    /* Start ADC1 Software Conversion */
    ADC_SoftwareStartConv(ADC1);

    /*------------------------------------------------
         RTC Configuration
     ------------------------------------------------*/
    I2C_ByteWrite(PCA9536DP_ADDRESS, REG_C, 0xF4, NULL); // IO2 us input  pin
    I2C_ByteWrite(PCA9536DP_ADDRESS, REG_O, 0xF6, NULL); // 11110110 , APC250 EN = Low

    /* Output a message on Hyperterminal using printf function */
    printf("\n\r*********************** RTC Time Stamp Example ***********************\n\r");
    vTaskDelay(100);

    if (RTC_ReadBackupRegister(RTC_BKP_DR0) != 0x32F2)
    {
        /* RTC configuration  */
        RTC_Config();

        /* Configure the RTC data register and RTC prescaler */
        RTC_InitStruct.RTC_AsynchPrediv = AsynchPrediv;
        RTC_InitStruct.RTC_SynchPrediv = SynchPrediv;
        RTC_InitStruct.RTC_HourFormat = RTC_HourFormat_24;

        /* Check on RTC init */
        if (RTC_Init(&RTC_InitStruct) == ERROR)
        {
            printf("\n\r        /!\\***** RTC Prescaler Config failed ********/!\\ \n\r");
            vTaskDelay(100);
        }

        /* Configure the time register */
        RTC_TimeRegulate();
    }
    else
    {
        /* Check if the Power On Reset flag is set */
        if (RCC_GetFlagStatus(RCC_FLAG_PORRST) != RESET)
        {
            printf("\r\n Power On Reset occurred....\n\r");
            vTaskDelay(100);
        }
        /* Check if the Pin Reset flag is set */
        else if (RCC_GetFlagStatus(RCC_FLAG_PINRST) != RESET)
        {
            printf("\r\n External Reset occurred....\n\r");
            vTaskDelay(100);
        }

        printf("\r\n No need to configure RTC....\n\r");
        vTaskDelay(100);

        /* Enable the PWR clock */
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

        /* Allow access to RTC */
        PWR_BackupAccessCmd(ENABLE);

        /* Wait for RTC APB registers synchronisation */
        RTC_WaitForSynchro();

        /* Clear the RTC Alarm Flag */
        RTC_ClearFlag(RTC_FLAG_ALRAF);

        /* Clear the EXTI Line 17 Pending bit (Connected internally to RTC Alarm) */
        EXTI_ClearITPendingBit(EXTI_Line17);

        /* Display the RTC Time/Date and TimeStamp Time/Date */
        RTC_TimeShow();
        RTC_DateShow();
        RTC_TimeStampShow();
    }

    /*------------------------------------------------
         RTC Configuration
     ------------------------------------------------*/
    /* Interrupt Config */
    printf("\r\n NVIC_Configuration....\n\r");
    vTaskDelay(100);

    NVIC_Configuration();

    /*------------------------------ SD Init ---------------------------------- */
    printf("\r\n SD_Init....\n\r");
    vTaskDelay(100);

    if((Status = SD_Init()) != SD_OK)
    {
        printf("\n\r>> !! SD init failed. !! <<\n\r");
        vTaskDelay(100);
    }
}

/**
  * @brief  Gets numeric values from the hyperterminal.
  * @param  MinValue: minimum value to be used.
  * @param  MaxValue: maximum value to be used.
  * @retval None
  */
uint8_t USART_Scanf(uint32_t MinValue, uint32_t MaxValue)
{
    uint32_t index = 0;
    uint32_t tmp[2] = {0, 0};

    while (index < 2)
    {
        /* Loop until RXNE = 1 */
        while (USART_GetFlagStatus(USART6, USART_FLAG_RXNE) == RESET)
        {}
        tmp[index++] = (USART_ReceiveData(USART6));
        if ((tmp[index - 1] < 0x30) || (tmp[index - 1] > 0x39))
        {
            printf("\n\r Please enter valid number between 0 and 9\n\r");
            index--;
        }
    }
    /* Calculate the Corresponding value */
    index = (tmp[1] - 0x30) + ((tmp[0] - 0x30) * 10);
    /* Checks the value */
    if ((index < MinValue) || (index > MaxValue))
    {
        printf("\n\r Please enter valid number between %d and %d\n\r", MinValue, MaxValue);
        return 0xFF;
    }
    return index;
}

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
    /* Place your implementation of fputc here */
    /* e.g. write a character to the USART */
    USART_SendData(USART6, (uint8_t) ch);

    /* Loop until the end of transmission */
    while (USART_GetFlagStatus(USART6, USART_FLAG_TC) == RESET)
    {}

    return ch;
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
        ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while (1)
    {
    }
}
#endif

void RTC_TimeShow(void)
{
    /* Get the current Time and Date */
    RTC_GetTime(RTC_Format_BIN, &RTC_TimeStruct);
    printf("\n\r============== Current Time Display ============================\n\r");
    vTaskDelay(100);
    printf("\n\r  The current time (Hour-Minute-Second) is :  %0.2d:%0.2d:%0.2d \n\r", RTC_TimeStruct.RTC_Hours, RTC_TimeStruct.RTC_Minutes, RTC_TimeStruct.RTC_Seconds);
    vTaskDelay(100);
    (void)RTC->DR;
}

/**
  * @brief  Display the current date on the Hyperterminal.
  * @param  None
  * @retval None
  */
void RTC_DateShow(void)
{
  /* Get the current Date */
    RTC_GetDate(RTC_Format_BIN, &RTC_DateStruct);
    printf("\n\r============== Current Date Display ============================\n\r");
    vTaskDelay(100);
    printf("\n\r  The current date (WeekDay-Date-Month-Year) is :  %0.2d-%0.2d-%0.2d-%0.2d \n\r", RTC_DateStruct.RTC_WeekDay, RTC_DateStruct.RTC_Date, RTC_DateStruct.RTC_Month, RTC_DateStruct.RTC_Year);
    vTaskDelay(100);
}


/**
  * @brief  Display the current TimeStamp (time and date) on the Hyperterminal.
  * @param  None
  * @retval None
  */
void RTC_TimeStampShow(void)
{
    /* Get the current TimeStamp */
    RTC_GetTimeStamp(RTC_Format_BIN, &RTC_TimeStampStruct, &RTC_TimeStampDateStruct);
    printf("\n\r==============TimeStamp Display (Time and Date)=================\n\r");
    vTaskDelay(100);
    printf("\n\r  The current timestamp time (Hour-Minute-Second) is :  %0.2d:%0.2d:%0.2d \n\r", RTC_TimeStampStruct.RTC_Hours, RTC_TimeStampStruct.RTC_Minutes, RTC_TimeStampStruct.RTC_Seconds);
    vTaskDelay(100);
    printf("\n\r  The current timestamp date (WeekDay-Date-Month) is :  %0.2d-%0.2d-%0.2d \n\r", RTC_TimeStampDateStruct.RTC_WeekDay, RTC_TimeStampDateStruct.RTC_Date, RTC_TimeStampDateStruct.RTC_Month);
    vTaskDelay(100);
}

/**
  * @brief  Configure the RTC peripheral by selecting the clock source.
  * @param  None
  * @retval None
  */
void RTC_Config(void)
{
    /* Enable the PWR clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

    /* Allow access to RTC */
    PWR_BackupAccessCmd(ENABLE);

#if defined (RTC_CLOCK_SOURCE_LSI)  /* LSI used as RTC source clock*/
/* The RTC Clock may varies due to LSI frequency dispersion. */
    /* Enable the LSI OSC */
    RCC_LSICmd(ENABLE);

    /* Wait till LSI is ready */
    while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
    {
    }

    /* Select the RTC Clock Source */
    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);

    SynchPrediv = 0xFF;
    AsynchPrediv = 0x7F;

#elif defined (RTC_CLOCK_SOURCE_LSE) /* LSE used as RTC source clock */
    /* Enable the LSE OSC */
    RCC_LSEConfig(RCC_LSE_ON);

    /* Wait till LSE is ready */
    while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
    {
    }

    /* Select the RTC Clock Source */
    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);

    SynchPrediv = 0xFF;
    AsynchPrediv = 0x7F;

#else
    #error Please select the RTC Clock source inside the main.c file
#endif /* RTC_CLOCK_SOURCE_LSI */

    /* Enable the RTC Clock */
    RCC_RTCCLKCmd(ENABLE);

    /* Wait for RTC APB registers synchronisation */
    RTC_WaitForSynchro();

    /* Enable The TimeStamp */
    RTC_TimeStampCmd(RTC_TimeStampEdge_Falling, ENABLE);
}

/**
  * @brief  Returns the time entered by user, using Hyperterminal.
  * @param  None
  * @retval None
  */
void RTC_TimeRegulate(void)
{
    uint32_t tmp_hh = 0xFF, tmp_mm = 0xFF, tmp_ss = 0xFF;

    printf("\n\r==============Time Settings=====================================\n\r");
    vTaskDelay(100);
    
    RTC_TimeStruct.RTC_H12     = RTC_H12_AM;
    printf("  Please Set Hours\n\r");
    vTaskDelay(100);
    
    while (tmp_hh == 0xFF)
    {
        tmp_hh = USART_Scanf(0, 23);
        RTC_TimeStruct.RTC_Hours = tmp_hh;
    }
    printf(":  %0.2d\n\r", tmp_hh);
    vTaskDelay(100);

    printf("  Please Set Minutes\n\r");
    vTaskDelay(100);
    
    while (tmp_mm == 0xFF)
    {
        tmp_mm = USART_Scanf(0, 59);
        RTC_TimeStruct.RTC_Minutes = tmp_mm;
    }
    printf(":  %0.2d\n\r", tmp_mm);
    vTaskDelay(100);

    printf("  Please Set Seconds\n\r");
    vTaskDelay(100);
    
    while (tmp_ss == 0xFF)
    {
        tmp_ss = USART_Scanf(0, 59);
        RTC_TimeStruct.RTC_Seconds = tmp_ss;
    }
    printf(":  %0.2d\n\r", tmp_ss);
    vTaskDelay(100);

    /* Configure the RTC time register */
    if(RTC_SetTime(RTC_Format_BIN, &RTC_TimeStruct) == ERROR)
    {
        printf("\n\r>> !! RTC Set Time failed. !! <<\n\r");
        vTaskDelay(100);
    }
    else
    {
        printf("\n\r>> !! RTC Set Time success. !! <<\n\r");
        vTaskDelay(100);
        
        RTC_TimeShow();
        /* Indicator for the RTC configuration */
        RTC_WriteBackupRegister(RTC_BKP_DR0, 0x32F2);
    }

    tmp_hh = 0xFF;
    tmp_mm = 0xFF;
    tmp_ss = 0xFF;

    printf("\n\r==============Date Settings=====================================\n\r");
    vTaskDelay(100);

    printf("  Please Set WeekDay (01-07)\n\r");
    vTaskDelay(100);
    
    while (tmp_hh == 0xFF)
    {
        tmp_hh = USART_Scanf(1, 7);
        RTC_DateStruct.RTC_WeekDay = tmp_hh;
    }
    
    printf(":  %0.2d\n\r", tmp_hh);
    vTaskDelay(100);
    
    tmp_hh = 0xFF;
    
    printf("  Please Set Date (01-31)\n\r");
    vTaskDelay(100);
    
    while (tmp_hh == 0xFF)
    {
        tmp_hh = USART_Scanf(1, 31);
        RTC_DateStruct.RTC_Date = tmp_hh;
    }
    
    printf(":  %0.2d\n\r", tmp_hh);
    vTaskDelay(100);

    printf("  Please Set Month (01-12)\n\r");
    vTaskDelay(100);
    
    while (tmp_mm == 0xFF)
    {
        tmp_mm = USART_Scanf(1, 12);
        RTC_DateStruct.RTC_Month = tmp_mm;
    }
    printf(":  %0.2d\n\r", tmp_mm);
    vTaskDelay(100);

    printf("  Please Set Year (00-99)\n\r");
    vTaskDelay(100);
    
    while (tmp_ss == 0xFF)
    {
        tmp_ss = USART_Scanf(0, 99);
        RTC_DateStruct.RTC_Year = tmp_ss;
    }
    printf(":  %0.2d\n\r", tmp_ss);
    vTaskDelay(100);

    /* Configure the RTC date register */
    if(RTC_SetDate(RTC_Format_BIN, &RTC_DateStruct) == ERROR)
    {
        printf("\n\r>> !! RTC Set Date failed. !! <<\n\r");
        vTaskDelay(100);
    }
    else
    {
        printf("\n\r>> !! RTC Set Date success. !! <<\n\r");
        vTaskDelay(100);
        RTC_DateShow();
        /* Indicator for the RTC configuration */
        RTC_WriteBackupRegister(RTC_BKP_DR0, 0x32F2);
    }

}

/**
  * @brief  Configures SDIO IRQ channel.
  * @param  None
  * @retval None
  */
void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Configure the NVIC Preemption Priority Bits */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

  NVIC_InitStructure.NVIC_IRQChannel = SDIO_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  NVIC_InitStructure.NVIC_IRQChannel = SD_SDIO_DMA_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_Init(&NVIC_InitStructure);  
}

/**
  * @brief  Tests the SD card erase operation.
  * @param  None
  * @retval None
  */
void SD_EraseTest(void)
{
  /*------------------- Block Erase ------------------------------------------*/
  if (Status == SD_OK)
  {
    //printf("SD_Erase...\n\r");
    //vTaskDelay(100);
    
    /* Erase NumberOfBlocks Blocks of WRITE_BL_LEN(512 Bytes) */
    Status = SD_Erase(0x00, (BLOCK_SIZE * NUMBER_OF_BLOCKS));
  }

  if (Status == SD_OK)
  {
    //printf("SD_ReadMultiBlocks...\n\r");
    //vTaskDelay(100);
    
    Status = SD_ReadMultiBlocks(Buffer_MultiBlock_Rx, 0x00, BLOCK_SIZE, NUMBER_OF_BLOCKS);

    //printf("SD_WaitReadOperation...\n\r");
    //vTaskDelay(100);
    
    /* Check if the Transfer is finished */
    Status = SD_WaitReadOperation();

    //printf("SD_GetStatus...\n\r");
    //vTaskDelay(100);
    /* Wait until end of DMA transfer */
    while(SD_GetStatus() != SD_TRANSFER_OK);
  }

  /* Check the correctness of erased blocks */
  if (Status == SD_OK)
  {
    //printf("eBuffercmp...\n\r");
    //vTaskDelay(100);
    
    EraseStatus = eBuffercmp(Buffer_MultiBlock_Rx, MULTI_BUFFER_SIZE);
  }
  
  if(EraseStatus == PASSED)
  {
    //STM_EVAL_LEDOn(LED1);
    printf("EraseStatus = PASSED...\n\r");
    vTaskDelay(100);
  }
  else
  {
    //STM_EVAL_LEDOff(LED1);
    //STM_EVAL_LEDOn(LED4);
    printf("EraseStatus = FAILED...\n\r");
    vTaskDelay(100);
  }
}

/**
  * @brief  Tests the SD card Single Blocks operations.
  * @param  None
  * @retval None
  */
void SD_SingleBlockTest(void)
{
  /*------------------- Block Read/Write --------------------------*/
  /* Fill the buffer to send */
  Fill_Buffer(Buffer_Block_Tx, BLOCK_SIZE, 0x320F);

  if (Status == SD_OK)
  {
    /* Write block of 512 bytes on address 0 */
    Status = SD_WriteBlock(Buffer_Block_Tx, 0x00, BLOCK_SIZE);
    /* Check if the Transfer is finished */
    Status = SD_WaitWriteOperation();
    while(SD_GetStatus() != SD_TRANSFER_OK);
  }

  if (Status == SD_OK)
  {
    /* Read block of 512 bytes from address 0 */
    Status = SD_ReadBlock(Buffer_Block_Rx, 0x00, BLOCK_SIZE);
    /* Check if the Transfer is finished */
    Status = SD_WaitReadOperation();
    while(SD_GetStatus() != SD_TRANSFER_OK);
  }

  /* Check the correctness of written data */
  if (Status == SD_OK)
  {
    TransferStatus1 = Buffercmp(Buffer_Block_Tx, Buffer_Block_Rx, BLOCK_SIZE);
  }
  
  if(TransferStatus1 == PASSED)
  {
    //STM_EVAL_LEDOn(LED2);
    printf("TransferStatus1 = PASSED...\n\r");
    vTaskDelay(100);
  }
  else
  {
    //STM_EVAL_LEDOff(LED2);
    //STM_EVAL_LEDOn(LED4);
    printf("TransferStatus1 = FAILED...\n\r");
    vTaskDelay(100);
  }
}

/**
  * @brief  Tests the SD card Multiple Blocks operations.
  * @param  None
  * @retval None
  */
void SD_MultiBlockTest(void)
{
  /*--------------- Multiple Block Read/Write ---------------------*/
  /* Fill the buffer to send */
  Fill_Buffer(Buffer_MultiBlock_Tx, MULTI_BUFFER_SIZE, 0x0);

  if (Status == SD_OK)
  {
    /* Write multiple block of many bytes on address 0 */
    Status = SD_WriteMultiBlocks(Buffer_MultiBlock_Tx, 0x00, BLOCK_SIZE, NUMBER_OF_BLOCKS);
    /* Check if the Transfer is finished */
    Status = SD_WaitWriteOperation();
    while(SD_GetStatus() != SD_TRANSFER_OK);
  }

  if (Status == SD_OK)
  {
    /* Read block of many bytes from address 0 */
    Status = SD_ReadMultiBlocks(Buffer_MultiBlock_Rx, 0x00, BLOCK_SIZE, NUMBER_OF_BLOCKS);
    /* Check if the Transfer is finished */
    Status = SD_WaitReadOperation();
    while(SD_GetStatus() != SD_TRANSFER_OK);
  }

  /* Check the correctness of written data */
  if (Status == SD_OK)
  {
    TransferStatus2 = Buffercmp(Buffer_MultiBlock_Tx, Buffer_MultiBlock_Rx, MULTI_BUFFER_SIZE);
  }
  
  if(TransferStatus2 == PASSED)
  {
    //STM_EVAL_LEDOn(LED3);
    printf("TransferStatus2 = PASSED...\n\r");
    vTaskDelay(100);
  }
  else
  {
    //STM_EVAL_LEDOff(LED3);
    //STM_EVAL_LEDOn(LED4);
    printf("TransferStatus2 = FAILED...\n\r");
    vTaskDelay(100);
  }
}

/**
  * @brief  Compares two buffers.
  * @param  pBuffer1, pBuffer2: buffers to be compared.
  * @param  BufferLength: buffer's length
  * @retval PASSED: pBuffer1 identical to pBuffer2
  *         FAILED: pBuffer1 differs from pBuffer2
  */
TestStatus Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint32_t BufferLength)
{
  while (BufferLength--)
  {
    if (*pBuffer1 != *pBuffer2)
    {
      return FAILED;
    }

    pBuffer1++;
    pBuffer2++;
  }

  return PASSED;
}

/**
  * @brief  Fills buffer with user predefined data.
  * @param  pBuffer: pointer on the Buffer to fill
  * @param  BufferLength: size of the buffer to fill
  * @param  Offset: first value to fill on the Buffer
  * @retval None
  */
void Fill_Buffer(uint8_t *pBuffer, uint32_t BufferLength, uint32_t Offset)
{
  uint16_t index = 0;

  /* Put in global buffer same values */
  for (index = 0; index < BufferLength; index++)
  {
    pBuffer[index] = index + Offset;
  }
}

/**
  * @brief  Checks if a buffer has all its values are equal to zero.
  * @param  pBuffer: buffer to be compared.
  * @param  BufferLength: buffer's length
  * @retval PASSED: pBuffer values are zero
  *         FAILED: At least one value from pBuffer buffer is different from zero.
  */
TestStatus eBuffercmp(uint8_t* pBuffer, uint32_t BufferLength)
{
  while (BufferLength--)
  {
    /* In some SD Cards the erased state is 0xFF, in others it's 0x00 */
    if ((*pBuffer != 0xFF) && (*pBuffer != 0x00))
    {
      return FAILED;
    }

    pBuffer++;
  }

  return PASSED;
}

