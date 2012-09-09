/* Standard includes. */
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* ST StdPeriph Driver includes. */
#include "stm32f4xx_conf.h"

#include "drv_mpu6050.h"
#include "drv_hmc5883l.h"

unsigned int count = 0;
unsigned int p1=2,p2=10,p3=10000;
unsigned int data[6] = {0};
u8 data_mpu[118] = {0};
u8 data_hmc[13] = {0};
unsigned int data_g[6] = {0};

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


void vLoopTask( void *pvParameters );
void vDiagTask( void *pvParameters );
uint8_t I2C_ByteRead(u16 Addr, u8 Reg);
void I2C_ByteWrite(u16 Addr, u8 Reg,u8 Data,u8 Cmd);
void I2C_MutiRead(u8* pBuffer, u8 Addr, u8 Reg,u8 Count);
void I2C2_Configuration(void);
unsigned char crc4(unsigned int n_prom[]) ;

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

    while(1)
    {
        count++;
        vTaskDelay(delay);
    }
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


#define TRUE   (1)                      /* Boolean true value.   */
#define FALSE  (0)                      /* Boolean false value.  */


void vDiagTask( void *pvParameters )
{
    GPIO_InitTypeDef  GPIO_PB;
    I2C_InitTypeDef   I2C_CH2;
    unsigned int delay=0, i=0;

    if( pvParameters != NULL )
        delay = *((unsigned int*)pvParameters);
    else
        delay = 2;

    //------------------------------------------------
    // BEEP & GPIO Configuration
    //------------------------------------------------
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_PB.GPIO_Pin = GPIO_Pin_12;
    GPIO_PB.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_PB.GPIO_OType = GPIO_OType_PP;
    GPIO_PB.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_PB.GPIO_PuPd = GPIO_PuPd_NOPULL;

    GPIO_Init(GPIOB, &GPIO_PB);
    //------------------------------------------------

    //------------------------------------------------
    // I2C & GPIO Configuration
    //------------------------------------------------
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

    GPIO_PB.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
    GPIO_PB.GPIO_Mode = GPIO_Mode_AF;
    GPIO_PB.GPIO_OType = GPIO_OType_OD;
    GPIO_PB.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_PB.GPIO_PuPd = GPIO_PuPd_NOPULL;

    GPIO_Init(GPIOB, &GPIO_PB);

    /*!< I2C configuration */
    /* I2C configuration */
    I2C_CH2.I2C_Mode = I2C_Mode_I2C;
    I2C_CH2.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_CH2.I2C_OwnAddress1 = 0;
    I2C_CH2.I2C_Ack = I2C_Ack_Enable;
    I2C_CH2.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_CH2.I2C_ClockSpeed = I2C_SPEED;

    /* I2C Peripheral Enable */
    I2C_Cmd(I2C2, ENABLE);
    /* Apply I2C configuration after enabling it */
    I2C_Init(I2C2, &I2C_CH2);
    //------------------------------------------------

    //------------------------------------------------
    // Test PCA9536
    //------------------------------------------------
    I2C_ByteWrite(PCA9536DP_ADDRESS, 0x03, 0xF7, NULL); // IO3 us output pin
    //------------------------------------------------

    //------------------------------------------------
    // Test PCA9533
    //------------------------------------------------
    I2C_ByteWrite(PCA9533DP_ADDRESS, 0x01, 0xFF, FALSE); // PSC0 : P = 1.68
    I2C_ByteWrite(PCA9533DP_ADDRESS, 0x02, 0x40, FALSE); // PWM0 50%(0x80)
    I2C_ByteWrite(PCA9533DP_ADDRESS, 0x03, 0x97, FALSE); // PSC1 : P = 1.00
    I2C_ByteWrite(PCA9533DP_ADDRESS, 0x04, 0x40, FALSE); // PWM1 25%(0x40)
    I2C_ByteWrite(PCA9533DP_ADDRESS, 0x05, 0xBB, FALSE); // LS0
    //------------------------------------------------

    //------------------------------------------------
    // Test MPU6050
    //------------------------------------------------
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
    for(i=0 ; i<13 ; i++)
    {
        data_hmc[i]=I2C_ByteRead(HMC5883L_ADDRESS, i); // to check HMC5883L
    }
    //------------------------------------------------

    //------------------------------------------------
    // LOOP
    //------------------------------------------------
    while(1)
    {
        #if 0
        GPIO_SetBits(GPIOB, GPIO_Pin_12);
        delay = 20 / portTICK_RATE_MS;
        vTaskDelay(delay);
        GPIO_ResetBits(GPIOB, GPIO_Pin_12);
        #endif

        delay = 1000 / portTICK_RATE_MS;

        I2C_ByteWrite(PCA9536DP_ADDRESS, 0x01, 0xF7, FALSE); // IO3 (ON)
        vTaskDelay(delay);
        I2C_ByteWrite(PCA9536DP_ADDRESS, 0x01, 0xFF, FALSE); // IO3 (OFF)
        vTaskDelay(delay);
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
