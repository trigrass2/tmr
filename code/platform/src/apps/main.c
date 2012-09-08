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
unsigned int data[6] = {0};
unsigned int data_m[13] = {0};

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


void vLoopTask( void *pvParameters );
void vDiagTask( void *pvParameters );
uint8_t I2C_ByteRead(u16 Add, u8 Reg);
void I2C_ByteWrite(u16 Add, u8 Reg,u8 Data,u8 Cmd);
void I2C_MutiRead(u8* pBuffer, u8 Add, u8 Reg,u8 Count);
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

#define PCA9536DP_ADDRESS 0x41
#define PCA9533DP_ADDRESS 0x62
#define MPU6050_ADDRESS   0x69
#define HMC5883L_ADDRESS  0x1E
#define MS5611_ADDRESS    0x77
#define I2C_SPEED 400000

// MS5611
#define CMD_RESET    0x1E // ADC reset command 
#define CMD_ADC_READ 0x00 // ADC read command 
#define CMD_ADC_CONV 0x40 // ADC conversion command 
#define CMD_ADC_D1   0x00 // ADC D1 conversion 
#define CMD_ADC_D2   0x10 // ADC D2 conversion 
#define CMD_ADC_256  0x00 // ADC OSR=256 
#define CMD_ADC_512  0x02 // ADC OSR=512 
#define CMD_ADC_1024 0x04 // ADC OSR=1024 
#define CMD_ADC_2048 0x06 // ADC OSR=2056 
#define CMD_ADC_4096 0x08 // ADC OSR=4096 
#define CMD_PROM_RD  0xA0 // Prom read command


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
    I2C_ByteWrite(PCA9533DP_ADDRESS, 0x01, 0xFF, NULL); // PSC0 : P = 1.68
    I2C_ByteWrite(PCA9533DP_ADDRESS, 0x02, 0x40, NULL); // PWM0 50%(0x80)
    I2C_ByteWrite(PCA9533DP_ADDRESS, 0x03, 0x97, NULL); // PSC1 : P = 1.00
    I2C_ByteWrite(PCA9533DP_ADDRESS, 0x04, 0x40, NULL); // PWM1 25%(0x40)
    I2C_ByteWrite(PCA9533DP_ADDRESS, 0x05, 0xBB, NULL); // LS0
    //------------------------------------------------

    //------------------------------------------------
    // Test MPU6050
    //------------------------------------------------
    I2C_ByteWrite(MPU6050_ADDRESS, 0x37, 0x02, NULL);
    
    data[0]=I2C_ByteRead(MPU6050_ADDRESS, 0x75); // to check MPU6050
    data[1]=I2C_ByteRead(MPU6050_ADDRESS, 0x24); // to check MPU6050
    data[2]=I2C_ByteRead(MPU6050_ADDRESS, 0x6A); // to check MPU6050
    data[3]=I2C_ByteRead(MPU6050_ADDRESS, 0x37); // to check MPU6050
    //------------------------------------------------

    //------------------------------------------------
    // Test MS5611
    //------------------------------------------------
    I2C_ByteWrite(MS5611_ADDRESS, CMD_RESET, NULL, TRUE); // Reset command for MS5611
    
    delay = 20 / portTICK_RATE_MS; // delay 20 ms
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
    //Read out all register data from HMC5883L
    for(i=0 ; i<13 ; i++)
    {
        data_m[i]=I2C_ByteRead(HMC5883L_ADDRESS, i); // to check HMC5883L
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

        I2C_ByteWrite(PCA9536DP_ADDRESS, 0x01, 0xF7, NULL); // IO3 (ON)
        vTaskDelay(delay);
        I2C_ByteWrite(PCA9536DP_ADDRESS, 0x01, 0xFF, NULL); // IO3 (OFF)
        vTaskDelay(delay);
    }
    //------------------------------------------------
}

/*******************************************************************************
* Function Name  : SKATER_I2C_ByteRead
* Description    : 單字節讀取指定地址的數據（7位地址）.
* Input          : - I2C_SLAVE_ADDRESS : 設備地址。
*                  - RegAddr:寄存器地址.
*                  - Data：讀取的單字節數據
* Output         : None
* Return         : rData：讀取的數據
*******************************************************************************/
uint8_t I2C_ByteRead(u16 Add, u8 Reg)
{
    u16 tempADD; 
    uint8_t rData;
    //I2C2_Configuration();
    while(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY));    //檢測總線是否忙 就是看 SCL 或SDA是否為 低

    tempADD=Add<<1;
    I2C_AcknowledgeConfig(I2C2, ENABLE);    //允許1字節1應答模式

    I2C_GenerateSTART(I2C2, ENABLE);    // 發送起始位 
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));    //EV5,主模式 SR1.SB應當=1
    //while((((u32)(I2C1->SR2) << 16) | (u32)(I2C1->SR1) & 0x00FFFFBF) != I2C_EVENT_MASTER_MODE_SELECT); 

    I2C_Send7bitAddress(I2C2, tempADD, I2C_Direction_Transmitter);    //發送移位後的器件地址(寫)
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));    // Test on EV6 and clear it 仿真過不去，直接運行沒問題

    //  I2C_Cmd(I2C1, ENABLE);    /* Clear EV6 by setting again the PE bit */--------驗證是否需要

    I2C_SendData(I2C2, Reg);    /*發送P寄存器地址*/
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));    /*數據已發送*/ /* Test on EV8 and clear it */

    I2C_GenerateSTART(I2C2, ENABLE);    //起始位
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));    /* Test on EV5 and clear it */

    I2C_Send7bitAddress(I2C2, tempADD, I2C_Direction_Receiver);    /*發送器件地址(讀)*/
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));    /* Test on EV6 and clear it */

    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED));    /* EV7 */
    rData= I2C_ReceiveData(I2C2);    /* 讀 Register*/
    I2C_AcknowledgeConfig(I2C2, DISABLE);    //最後一位後要關閉應答的
    I2C_GenerateSTOP(I2C2, ENABLE);    //發送停止位 

    return rData;
}

void I2C2_Configuration(void)
{
    I2C_InitTypeDef  I2C_InitStructure; 
    I2C_DeInit(I2C2); 
    I2C_Cmd(I2C2, DISABLE); 

    /* I2C configuration */
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;    //設置為i2c模式
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;    //I2C 快速模式 Tlow / Thigh = 2 
    I2C_InitStructure.I2C_OwnAddress1 = 0;    //這個地方不太明白什麼作用，自身地址？是不是只有從模式才有效？
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;    //使能 應答 功能
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;    //應答7位地址
    I2C_InitStructure.I2C_ClockSpeed = I2C_SPEED;    //設置i2c的速率，不能高於 400KHz

    I2C_Cmd(I2C2, ENABLE);    //使能i2c1外設
    I2C_Init(I2C2, &I2C_InitStructure);    //配置i2c1

    I2C_AcknowledgeConfig(I2C2, ENABLE);    //允許1字節1應答模式

    //  printf("\n\r I2C1_初始化完成\n\r");
}

/*******************************************************************************
* Function Name  : SKATER_I2C_ByteWrite
* Description    : 單字節寫入指定地址的數據指定數據（7位地址）--一般為控制字或配置.
* Input          : - I2C_SLAVE_ADDRESS : 設備地址。
*                  - RegAddr:寄存器地址.
*                  - CData：數據
* Output         : None
* Return         : None
*******************************************************************************/
void I2C_ByteWrite(u16 Add, u8 Reg,u8 Data,u8 Cmd)
{
    u16 tempADD;
    //I2C2_Configuration();
    //*((u8 *)0x4001080c) |=0x80; 
    while(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY));    // 檢測i2c總線狀態
    tempADD=Add<<1;    

    I2C_AcknowledgeConfig(I2C2, ENABLE);    //允許1字節1應答模式

    I2C_GenerateSTART(I2C2, ENABLE);    //發送start信號

    /* Test on EV5 and clear it */
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));    //EV5：SB=1，讀SR1然後將地址寫入DR寄存器將清除該事件 
    I2C_Send7bitAddress(I2C2, tempADD, I2C_Direction_Transmitter);    //發送設備地址，主發送

    /* Test on EV6 and clear it */
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));    //EV6：ADDR=1，讀SR1然後讀SR2將清除該事件

    /* Clear EV6 by setting again the PE bit */
    I2C_Cmd(I2C2, ENABLE);    //驗證是否需要此步？************

    /* Send the EEPROM's internal address to write to */
    I2C_SendData(I2C2, Reg);    //發送寄存器地址

    /* Test on EV8 and clear it */
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));    //EV8：TxE=1，寫入DR寄存器將清除該事件

    if(Cmd == NULL)
    {
        /* Send the byte to be written */
        I2C_SendData(I2C2, Data);    //發送配置字

        /* Test on EV8 and clear it */
        while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    }
  
    /* Send STOP condition */
    I2C_GenerateSTOP(I2C2, ENABLE);    //發送停止信號
}

/*******************************************************************************
* Function Name  : SKATER_I2C_MultRead
* Description    : 多字節寫讀取指定地址的數據（7位地址）
* Input          : - pBuffer：數據存儲區
*                  - I2C_SLAVE_ADDRESS : 設備地址
*                  - RegAddr:起始寄存器地址
*                  - NumByteToRead：連續讀取的字節數目
* Output         : None
* Return         : None
*******************************************************************************/
void I2C_MutiRead(u8* pBuffer, u8 Add, u8 Reg,u8 Count)
{
    u8 tempADD;
    //I2C1_Configuration();
    //*((u8 *)0x4001080c) |=0x80; 
    while(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY));    // Added by Najoua 27/08/2008
    I2C_AcknowledgeConfig(I2C2, ENABLE);    //允許1字節1應答模式    

    /* Send START condition */
    I2C_GenerateSTART(I2C2, ENABLE);
    //*((u8 *)0x4001080c) &=~0x80;
    tempADD=Add<<1;

    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));    /* Test on EV5 and clear it */
    I2C_Send7bitAddress(I2C2, tempADD, I2C_Direction_Transmitter);    /* 發送移位後的設備地址，設置主發送模式 */

    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));    /* Test on EV6 and clear it */
    //  I2C_Cmd(I2C1, ENABLE); /* Clear EV6 by setting again the PE bit */

    I2C_SendData(I2C2, Reg);    /* 發送要讀取的寄存器地址 */
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));    /* Test on EV8 and clear it */

    I2C_GenerateSTART(I2C2, ENABLE);    /* 發送 開始 信號 */  
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));    /* Test on EV5 and clear it */

    I2C_Send7bitAddress(I2C2, tempADD, I2C_Direction_Receiver);    /* 發送移位後的設備地址 */
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

        /* 為了在收到最後一個字節後產生一個NACK脈衝，在讀倒數第二個數據字節之後(在倒數第二個RxNE事件之後)必須清除ACK位。
        為了產生一個停止/重起始條件，軟件必須在讀倒數第二個數據字節之後(在倒數第二個RxNE事件之後)設置STOP/START位。
        只接收一個字節時，剛好在EV6之後(EV6_1時，清除ADDR之後)要關閉應答和停止條件的產生位。*/

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
