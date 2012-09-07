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

void vLoopTask( void *pvParameters );
void vDiagTask( void *pvParameters );
uint8_t I2C_ByteRead(u16 I2C_SLAVE_ADDRESS, u8 RegAddr);
void I2C_ByteWrite(u16 I2C_SLAVE_ADDRESS, u8 RegAddr,u8 CData);

void I2C2_Configuration(void);



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
#define MS5611_ADDRESS    0x76
#define I2C_SPEED 400000

void vDiagTask( void *pvParameters )
{
    GPIO_InitTypeDef  GPIO_PB;
    I2C_InitTypeDef   I2C_CH2;
    unsigned int delay = 0;
    
    //const portTickType xDelay = 5000 / portTICK_RATE_MS;

    if( pvParameters != NULL )
        delay = *((unsigned int*)pvParameters);
    else
        delay = 2;

    // BEEP
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_PB.GPIO_Pin = GPIO_Pin_12;
    GPIO_PB.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_PB.GPIO_OType = GPIO_OType_PP;
    GPIO_PB.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_PB.GPIO_PuPd = GPIO_PuPd_NOPULL;
    
    GPIO_Init(GPIOB, &GPIO_PB);

    // I2C
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

    I2C_ByteWrite(PCA9536DP_ADDRESS, 0x03, 0xF7); // IO3 us output pin

    I2C_ByteWrite(PCA9533DP_ADDRESS, 0x01, 0xFF); // PSC0 : P = 1.68
    I2C_ByteWrite(PCA9533DP_ADDRESS, 0x02, 0x80); // PWM0
    I2C_ByteWrite(PCA9533DP_ADDRESS, 0x03, 0x97); // PSC1 : P = 1.00
    I2C_ByteWrite(PCA9533DP_ADDRESS, 0x04, 0x80); // PWM1
    I2C_ByteWrite(PCA9533DP_ADDRESS, 0x05, 0xBB); // LS0
    

    I2C_ByteWrite(MPU6050_ADDRESS, 0x37, 0x02);
    
    data[0]=I2C_ByteRead(MPU6050_ADDRESS, 0x75); // to check MPU6050
    //data[1]=I2C_ByteRead(MPU6050_ADDRESS, 0x24); // to check MPU6050
    data[1]=I2C_ByteRead(MPU6050_ADDRESS, 0x6A); // to check MPU6050
    data[2]=I2C_ByteRead(MPU6050_ADDRESS, 0x37); // to check MPU6050
    //data[3]=I2C_ByteRead(HMC5883L_ADDRESS, 0x0A); // to check HMC5883L
    //data[4]=I2C_ByteRead(MS5611_ADDRESS, 0x0); // to check MS5611
    while(1)
    {
       #if 1
        GPIO_SetBits(GPIOB, GPIO_Pin_12);
        delay = 20 / portTICK_RATE_MS;
        vTaskDelay(delay);
        GPIO_ResetBits(GPIOB, GPIO_Pin_12);
       #endif

        delay = 1000 / portTICK_RATE_MS;

        I2C_ByteWrite(PCA9536DP_ADDRESS, 0x01, 0xF7); // IO3 (ON)
        vTaskDelay(delay);
        I2C_ByteWrite(PCA9536DP_ADDRESS, 0x01, 0xFF); // IO3 (OFF)
        vTaskDelay(delay);

        //data=I2C_ByteRead(PCA9536DP_ADDRESS, 0x00);
        //data=I2C_ByteRead(MPU6050_ADDRESS, 0x75);
    }
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
uint8_t I2C_ByteRead(u16 I2C_SLAVE_ADDRESS, u8 RegAddr)
{
    u16 tempADD; 
    uint8_t rData;
    //I2C2_Configuration();
    while(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY));            //檢測總線是否忙 就是看 SCL 或SDA是否為 低

    tempADD=I2C_SLAVE_ADDRESS<<1;
    I2C_AcknowledgeConfig(I2C2, ENABLE);                     //允許1字節1應答模式

    I2C_GenerateSTART(I2C2, ENABLE);                        // 發送起始位 
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT)); //EV5,主模式 SR1.SB應當=1
//while((((u32)(I2C1->SR2) << 16) | (u32)(I2C1->SR1) & 0x00FFFFBF) != I2C_EVENT_MASTER_MODE_SELECT); 

    I2C_Send7bitAddress(I2C2, tempADD, I2C_Direction_Transmitter);   //發送移位後的器件地址(寫)
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); // Test on EV6 and clear it 仿真過不去，直接運行沒問題

//  I2C_Cmd(I2C1, ENABLE);                              /* Clear EV6 by setting again the PE bit */--------驗證是否需要
 
    I2C_SendData(I2C2, RegAddr);                                      /*發送P寄存器地址*/
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED)); /*數據已發送*/ /* Test on EV8 and clear it */


   I2C_GenerateSTART(I2C2, ENABLE);                                     //起始位
   while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));  /* Test on EV5 and clear it */


   I2C_Send7bitAddress(I2C2, tempADD, I2C_Direction_Receiver);                /*發送器件地址(讀)*/
   while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)); /* Test on EV6 and clear it */

   while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED)); /* EV7 */
   rData= I2C_ReceiveData(I2C2);                                 /* 讀 Register*/
    I2C_AcknowledgeConfig(I2C2, DISABLE); //最後一位後要關閉應答的
    I2C_GenerateSTOP(I2C2, ENABLE);   //發送停止位 

             
 return rData;
}

void I2C2_Configuration(void)
{
  I2C_InitTypeDef  I2C_InitStructure; 
  I2C_DeInit(I2C2); 
  I2C_Cmd(I2C2, DISABLE); 

  /* I2C configuration */
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;                   //設置為i2c模式
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;           //I2C 快速模式 Tlow / Thigh = 2 
  I2C_InitStructure.I2C_OwnAddress1 = 0;     //這個地方不太明白什麼作用，自身地址？是不是只有從模式才有效？
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;                  //使能 應答 功能
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;   //應答7位地址
  I2C_InitStructure.I2C_ClockSpeed = I2C_SPEED;                               //設置i2c的速率，不能高於 400KHz
  

  I2C_Cmd(I2C2, ENABLE);                                       //使能i2c1外設
  I2C_Init(I2C2, &I2C_InitStructure);                          //配置i2c1

  I2C_AcknowledgeConfig(I2C2, ENABLE);                        //允許1字節1應答模式
    
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
void I2C_ByteWrite(u16 I2C_SLAVE_ADDRESS, u8 RegAddr,u8 CData)
{
   u16 tempADD;
   //I2C2_Configuration();
//*((u8 *)0x4001080c) |=0x80; 
    while(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY)); // 檢測i2c總線狀態
    tempADD=I2C_SLAVE_ADDRESS<<1;    
   
  I2C_AcknowledgeConfig(I2C2, ENABLE);           //允許1字節1應答模式
  
  I2C_GenerateSTART(I2C2, ENABLE);                //發送start信號
  
  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));  //EV5：SB=1，讀SR1然後將地址寫入DR寄存器將清除該事件 
  I2C_Send7bitAddress(I2C2, tempADD, I2C_Direction_Transmitter);   //發送設備地址，主發送

  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));  //EV6：ADDR=1，讀SR1然後讀SR2將清除該事件
  
  /* Clear EV6 by setting again the PE bit */
  I2C_Cmd(I2C2, ENABLE);                                                   //驗證是否需要此步？************

  /* Send the EEPROM's internal address to write to */
  I2C_SendData(I2C2, RegAddr);                                             //發送寄存器地址

  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));         //EV8：TxE=1，寫入DR寄存器將清除該事件

  /* Send the byte to be written */
  I2C_SendData(I2C2, CData);                                               //發送配置字
   
  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  
  /* Send STOP condition */
  I2C_GenerateSTOP(I2C2, ENABLE);                                          //發送停止信號
  
}

