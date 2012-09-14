/**
  ******************************************************************************
  * @file    STM32_EVAL_CPAL/Common/drv_i2c_cpal.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    17-June-2011
  * @brief   This file provides a set of functions needed to manage the I2C LM75 
  *          temperature sensor mounted on STM32xx-EVAL board (refer to stm32_eval.h
  *          to know about the boards supporting this sensor). 
  *          It implements a high level communication layer for read and write 
  *          from/to this sensor. The needed STM32 hardware resources (I2C and 
  *          GPIO) are defined in stm32xx_eval.h file, and the initialization is 
  *          performed in I2C2_LowLevel_Init() function declared in stm32xx_eval.c 
  *          file.
  *            
  *          @note This file is a modified version of stm32_eval_i2c_tsensor.c driver;
  *                I2C CPAL library drivers are used instead of the Standard Peripherals
  *                I2C driver.
  *                 
  *          You can easily tailor this driver to any other development board, 
  *          by just adapting the defines for hardware resources and 
  *          I2C2_LowLevel_Init() function.
  *
  *     +-----------------------------------------------------------------+
  *     |                        Pin assignment                           |                 
  *     +---------------------------------------+-----------+-------------+
  *     |  STM32 I2C Pins                       |   STLM75  |   Pin       |
  *     +---------------------------------------+-----------+-------------+
  *     | I2C2_I2C_SDA_PIN/ SDA                 |   SDA     |    1        |
  *     | I2C2_I2C_SCL_PIN/ SCL                 |   SCL     |    2        |
  *     |                                       |   OS/INT  |    3        |
  *     | .                                     |   GND     |    4  (0V)  |
  *     | .                                     |   GND     |    5  (0V)  |
  *     | .                                     |   GND     |    6  (0V)  |
  *     | .                                     |   GND     |    7  (0V)  |
  *     | .                                     |   VDD     |    8  (3.3V)|
  *     +---------------------------------------+-----------+-------------+
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "drv_i2c_cpal.h"

/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
#define I2C2_SD_SET      0x01 /*!< Set SD bit in the configuration register */
#define I2C2_SD_RESET    0xFE /*!< Reset SD bit in the configuration register */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
  
CPAL_TransferTypeDef  I2C2_RXTransfer = { 
                        /* Initialize TX Transfer structure */
                        pNULL,
                        0,
                        0,
                        0};

CPAL_TransferTypeDef  I2C2_TXTransfer = {
                        /* Initialize RX Transfer structure */
                        pNULL,
                        0,
                        0,
                        0};

uint8_t I2C2_Buffer[2] = {0x00,0x00}; 

extern CPAL_InitTypeDef I2C2_DevStructure;


__IO uint32_t  I2C2_Timeout = I2C2_TIMEOUT; 

/* Private function prototypes -----------------------------------------------*/
static void I2C2_StructInit(void);
static void I2C2_Init(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Configure the I2C2_I2C.
  * @param  None
  * @retval None
  */
void I2C2_Config(void)
{
  I2C2_StructInit ();
  I2C2_Init();
}

/**
  * @brief  Deinitialize the I2C2_I2C.
  * @param  None
  * @retval None
  */
void I2C2_DeInit(void)
{
    /* Initialize CPAL peripheral */
  CPAL_I2C_DeInit(&I2C2_DevStructure);
}

/**
  * @brief  Initializes the I2C2_I2C.
  * @param  None
  * @retval None
  */
static void I2C2_Init(void)
{
  /* Initialize CPAL peripheral */
  CPAL_I2C_Init(&I2C2_DevStructure);
}

/**
  * @brief  Initializes the I2C2_I2C.
  * @param  None
  * @retval None
  */
static void I2C2_StructInit(void)
{
  /* Set CPAL structure parameters to their default values */  
  CPAL_I2C_StructInit(&I2C2_DevStructure);
    
  /* Set I2C clock speed */
  I2C2_DevStructure.pCPAL_I2C_Struct->I2C_ClockSpeed = I2C2_SPEED;
  			 
#ifdef I2C2_IT
  /* Select Interrupt programming model and disable all options */
  I2C2_DevStructure.CPAL_ProgModel = CPAL_PROGMODEL_INTERRUPT;
  I2C2_DevStructure.wCPAL_Options  = 0;
#else
  /* Select DMA programming model and activate TX_DMA_TC and RX_DMA_TC interrupts */
  I2C2_DevStructure.CPAL_ProgModel = CPAL_PROGMODEL_DMA;
  I2C2_DevStructure.wCPAL_Options  = CPAL_OPT_DMATX_TCIT | CPAL_OPT_DMARX_TCIT ;
#endif /* I2C2_IT */  

  /* point to CPAL_TransferTypeDef structure */
  I2C2_DevStructure.pCPAL_TransferTx = &I2C2_TXTransfer;
  I2C2_DevStructure.pCPAL_TransferRx = &I2C2_RXTransfer;
}

static uint32_t I2C2_Status (void)
{
  I2C2_DevStructure.pCPAL_TransferTx = &I2C2_TXTransfer;     
  I2C2_DevStructure.pCPAL_TransferTx->wAddr1 = (uint32_t)I2C2_ADDR;
  
  return CPAL_I2C_IsDeviceReady(&I2C2_DevStructure);
}

/**
  * @brief  Checks the LM75 status.
  * @param  None
  * @retval ErrorStatus: LM75 Status (ERROR or SUCCESS).
  */
ErrorStatus I2C2_GetStatus(void)
{  
  /* Test if LM75 is ready */
  while ((I2C2_Status() == CPAL_FAIL) && I2C2_Timeout)  
  {
    I2C2_Timeout--;
  }
  
  /* If LM75 is not responding return ERROR */
  if (I2C2_Timeout == 0)
  {
    return ERROR;
  }
  
  /* In other case return SUCCESS */
  return SUCCESS;  
}
/**
  * @brief  Read the specified register from the LM75.
  * @param  RegName: specifies the LM75 register to be read.
  *              This member can be one of the following values:  
  *                  - I2C2_REG_TEMP: temperature register
  *                  - I2C2_REG_TOS: Over-limit temperature register
  *                  - I2C2_REG_THYS: Hysteresis temperature register
  * @retval LM75 register value.
  */
uint16_t I2C2_ReadReg(uint8_t RegName)
{   
  uint16_t tmp = 0;
  
  I2C2_Buffer[0] = 0;
  I2C2_Buffer[1] = 0;
  
  /* Disable all options */
  I2C2_DevStructure.wCPAL_Options = 0;

  /* point to CPAL_TransferTypeDef structure */
  I2C2_DevStructure.pCPAL_TransferRx = &I2C2_RXTransfer;
  
  /* Configure transfer parameters */  
  I2C2_DevStructure.pCPAL_TransferRx->wNumData = 2;
  I2C2_DevStructure.pCPAL_TransferRx->pbBuffer = I2C2_Buffer ;
  #if 0
  I2C2_DevStructure.pCPAL_TransferRx->wAddr1   = (uint32_t)I2C2_ADDR;
  #else
  I2C2_DevStructure.pCPAL_TransferRx->wAddr1   = (uint32_t)(0x69 << 1);
  #endif
  I2C2_DevStructure.pCPAL_TransferRx->wAddr2   = (uint32_t)RegName;
  
  /* Read Operation */
  if(CPAL_I2C_Read(&I2C2_DevStructure) == CPAL_PASS)
  {
    while ((I2C2_DevStructure.CPAL_State != CPAL_STATE_READY) && (I2C2_DevStructure.CPAL_State != CPAL_STATE_ERROR) )
    { }
  }
  
  /* Store I2C2_I2C received data */
  tmp = (uint16_t)(I2C2_Buffer[0] << 8);
  tmp |= I2C2_Buffer[1];
  
  /* return a Reg value */
  return (uint16_t)tmp;  
}

/**
  * @brief  Write to the specified register of the LM75.
  * @param  RegName: specifies the LM75 register to be written.
  *              This member can be one of the following values:    
  *                  - I2C2_REG_TOS: Over-limit temperature register
  *                  - I2C2_REG_THYS: Hysteresis temperature register
  * @param  RegValue: value to be written to LM75 register.  
  * @retval None
  */
uint8_t I2C2_WriteReg(uint8_t RegName, uint16_t RegValue)
{   
  I2C2_Buffer[0] = (uint8_t)(RegValue >> 8);
  I2C2_Buffer[1] = (uint8_t)(RegValue);
     
  /* Disable all options */
  I2C2_DevStructure.wCPAL_Options = 0;
  
  /* point to CPAL_TransferTypeDef structure */
  I2C2_DevStructure.pCPAL_TransferTx = &I2C2_TXTransfer;
  
  /* Configure transfer parameters */  
  I2C2_DevStructure.pCPAL_TransferTx->wNumData = 2;
  I2C2_DevStructure.pCPAL_TransferTx->pbBuffer = I2C2_Buffer ;
  I2C2_DevStructure.pCPAL_TransferTx->wAddr1   = (uint32_t)I2C2_ADDR;
  I2C2_DevStructure.pCPAL_TransferTx->wAddr2   = (uint32_t)RegName;
  
  /* Write Operation */
  if(CPAL_I2C_Write(&I2C2_DevStructure) == CPAL_PASS)
  {
    while ((I2C2_DevStructure.CPAL_State != CPAL_STATE_READY) && (I2C2_DevStructure.CPAL_State != CPAL_STATE_ERROR) )
    { }
    
    if (I2C2_DevStructure.CPAL_State == CPAL_STATE_ERROR)
    {
      return I2C2_FAIL;
    }
  }
  else
  {
    return I2C2_FAIL;
  }
  
  return I2C2_OK;
}

/**
  * @brief  Read Temperature register of LM75: double temperature value.
  * @param  None
  * @retval LM75 measured temperature value.
  */
uint16_t I2C2_ReadTemp(void)
{   
  uint16_t tmp = 0;
  
  I2C2_Buffer[0] = 0;
  I2C2_Buffer[1] = 0;
  
  /* Disable all options */
  I2C2_DevStructure.wCPAL_Options = 0;

  /* point to CPAL_TransferTypeDef structure */
  I2C2_DevStructure.pCPAL_TransferRx = &I2C2_RXTransfer;
  
  /* Configure transfer parameters */  
  I2C2_DevStructure.pCPAL_TransferRx->wNumData = 2;
  I2C2_DevStructure.pCPAL_TransferRx->pbBuffer = I2C2_Buffer ;
  I2C2_DevStructure.pCPAL_TransferRx->wAddr1   = (uint32_t)I2C2_ADDR;
  I2C2_DevStructure.pCPAL_TransferRx->wAddr2   = (uint32_t)I2C2_REG_TEMP;
  
  /* Read Operation */
  if(CPAL_I2C_Read(&I2C2_DevStructure) == CPAL_PASS)
  {
    while ((I2C2_DevStructure.CPAL_State != CPAL_STATE_READY) && (I2C2_DevStructure.CPAL_State != CPAL_STATE_ERROR) )
    { }
  }
  
  /* Store I2C2_I2C received data */
  tmp = (uint16_t)(I2C2_Buffer[0] << 8);
  tmp |= I2C2_Buffer[1];    
  
  /* Return Temperature value */
  return (uint16_t)(tmp >> 7);
}

/**
  * @brief  Read the configuration register from the LM75.
  * @param  None
  * @retval LM75 configuration register value.
  */
uint8_t I2C2_ReadConfReg(void)
{    
  I2C2_Buffer[0] = 0;
  
  /* Disable all options */
  I2C2_DevStructure.wCPAL_Options = 0;
  
  /* point to CPAL_TransferTypeDef structure */
  I2C2_DevStructure.pCPAL_TransferRx = &I2C2_RXTransfer;
  
  /* Configure transfer parameters */  
  I2C2_DevStructure.pCPAL_TransferRx->wNumData = 1;
  I2C2_DevStructure.pCPAL_TransferRx->pbBuffer = I2C2_Buffer ;
  I2C2_DevStructure.pCPAL_TransferRx->wAddr1   = (uint32_t)I2C2_ADDR;
  I2C2_DevStructure.pCPAL_TransferRx->wAddr2   = (uint32_t)I2C2_REG_CONF;
  
  /* Read Operation */
  if(CPAL_I2C_Read(&I2C2_DevStructure) == CPAL_PASS)
  {
    while ((I2C2_DevStructure.CPAL_State != CPAL_STATE_READY) && (I2C2_DevStructure.CPAL_State != CPAL_STATE_ERROR) )
    { }
  }
  
  /* Return Temperature value */
  return (uint8_t)I2C2_Buffer[0];
}

/**
  * @brief  Write to the configuration register of the LM75.
  * @param  RegValue: specifies the value to be written to LM75 configuration 
  *         register.
  * @retval None
  */
uint8_t I2C2_WriteConfReg(uint8_t RegValue)
{   
  I2C2_Buffer[0] = (uint8_t)(RegValue);
    
  /* Disable all options */
  I2C2_DevStructure.wCPAL_Options = 0;
  
  /* point to CPAL_TransferTypeDef structure */
  I2C2_DevStructure.pCPAL_TransferTx = &I2C2_TXTransfer;
  
  /* Configure transfer parameters */  
  I2C2_DevStructure.pCPAL_TransferTx->wNumData = 1;
  I2C2_DevStructure.pCPAL_TransferTx->pbBuffer = I2C2_Buffer ;
  I2C2_DevStructure.pCPAL_TransferTx->wAddr1   = (uint32_t)I2C2_ADDR;
  I2C2_DevStructure.pCPAL_TransferTx->wAddr2   = (uint32_t)I2C2_REG_CONF;
  
  /* Write Operation */
  if(CPAL_I2C_Write(&I2C2_DevStructure) == CPAL_PASS)
  {
    while ((I2C2_DevStructure.CPAL_State != CPAL_STATE_READY) && (I2C2_DevStructure.CPAL_State != CPAL_STATE_ERROR) )
    { }
    
    if (I2C2_DevStructure.CPAL_State == CPAL_STATE_ERROR)
    {
      return I2C2_FAIL;
    }
  }
  else
  {
    return I2C2_FAIL;
  }
  
  return I2C2_OK;
  
}

/**
  * @brief  Enables or disables the LM75.
  * @param  NewState: specifies the LM75 new status. This parameter can be ENABLE
  *         or DISABLE.  
  * @retval None
  */
uint8_t I2C2_ShutDown(FunctionalState NewState)
{   
  __IO uint8_t RegValue = 0;    
  
  I2C2_Buffer[0] = 0;
  		  
  /* Disable all options */
  I2C2_DevStructure.wCPAL_Options = 0;

  /* point to CPAL_TransferTypeDef structure */
  I2C2_DevStructure.pCPAL_TransferRx = &I2C2_RXTransfer;
  
  /* Configure transfer parameters */  
  I2C2_DevStructure.pCPAL_TransferRx->wNumData = 1;
  I2C2_DevStructure.pCPAL_TransferRx->pbBuffer = I2C2_Buffer ;
  I2C2_DevStructure.pCPAL_TransferRx->wAddr1   = (uint32_t)I2C2_ADDR;
  I2C2_DevStructure.pCPAL_TransferRx->wAddr2   = (uint32_t)I2C2_REG_CONF;
  
  /* Read Operation */
  if(CPAL_I2C_Read(&I2C2_DevStructure) == CPAL_PASS)
  {
    while ((I2C2_DevStructure.CPAL_State != CPAL_STATE_READY) && (I2C2_DevStructure.CPAL_State != CPAL_STATE_ERROR) )
    { }
  }
  
  /* Get received data */
  RegValue = (uint8_t)I2C2_Buffer[0];
  
  /*---------------------------- Transmission Phase ---------------------------*/
  
  /* Enable or disable SD bit */
  if (NewState != DISABLE)
  {
    /* Enable LM75 */
    I2C2_Buffer[0] = RegValue & I2C2_SD_RESET;
  }
  else
  {
    /* Disable LM75 */
    I2C2_Buffer[0] = RegValue | I2C2_SD_SET;
  }  
  
  /* point to CPAL_TransferTypeDef structure */
  I2C2_DevStructure.pCPAL_TransferTx = &I2C2_TXTransfer;
  
  /* Configure transfer parameters */  
  I2C2_DevStructure.pCPAL_TransferTx->wNumData = 1;
  I2C2_DevStructure.pCPAL_TransferTx->pbBuffer = I2C2_Buffer ;
  I2C2_DevStructure.pCPAL_TransferTx->wAddr1   = (uint32_t)I2C2_ADDR;
  I2C2_DevStructure.pCPAL_TransferTx->wAddr2   = (uint32_t)I2C2_REG_CONF;
  
  /* Write Operation */
  if(CPAL_I2C_Write(&I2C2_DevStructure) == CPAL_PASS)
  {
    while ((I2C2_DevStructure.CPAL_State != CPAL_STATE_READY) && (I2C2_DevStructure.CPAL_State != CPAL_STATE_ERROR) )
    { }
    
    if (I2C2_DevStructure.CPAL_State == CPAL_STATE_ERROR)
    {
      return I2C2_FAIL;
    }
  }
  else
  {
    return I2C2_FAIL;
  }
  
  return I2C2_OK;
}

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

uint8_t I2Cx_read_byte(CPAL_DevTypeDef i2cx, uint8_t addr, uint8_t reg)
{
    uint8_t tmp = 0;
  
    I2C2_Buffer[0] = 0;
  
    /* Disable all options */
    I2C2_DevStructure.wCPAL_Options = 0;

    /* point to CPAL_TransferTypeDef structure */
    I2C2_DevStructure.pCPAL_TransferRx = &I2C2_RXTransfer;
  
    /* Configure transfer parameters */  
    I2C2_DevStructure.pCPAL_TransferRx->wNumData = 1;
    I2C2_DevStructure.pCPAL_TransferRx->pbBuffer = I2C2_Buffer ;
    I2C2_DevStructure.pCPAL_TransferRx->wAddr1   = (uint32_t)(addr << 1);
    I2C2_DevStructure.pCPAL_TransferRx->wAddr2   = (uint32_t)reg;
  
    /* Read Operation */
    if(CPAL_I2C_Read(&I2C2_DevStructure) == CPAL_PASS)
    {
        while ((I2C2_DevStructure.CPAL_State != CPAL_STATE_READY) && (I2C2_DevStructure.CPAL_State != CPAL_STATE_ERROR) )
        { }
    }
  
    /* Store I2C2_I2C received data */
    tmp = I2C2_Buffer[0];
  
    /* return a Reg value */
    return (uint8_t)tmp;  
}

I2C2_Status_TypDef I2Cx_read_byte_buf(CPAL_DevTypeDef i2cx, uint8_t addr, uint8_t reg, uint8_t* buf)
{
    return I2C2_OK;
}

I2C2_Status_TypDef I2Cx_read_mbytes_buf(CPAL_DevTypeDef i2cx, uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buf)
{
    return I2C2_OK;
}

I2C2_Status_TypDef I2Cx_write_byte(CPAL_DevTypeDef i2cx, uint8_t addr, uint8_t reg, uint8_t data)
{
    I2C2_Buffer[0] = (uint8_t)(data);
     
    /* Disable all options */
    I2C2_DevStructure.wCPAL_Options = 0;
  
    /* point to CPAL_TransferTypeDef structure */
    I2C2_DevStructure.pCPAL_TransferTx = &I2C2_TXTransfer;
  
    /* Configure transfer parameters */  
    I2C2_DevStructure.pCPAL_TransferTx->wNumData = 1;
    I2C2_DevStructure.pCPAL_TransferTx->pbBuffer = I2C2_Buffer ;
    I2C2_DevStructure.pCPAL_TransferTx->wAddr1   = (uint32_t)(addr << 1);
    I2C2_DevStructure.pCPAL_TransferTx->wAddr2   = (uint32_t)reg;
  
    /* Write Operation */
    if(CPAL_I2C_Write(&I2C2_DevStructure) == CPAL_PASS)
    {
        while ((I2C2_DevStructure.CPAL_State != CPAL_STATE_READY) && (I2C2_DevStructure.CPAL_State != CPAL_STATE_ERROR) )
        { }
    
        if (I2C2_DevStructure.CPAL_State == CPAL_STATE_ERROR)
        {
            return I2C2_FAIL;
        }
    }
    else
    {
        return I2C2_FAIL;
    }
  
    return I2C2_OK;
}

I2C2_Status_TypDef I2Cx_write_byte_buf(CPAL_DevTypeDef i2cx, uint8_t addr, uint8_t reg, uint8_t* buf)
{
    return I2C2_OK;
}

I2C2_Status_TypDef I2Cx_write_mbytes_buf(CPAL_DevTypeDef i2cx, uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buf)
{
    return I2C2_OK;
}

