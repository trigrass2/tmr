/**
  ******************************************************************************
  * @file    STM32_EVAL_CPAL/Common/drv_i2c_cpal.h
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    17-June-2011
  * @brief   This file contains all the functions prototypes for the 
  *          drv_i2c_cpal.c firmware driver.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DRV_I2C_CPAL_H
#define __DRV_I2C_CPAL_H

#ifdef __cplusplus
 extern "C" {
#endif
   
   
/* Includes ------------------------------------------------------------------*/
#include "cpal_i2c.h"
   
/* Exported types ------------------------------------------------------------*/

/** 
  * @brief  TSENSOR Status  
  */ 
typedef enum
{
  LM75_OK = 0,
  LM75_FAIL
}LM75_Status_TypDef;

/* Exported constants --------------------------------------------------------*/
    
/* CPAL Structure configuration : Select I2C device (uncomment relative define) */

//#define LM75_DevStructure                I2C1_DevStructure   
#define LM75_DevStructure                I2C2_DevStructure  
//#define LM75_DevStructure                I2C3_DevStructure 

   
/* Select clock Speed */
/* To use the I2C at 400 KHz (in fast mode), the PCLK1 frequency (I2C peripheral
   input clock) must be a multiple of 10 MHz */

#define I2C_SPEED                        300000

/* Select interrupt programming model : By default DMA programming model is selected.
 To select interrupt programming model uncomment this define */
//#define LM75_IT

/* Maximum Timeout values for waiting until device is ready for communication.*/
   
#define LM75_TIMEOUT        ((uint32_t)0x3FFFF)

/**
  * @brief  Internal register Memory
  */
#define LM75_REG_TEMP       0x00  /*!< Temperature Register of LM75 */
#define LM75_REG_CONF       0x01  /*!< Configuration Register of LM75 */
#define LM75_REG_THYS       0x02  /*!< Temperature Register of LM75 */
#define LM75_REG_TOS        0x03  /*!< Over-temp Shutdown threshold Register of LM75 */
#define LM75_ADDR           0x90   /*!< LM75 address */
   

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */ 

void LM75_DeInit(void);
void LM75_Config(void);
ErrorStatus LM75_GetStatus(void);
uint16_t LM75_ReadTemp(void);
uint16_t LM75_ReadReg(uint8_t RegName);
uint8_t LM75_WriteReg(uint8_t RegName, uint16_t RegValue);
uint8_t LM75_ReadConfReg(void);
uint8_t LM75_WriteConfReg(uint8_t RegValue);
uint8_t LM75_ShutDown(FunctionalState NewState);


 
#ifdef __cplusplus
}
#endif

#endif /* __STM32_EVAL_I2C_TSENSOR_CPAL_H */
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */ 

/**
  * @}
  */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
