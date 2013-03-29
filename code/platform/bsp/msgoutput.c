/**
  ******************************************************************************
  * @file    lcd_log.c
  * @author  MCD Application Team
  * @version V5.0.2
  * @date    05-March-2012
  * @brief   This file provides all the LCD Log firmware functions.
  *          
  *          The LCD Log module allows to automatically set a header and footer
  *          on any application using the LCD display and allows to dump user,
  *          debug and error messages by using the following macros: LCD_ErrLog(),
  *          LCD_UsrLog() and LCD_DbgLog().
  *         
  *          It supports also the scroll feature by embedding an internal software
  *          cache for display. This feature allows to dump message sequentially
  *          on the display even if the number of displayed lines is bigger than
  *          the total number of line allowed by the display.
  *      
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include  "msgoutput.h"

/** @addtogroup Utilities
  * @{
  */

/** @addtogroup STM32_EVAL
* @{
*/

/** @addtogroup Common
  * @{
  */

/** @defgroup LCD_LOG 
* @brief LCD Log LCD_Application module
* @{
*/ 

/** @defgroup LCD_LOG_Private_Types
* @{
*/ 
/**
* @}
*/ 


/** @defgroup LCD_LOG_Private_Defines
* @{
*/ 

/**
* @}
*/ 


/** @defgroup LCD_LOG_Private_Macros
* @{
*/ 
/**
* @}
*/ 


/** @defgroup LCD_LOG_Private_Variables
* @{
*/ 

LCD_LOG_line LCD_CacheBuffer [LCD_CACHE_DEPTH]; 
uint16_t LCD_LineColor;
uint16_t LCD_CacheBuffer_xptr;
uint16_t LCD_CacheBuffer_yptr_top;
uint16_t LCD_CacheBuffer_yptr_bottom;

uint16_t LCD_CacheBuffer_yptr_top_bak;
uint16_t LCD_CacheBuffer_yptr_bottom_bak;

FunctionalState LCD_CacheBuffer_yptr_invert;
FunctionalState LCD_ScrollActive;
FunctionalState LCD_Lock;
FunctionalState LCD_Scrolled;
uint16_t LCD_ScrollBackStep;

/**
* @}
*/ 


/** @defgroup LCD_LOG_Private_Functions
* @{
*/ 


/**
* @brief  Init the LCD Log module 
* @param  None
* @retval None
*/

void LCD_LOG_Init ( void)
{

}

/**
* @brief DeInitializes the LCD Log module. 
* @param  None
* @retval None
*/
void LCD_LOG_DeInit(void)
{

}

/**
* @brief  Display the application header (title) on the LCD screen 
* @param  Title :  pointer to the string to be displayed
* @retval None
*/
void LCD_LOG_SetHeader (uint8_t *Title)
{

}

/**
* @brief  Display the application footer (status) on the LCD screen 
* @param  Status :  pointer to the string to be displayed
* @retval None
*/
void LCD_LOG_SetFooter(uint8_t *Status)
{

}

/**
* @brief  Clear the Text Zone 
* @param  None 
* @retval None
*/
void LCD_LOG_ClearTextZone(void)
{

}

/**
* @brief  Redirect the printf to the lcd 
* @param  c: character to be displayed
* @param  f: output file pointer
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

#ifdef LCD_SCROLL_ENABLED
/**
* @brief  Display previous text frame
* @param  None
* @retval Status
*/
ErrorStatus LCD_LOG_ScrollBack (void)
{
    return SUCCESS;
}

/**
* @brief  Display next text frame
* @param  None
* @retval Status
*/
ErrorStatus LCD_LOG_ScrollForward (void)
{
    return SUCCESS; 
}
#endif /* LCD_SCROLL_ENABLED */

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


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
