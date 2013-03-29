#include "drv_pca9536.h"

/*
   Register 0 - Input Port register ( read-only )
   
   This register is a read-only port. It reflects the incoming logic levels of the pins, regardless 
   of whether the pin is defined as an input or an output by Register 3. Writes to this register 
   have no effect.
 */
uint8_t PCA9536_REG_IX_READ(void)
{  
  /* Get all the Pins direction */
  return I2C_ReadDeviceRegister(PCA9536_ADDRESS, PCA9536_REG_IX);;      
}

/*
   Register 1 - Output Port register
   
   This register reflects the outgoing logic levels of the pins defined as outputs by Register 3. 
   Bit values in this register have no effect on pins defined as inputs. Reads from this register 
   return the value that is in the flip-flop controlling the output selection, not the actual pin 
   value.
 */
uint8_t PCA9536_REG_OX_READ(void)
{  
  /* Get all the Pins direction */
  return I2C_ReadDeviceRegister(PCA9536_ADDRESS, PCA9536_REG_OX);;      
}

uint8_t PCA9536_REG_OX_WRITE(uint8_t bit, OX_VALUE_TypeDef value)
{
  uint8_t tmp = 0;   
  
  /* Get all the Pins direction */
  tmp = I2C_ReadDeviceRegister(PCA9536_ADDRESS, PCA9536_REG_OX);
  
  if (value != LOGIC_1)
  {
    tmp |= (uint8_t)bit;
  }  
  else 
  {
    tmp &= ~(uint8_t)bit;
  }
  
  /* Write the register new value */
  I2C_WriteDeviceRegister(PCA9536_ADDRESS, PCA9536_REG_OX, tmp);
  
  /* If all OK return OK */
  return OK;      
}

/* 
   Register 2 - Polarity Inversion register
   
   This register allows the user to invert the polarity of the Input Port register data. If a bit in 
   this register is set (written with ．1・), the corresponding Input Port data is inverted. If a bit in 
   this register is cleared (written with a ．0・), the Input Port data polarity is retained.
 */
uint8_t PCA9536_REG_CX_READ(void)
{  
  /* Get all the Pins direction */
  return I2C_ReadDeviceRegister(PCA9536_ADDRESS, PCA9536_REG_NX);;      
}

uint8_t PCA9536_REG_NX_WRITE(uint8_t bit, NX_VALUE_TypeDef value)
{
  uint8_t tmp = 0;   
  
  /* Get all the Pins direction */
  tmp = I2C_ReadDeviceRegister(PCA9536_ADDRESS, PCA9536_REG_NX);
  
  if (value != INVERTED)
  {
    tmp |= (uint8_t)bit;
  }  
  else 
  {
    tmp &= ~(uint8_t)bit;
  }
  
  /* Write the register new value */
  I2C_WriteDeviceRegister(PCA9536_ADDRESS, PCA9536_REG_NX, tmp);
  
  /* If all OK return OK */
  return OK;      
}

/*
   Register 3 - Configuration register
   
   This register configures the directions of the I/O pins. If a bit in this register is set, the 
   corresponding port pin is enabled as an input with high-impedance output driver. If a bit in 
   this register is cleared, the corresponding port pin is enabled as an output. At reset, the 
   I/Os are configured as inputs with a weak pull-up to VDD.
*/
uint8_t PCA9536_REG_CX_READ(void)
{  
  /* Get all the Pins direction */
  return I2C_ReadDeviceRegister(PCA9536_ADDRESS, PCA9536_REG_CX);;      
}

uint8_t PCA9536_REG_CX_WRITE(uint8_t bit, CX_VALUE_TypeDef value)
{
  uint8_t tmp = 0;   
  
  /* Get all the Pins direction */
  tmp = I2C_ReadDeviceRegister(PCA9536_ADDRESS, PCA9536_REG_CX);
  
  if (value != INPUT)
  {
    tmp |= (uint8_t)bit;
  }  
  else 
  {
    tmp &= ~(uint8_t)bit;
  }
  
  /* Write the register new value */
  I2C_WriteDeviceRegister(PCA9536_ADDRESS, PCA9536_REG_CX, tmp);
  
  /* If all OK return OK */
  return OK;      
}
