#ifndef __DRV_PCA9536_H__
#define __DRV_PCA9536_H__

#include "tmr_fc.h"

#define PCA9536_ADDRESS 0x41

/*
   Register 0 - Input Port register ( read-only )
   
   This register is a read-only port. It reflects the incoming logic levels of the pins, regardless 
   of whether the pin is defined as an input or an output by Register 3. Writes to this register 
   have no effect.
 */
#define PCA9536_REG_IX  0x00
#define PCA9536_REG_I0  0x00
#define PCA9536_REG_I1  0x01
#define PCA9536_REG_I2  0x02
#define PCA9536_REG_I3  0x03

/*
   Register 1 - Output Port register
   
   This register reflects the outgoing logic levels of the pins defined as outputs by Register 3. 
   Bit values in this register have no effect on pins defined as inputs. Reads from this register 
   return the value that is in the flip-flop controlling the output selection, not the actual pin 
   value.
 */
#define PCA9536_REG_OX  0x01
#define PCA9536_REG_O0  0x00
#define PCA9536_REG_O1  0x01
#define PCA9536_REG_O2  0x02
#define PCA9536_REG_O3  0x03

#define LOGIC_0 0x00
#define LOGIC_1 0x01

typedef enum 
{
  LOGIC_0 = 0,
  LOGIC_1 = 1
} OX_VALUE_TypeDef;
/* 
   Register 2 - Polarity Inversion register
   
   This register allows the user to invert the polarity of the Input Port register data. If a bit in 
   this register is set (written with ．1・), the corresponding Input Port data is inverted. If a bit in 
   this register is cleared (written with a ．0・), the Input Port data polarity is retained.
 */
#define PCA9536_REG_NX  0x02
#define PCA9536_REG_N0  0x00
#define PCA9536_REG_N1  0x01
#define PCA9536_REG_N2  0x02
#define PCA9536_REG_N3  0x03

#define RETAINED 0x00
#define INVERTED 0x01

typedef enum 
{
  RETAINED = 0,
  INVERTED = 1
} NX_VALUE_TypeDef;

/*
   Register 3 - Configuration register
   
   This register configures the directions of the I/O pins. If a bit in this register is set, the 
   corresponding port pin is enabled as an input with high-impedance output driver. If a bit in 
   this register is cleared, the corresponding port pin is enabled as an output. At reset, the 
   I/Os are configured as inputs with a weak pull-up to VDD.
*/
#define PCA9536_REG_CX  0x03
#define PCA9536_REG_C0  0x00
#define PCA9536_REG_C1  0x01
#define PCA9536_REG_C2  0x02
#define PCA9536_REG_C3  0x03

#define OUTPUT 0x00
#define INPUT 0x01

typedef enum 
{
  OUTPUT = 0,
  INPUT = 1
} CX_VALUE_TypeDef;

#endif
