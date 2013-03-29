#ifndef __TMR_BOARD_H__
#define __TMR_BOARD_H__

/* Standard includes. */
#include <stdbool.h>
#include <stdio.h>
#include <math.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* ST StdPeriph Driver includes. */
#include "stm32f4xx_conf.h"

#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

#define digitalHi(p, i)     { p->BSRRL = i; }
#define digitalLo(p, i)     { p->BSRRH = i; }
#define digitalToggle(p, i) { p->ODR ^= i; }

// Hardware definitions and GPIO

#define BEEP_GPIO   GPIOB
#define BEEP_PIN    GPIO_Pin_12

#define BEEP_TOGGLE digitalToggle(BEEP_GPIO, BEEP_PIN);
#define BEEP_ON    digitalHi(BEEP_GPIO, BEEP_PIN);
#define BEEP_OFF     digitalLo(BEEP_GPIO, BEEP_PIN);

#endif
