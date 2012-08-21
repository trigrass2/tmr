/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
/*******************************************************************************
 *
 * $Id: mlos_win32.c 4598 2011-01-25 19:33:13Z prao $
 *
 *******************************************************************************/

/**
 *  @defgroup MLOS
 *  @brief OS Interface for Atmel AVR32
 *
 *  @{
 *      @file mlos.c
 *      @brief OS Interface.
 */

/* ------------- */
/* - Includes. - */
/* ------------- */

#include "mlos.h"
#include "stdint_invensense.h"

#if 0 // cctsao1008
#include <asf.h>
#endif

#include "log.h"
#undef MPL_LOG_TAG
#define MPL_LOG_TAG "mlos_at32"

/* -------------- */
/* - Functions. - */
/* -------------- */


/**
 *  @brief  Allocate space.
 *  @param  numBytes  
 *              number of bytes.
 *  @return pointer to allocated space.
 */
void *inv_malloc(unsigned int numBytes)
{
    MPL_LOGE("inv_malloc should not be used!\n");
    return NULL;
}


/**
 *  @brief  Free allocated space.
 *  @param  ptr pointer to space to deallocate
 *  @return error code.
 */
inv_error_t inv_free(void *ptr)
{
    MPL_LOGE("inv_free should not be used!\n");
    return INV_SUCCESS;
}





/**
 *  @brief  open file
 *  @param  filename    name of the file to open.
 *  @return error code.
 */
FILE *inv_fopen(char *filename)
{
    MPL_LOGE("inv_fopen should not be used!\n");
    return NULL;
}


/**
 *  @brief  close the file.
 *  @param  fp  handle to file to close.
 *  @return error code.
 */
void inv_fclose(FILE *fp)
{
    MPL_LOGE("inv_fclose should not be used!\n");
    return;
}


/**
 *  @brief  Sleep function.
**/
void inv_sleep(int mSecs)
{
    #if 0 // cctsao1008
    delay_ms(mSecs);
    #endif
}


/**
 *  @brief  get system's internal tick count.
 *          Used for time reference.
 *  @return current tick count.
**/
unsigned long inv_get_tick_count(void)
{
	//const long cpu_hz = 12000000;
	long count, ms=0;
	
	count = count;
	
	#if 0 // cctsao1008
	count = Get_system_register(AVR32_COUNT);
	ms = cpu_cy_2_ms(count,cpu_hz);
	#endif
	
	return ms;
}

/**
 *
 * MUTEX Stubs
 *
**/
inv_error_t inv_create_mutex(HANDLE *mutex) 
{
    return INV_SUCCESS;
}

inv_error_t inv_lock_mutex(HANDLE mutex)
{
    return INV_SUCCESS;
}

inv_error_t inv_unlock_mutex(HANDLE mutex)
{
    return INV_SUCCESS;
}

inv_error_t inv_destroy_mutex(HANDLE handle)
{
    return INV_SUCCESS;
}

  /**********************/
 /** @} */ /* defgroup */
/**********************/


