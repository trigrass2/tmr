/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
/******************************************************************************
 *
 * $Id: umpl-setup-manits-msb.c 2011-04-15  phickey $
 *
 *****************************************************************************/

/** 
 *  @defgroup UMPL 
 *  @brief  uMPL Layer.
 *          umpl-setup-manits-9x provides platform setup code for Mantis
 *          with an AK8975 slave.
 *
 *  @{
 *      @file   umpl-setup-manits-msb.c
 *      @brief  Performs set-up specific to platform.
**/

/* ------------------ */
/* - Include Files. - */
/* ------------------ */
#include <stdio.h>
#include <string.h>

#include "umpl-setup.h"

#include "ml.h"
#include "mldl.h"
#include "mlstates.h"
#include "mlFIFO.h"
#include "mpu.h"
#include "mldl_cfg.h"
#include "slave.h"

#ifdef UMPL_ENABLE_TEMPCOMP
#include "temp_comp.h"
#endif

#ifdef UMPL_NINE_AXIS
#include "akm/inv_external_slave_akm8975.h"
#endif

#include "log.h"
#undef MPL_LOG_TAG
#define MPL_LOG_TAG "umpl-setup"

/* found in mldl.c */
extern struct ext_slave_descr gAccel;
extern struct ext_slave_descr gCompass;
extern struct mpu_platform_data gPdata;

/* --------------- */
/* -  Functions. - */
/* --------------- */

/**
 *  @brief  umplPlatformSetup initializes the platform data structure to the required values.
 *			It sets the orientation matrix, the slave address of the gyro and the accel.
 *
 *
 *  @return INV_SUCCESS if successful, a non-zero error code otherwise.
 */

inv_error_t umplPlatformSetup(void)
{
    return INV_SUCCESS;
}

void umplMotionCallback(unsigned short motionstate)
{
    if (motionstate == INV_MOTION)
        MPL_LOGI("umplMotionCallback: INV_MOTION\n");
    else if (motionstate == INV_NO_MOTION)
        MPL_LOGI("umplMotionCallback: INV_NO_MOTION\n");
}
/**
 *  @brief  umplDmpSetup is called by Called by umplStartMPU(). This function 
 *			enables the required features such as turning on the bias trackers 
 *			and temperature compensation. This function also enables the required 
 *			type of data to be put in FIFO.
 *
 *
 *  @pre    To be run after inv_dmp_open(), before inv_dmp_start().
 *
 *  @return INV_SUCCESS if successful, a non-zero error code otherwise.
 */

inv_error_t umplDmpSetup(void)
{
    inv_error_t result;
 
    /* Setup Bias Trackers: 
     * inv_set_bias_update
     * Known to work with the following arguments:
     * INV_BIAS_FROM_NO_MOTION
     * INV_BIAS_FROM_TEMPERATURE
     * INV_LEARN_BIAS_FROM_TEMPERATURE
     *
     */
#if defined UMPL_NINE_AXIS
    inv_set_mpu_sensors(INV_NINE_AXIS);
#else
    inv_set_mpu_sensors(INV_SIX_AXIS_GYRO_ACCEL);
#endif

#ifdef UMPL_ENABLE_TEMPCOMP
    result = inv_set_bias_update( INV_BIAS_FROM_NO_MOTION
                                | INV_BIAS_FROM_TEMPERATURE
                                | INV_LEARN_BIAS_FROM_TEMPERATURE ); 
#else
    result = inv_set_bias_update( INV_BIAS_FROM_NO_MOTION );
#endif
    if (result != INV_SUCCESS)
        MPL_LOGE("umpl inv_set_bias_update returned %d\n",result);

#ifdef UMPL_NINE_AXIS
    result = inv_external_slave_akm8975_open();
    if (result != INV_SUCCESS)
        MPL_LOGE("umpl inv_external_slave_akm8975_open returned %d\n",result);
#endif

    inv_set_fifo_interrupt(TRUE);
      
    // Setup FIFO:
    result = inv_send_quaternion(INV_32_BIT);
    if (result != INV_SUCCESS)
        MPL_LOGE("umpl FIFOSendQuaterion returned %d\n",result);
    
    result = inv_send_gyro(INV_ALL,INV_32_BIT);
    if (result != INV_SUCCESS)
        MPL_LOGE("umpl inv_send_gyro returned %d\n",result);
    
    result = inv_send_accel(INV_ALL,INV_32_BIT);
    if (result != INV_SUCCESS)
        MPL_LOGE("umpl inv_send_accel returned %d\n",result);

    result = inv_set_fifo_rate(10);
    if (result != INV_SUCCESS)
        MPL_LOGE("umpl inv_set_fifo_rate returned %d\n",result);

#ifdef UMPL_ENABLE_TEMPCOMP
    result = inv_enable_temp_comp();
    if (result != INV_SUCCESS)
       MPL_LOGE("umpl inv_enable_temp_comp returned %d\n",result);
#endif

    result = inv_set_motion_callback(umplMotionCallback);
    if (result != INV_SUCCESS)
       MPL_LOGE("umpl inv_set_motion_callback returned %d\n",result);
    return INV_SUCCESS;
}

