/*
 $License:
    Copyright (C) 2011 InvenSense Corporation, All Rights Reserved.
 $
 */
/*******************************************************************************
 *
 * $Id: compass.h 5629 2011-06-11 03:13:08Z mcaramello $
 *
 *******************************************************************************/

#ifndef COMPASS_H
#define COMPASS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "mltypes.h"
#include "mpu.h"
#ifdef INV_INCLUDE_LEGACY_HEADERS
#include "compass_legacy.h"
#endif
    /* ------------ */
    /* - Defines. - */
    /* ------------ */

    /* --------------- */
    /* - Structures. - */
    /* --------------- */

    /* --------------------- */
    /* - Function p-types. - */
    /* --------------------- */

    unsigned char inv_compass_present(void);
    unsigned char inv_get_compass_slave_addr(void);
    inv_error_t inv_get_compass_data(long *data);
    inv_error_t inv_set_compass_bias(long *bias);
    inv_error_t inv_set_new_compass_bias(long *bias);
    unsigned short inv_get_compass_id(void);
    inv_error_t inv_compass_write_reg(unsigned char reg, unsigned char val);
    inv_error_t inv_compass_read_reg(unsigned char reg, unsigned char *val);
    inv_error_t inv_compass_read_scale(long *val);

#ifdef __cplusplus
}
#endif
#endif                          // COMPASS_H
