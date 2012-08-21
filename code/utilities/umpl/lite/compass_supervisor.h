/*
 $License:
    Copyright (C) 2011 InvenSense Corporation, All Rights Reserved.
 $
 */

/*******************************************************************************
 *
 * $Id:$
 *
 ******************************************************************************/

#ifndef INVENSENSE_INV_COMPASS_SUPERVISOR_H__
#define INVENSENSE_INV_COMPASS_SUPERVISOR_H__

#include "mltypes.h"
#include "mlinclude.h"
#include "ml.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_COMPASS_RATE_PROCESSES 6

struct compass_obj_t {
    long raw[3];
        /* raw: Raw values from magnetometer. Board frame & units. */
    long bias[3];
        /* bias: DMP scaled bias. Board frame. Board units * 2^16. */
    long init_bias[3];
        /* init_bias: Used by adv only. Board frame. Board units * 2^16. */
    unsigned long delta_time;
        /* delta_time: milliseconds since last measurement. */
    bool got_bias;
        /* got bias: whether magnetometer bias is valid or not. */
    inv_error_t (*update_bias)(long *bias);
        /* update_bias: callback for setting new bias. Bias passed must
         * be board frame, DMP scaled (board units * 2^16).
         */
    inv_error_t (*lost_bias)(void);
        /* lost_bias: callback for declaring the current bias invalid.
         * Typically called when magnetic disturbance is detected.
         */
};


#define INV_COMPASS_PRIORITY_RAW_DATA 100
#define INV_COMPASS_ADV_BIAS_FUSION 200
#define INV_COMPASS_PRIORITY_EXTERNAL_CALIBRATION 200
#define INV_COMPASS_PRIORITY_UPDATE_INVOBJ 300
#define INV_COMPASS_SET_9AXIS_QUATERNION_ADJUSTMENT 400

inv_error_t inv_enable_compass_supervisor(void);
inv_error_t inv_disable_compass_supervisor(void);
inv_error_t inv_run_compass_rate_processes(struct inv_obj_t *inv_obj);
inv_error_t inv_unregister_compass_rate_process(
    inv_error_t (*func)(struct compass_obj_t *obj));
inv_error_t inv_register_compass_rate_process(
    inv_error_t (*func)(struct compass_obj_t *obj), int priority);
inv_error_t inv_calibrate_compass(struct compass_obj_t *obj);

#ifdef __cplusplus
}
#endif

#endif // INVENSENSE_INV_COMPASS_SUPERVISOR_H__

