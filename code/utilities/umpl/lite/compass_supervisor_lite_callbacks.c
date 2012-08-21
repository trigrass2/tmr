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

#include "compass_supervisor_inv_obj_callbacks.h"
#include "compass_supervisor.h"

#include "ml.h"
#include "mldl.h"
#include "mlMathFunc.h"


#ifdef INV_FEATURE_ADV_FUSION
#error compass_supervisor_lite_callbacks should not be used with INV_FEATURE_ADV_FUSION
#error Disable ADV_FUSION or use compass_supervisor_adv_callbacks instead
#endif

/* Lite Fusion Compass Supervisor Delegates.
 * Take compass_supervisor's compass_obj_t, and populate the
 * inv_obj->mag data structure with the results.
 * (Admittedly, lite_fusion is a little bit of a misnomer.)
 */
static inv_error_t lite_fusion_calibrate_compass(struct compass_obj_t *);
static inv_error_t lite_fusion_raw_compass_data(struct compass_obj_t *);

inv_error_t inv_register_inv_obj_compass_supervisor_callbacks(void)
{
    inv_error_t result;
    result = inv_register_compass_rate_process(lite_fusion_raw_compass_data,
        INV_COMPASS_PRIORITY_RAW_DATA);
    if (result != INV_SUCCESS)
        return result;

    result = inv_register_compass_rate_process(lite_fusion_calibrate_compass,
        INV_COMPASS_PRIORITY_UPDATE_INVOBJ);
    return result;
}

inv_error_t inv_unregister_inv_obj_compass_supervisor_callbacks(void)
{
    inv_error_t result;

    result = inv_unregister_compass_rate_process(lite_fusion_raw_compass_data);
    if (result != INV_SUCCESS)
        return result;

    result = inv_unregister_compass_rate_process(lite_fusion_calibrate_compass);
    return result;
}

static inv_error_t lite_fusion_raw_compass_data(struct compass_obj_t *obj)
{
    int i;
    struct mldl_cfg *mldl_cfg = inv_get_dl_config();
    const signed char *orC =
        mldl_cfg->pdata_slave[EXT_SLAVE_TYPE_COMPASS]->orientation;
    for (i = 0; i < 3; i++) {
        int row = i*3;
        inv_obj.mag->sensor_data[i] =
            obj->raw[0]*orC[row+0] + obj->raw[1]*orC[row+1] + obj->raw[2]*orC[row+2];
    }
    return INV_SUCCESS;
}

/** Takes the raw compass data and removes the bias and applies the scaling to get
* calibrated compass data
*/
static inv_error_t lite_fusion_calibrate_compass(struct compass_obj_t *obj)
{
    int i;
    long unbiased_m[3]; /* magdata: board units * 2^16 */
    long scaled_m[3]; /* intermediate result: utesla * 2^16 */
    long scaled_b[3]; /* intermediate result: utesla * 2^16 */
    struct mldl_cfg *mldl_cfg = inv_get_dl_config();
    const signed char *orC =
        mldl_cfg->pdata_slave[EXT_SLAVE_TYPE_COMPASS]->orientation;
    const long sens = inv_obj.mag->sens;

    /* 
     * Calculate scaled bias.
     * Calculate bias corrected and scaled mag data.
     */
    for (i = 0; i < 3; i++) {
        unbiased_m[i] = obj->raw[i] - obj->bias[i];
        scaled_m[i] = inv_q_shift_mult(unbiased_m[i], sens, 30);
        scaled_b[i] = inv_q_shift_mult(obj->bias[i], sens, 30);
    }

    /* Set inv_obj members by rotating by orC */
    for (i = 0; i < 3; i++) {
        int row = i*3;
        inv_obj.mag->bias[i] =
            scaled_b[0]*orC[row+0] + scaled_b[1]*orC[row+1] + scaled_b[2]*orC[row+2];
        inv_obj.mag->calibrated_data[i] =
            scaled_m[0]*orC[row+0] + scaled_m[1]*orC[row+1] + scaled_m[2]*orC[row+2];
    }
    return INV_SUCCESS;
}

