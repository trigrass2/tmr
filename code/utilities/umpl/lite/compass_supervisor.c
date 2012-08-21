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

#include "compass_supervisor.h"
#include "compass_supervisor_inv_obj_callbacks.h"

#include "mlMathFunc.h"
#include "mlFIFO.h"
#include "dmpKey.h"
#include "mlos.h"
#include "compass.h"
#include "mldl.h"

#include "ustore_manager.h"
#include "ustore_delegate_io.h"

#include "compass_debug.h"

#include "log.h"
#undef MPL_LOG_TAG
#define MPL_LOG_TAG "MPL-CompSup"

/* Compass Supervisor Internal Types
 * inv_compass_cb_t : type for a callback function registered to
 * the compass supervisor.
 * struct compass_rate_t : storage for callback functions and control
 * flow data members.
 */
typedef inv_error_t (*inv_compass_cb_t)(struct compass_obj_t *obj);

struct compass_rate_t {
    // These describe callbacks happening everytime a new compass value is read
    int_fast8_t num_cb;
    HANDLE mutex;
    inv_compass_cb_t compass_process_cb[MAX_COMPASS_RATE_PROCESSES];
    int priority[MAX_COMPASS_RATE_PROCESSES];
    unsigned long polltime;
    unsigned long pollrate;
};

/* Compass Supervisor Static Data Allocation */
static struct compass_rate_t compass_rate_obj;
static struct compass_obj_t compass_obj;

/* Static compass_obj_t initializer. */
static void init_compass_obj(struct compass_obj_t *);

/* Compass Supervisor Static Functions
 * - inv_try_compass is the interface for getting sensor data from compass.c
 * - disable_ and enable_dmp_9x_fusion set mag fusion feedback gain in DMP.
 * - supervisor_update_bias and supervisor_lost_bias allow a delegate to
 *   update the compass bias or notify that bias is lost (due to mag disturb.)
 *   They are passed to delegates as function pointers in compass_obj.
 */
static inv_error_t inv_try_compass(int *got_data);

static inv_error_t enable_dmp_9x_fusion(void);
static inv_error_t disable_dmp_9x_fusion(void);

static inv_error_t supervisor_update_bias(long *bias);
static inv_error_t supervisor_lost_bias(void);

/* Compass Supervisor UStore Interface
 * - store_compass_supervisor stores the current bias, and the got_bias flag.
 * - load_compass_bias will set the bias to the stored bias if the got_bias
 *   flag is true.
 * - compass_supervisor_ustore_data is registered with the ustore_manager.
 */
#define COMPASS_SUPERVISOR_STORAGE_SIZE (sizeof(long[3]) + sizeof(bool))
static inv_error_t load_compass_supervisor(void);
static inv_error_t store_compass_supervisor(void);

const static struct uloadstoredelegate compass_supervisor_ustore_delegate = {
    /* .store = */   store_compass_supervisor
    /* .load  = */ , load_compass_supervisor
    /* .len   = */ , COMPASS_SUPERVISOR_STORAGE_SIZE
    /* .tag   = */ , INV_USTORE_ID_LITE_FUSION
};

static void init_compass_obj(struct compass_obj_t * compass_obj)
{
    memset(compass_obj,0,sizeof(struct compass_obj_t));
    compass_obj->update_bias = supervisor_update_bias;
    compass_obj->lost_bias = supervisor_lost_bias;
}

inv_error_t inv_enable_compass_supervisor()
{
    inv_error_t result;

    init_compass_obj(&compass_obj);

    result = inv_create_mutex(&compass_rate_obj.mutex);
    if (result) {
        LOG_RESULT_LOCATION(result);
        return result;
    }
    result = inv_register_fifo_rate_process(inv_run_compass_rate_processes,
        INV_PRIORITY_COMPASS_SUPERVISOR);
    if (result) {
        LOG_RESULT_LOCATION(result);
        return result;
    }
    compass_rate_obj.num_cb = 0;
    compass_rate_obj.polltime = 0;
    compass_rate_obj.pollrate = 20;

    /* inv_obj compass supervisor callbacks are registered
     * by an external function, so that compass_supervisor.c
     * doesn't depend on inv_obj in any way.
     */
    result = inv_register_inv_obj_compass_supervisor_callbacks();
    if (result) {
        LOG_RESULT_LOCATION(result);
        return result;
    }

    result = inv_ustore_register_handler(&compass_supervisor_ustore_delegate);
    if (result) {
        LOG_RESULT_LOCATION(result);
        return result;
    }
    return INV_SUCCESS;
}

inv_error_t inv_disable_compass_supervisor()
{
    inv_error_t result;
    result = inv_unregister_inv_obj_compass_supervisor_callbacks();
    if (result) {
        LOG_RESULT_LOCATION(result);
        return result;
    }
    result = inv_unregister_fifo_rate_process(inv_run_compass_rate_processes);
    return result;
}

/**
 * @internal
 * @brief   This registers a function to be called for each set of
 *          gyro/quaternion/rotation matrix/etc output.
 * @param[in] func The callback function to register
 * @param[in] priority The unique priority number of the callback. Lower numbers
 *            are called first.
 * @return  error code.
 */
inv_error_t inv_register_compass_rate_process(inv_error_t (*func)(struct compass_obj_t *obj), int priority)
{
    INVENSENSE_FUNC_START;
    inv_error_t result;
    int kk, nn;

    result = inv_lock_mutex(compass_rate_obj.mutex);
    if (INV_SUCCESS != result) {
        return result;
    }
    // Make sure we haven't registered this function already
    // Or used the same priority
    for (kk = 0; kk < compass_rate_obj.num_cb; ++kk) {
        if ((compass_rate_obj.compass_process_cb[kk] == func) ||
            (compass_rate_obj.priority[kk] == priority)) {
            inv_unlock_mutex(compass_rate_obj.mutex);
            return INV_ERROR_INVALID_PARAMETER;
        }
    }

    // Make sure we have not filled up our number of allowable callbacks
    if (compass_rate_obj.num_cb <= MAX_HIGH_RATE_PROCESSES - 1) {
        kk = 0;
        if (compass_rate_obj.num_cb != 0) {
            // set kk to be where this new callback goes in the array
            while ((kk < compass_rate_obj.num_cb) &&
                   (compass_rate_obj.priority[kk] < priority)) {
                kk++;
            }
            if (kk != compass_rate_obj.num_cb) {
                // We need to move the others
                for (nn = compass_rate_obj.num_cb; nn > kk; --nn) {
                    compass_rate_obj.compass_process_cb[nn] =
                        compass_rate_obj.compass_process_cb[nn - 1];
                    compass_rate_obj.priority[nn] = compass_rate_obj.priority[nn - 1];
                }
            }
        }
        // Add new callback
        compass_rate_obj.compass_process_cb[kk] = func;
        compass_rate_obj.priority[kk] = priority;
        compass_rate_obj.num_cb++;
    } else {
        result = INV_ERROR_MEMORY_EXAUSTED;
    }

    inv_unlock_mutex(compass_rate_obj.mutex);
    return result;
}

/**
 * @internal
 * @brief   This unregisters a function to be called for each set of
 *          gyro/quaternion/rotation matrix/etc output.
 * @return  error code.
 */
inv_error_t inv_unregister_compass_rate_process(inv_error_t (*func)(struct compass_obj_t *obj))
{
    INVENSENSE_FUNC_START;
    int kk, jj;
    inv_error_t result;

    result = inv_lock_mutex(compass_rate_obj.mutex);
    if (INV_SUCCESS != result) {
        return result;
    }
    // Make sure we haven't registered this function already
    result = INV_ERROR_INVALID_PARAMETER;
    for (kk = 0; kk < compass_rate_obj.num_cb; ++kk) {
        if (compass_rate_obj.compass_process_cb[kk] == func) {
            for (jj = kk + 1; jj < compass_rate_obj.num_cb; ++jj) {
                compass_rate_obj.compass_process_cb[jj - 1] =
                    compass_rate_obj.compass_process_cb[jj];
                compass_rate_obj.priority[jj - 1] =
                    compass_rate_obj.priority[jj];
            }
            compass_rate_obj.compass_process_cb[compass_rate_obj.num_cb - 1] = NULL;
            compass_rate_obj.priority[compass_rate_obj.num_cb - 1] = 0;
            compass_rate_obj.num_cb--;
            result = INV_SUCCESS;
            break;
        }
    }

    inv_unlock_mutex(compass_rate_obj.mutex);
    return result;

}

inv_error_t inv_run_compass_rate_processes(struct inv_obj_t *inv_obj)
{
    int kk,got_data;
    inv_error_t result, result2;

    result = inv_lock_mutex(compass_rate_obj.mutex);
    if (INV_SUCCESS != result) {
        MPL_LOGE("MLOsLockMutex returned %d\n", result);
        return result;
    }

    result = inv_try_compass(&got_data);

    if (result == INV_SUCCESS && got_data == 1) {
        for (kk = 0; kk < compass_rate_obj.num_cb; ++kk) {
            if (compass_rate_obj.compass_process_cb[kk]) {
                result2 = compass_rate_obj.compass_process_cb[kk](&compass_obj);
                if (result == INV_SUCCESS)
                    result = result2;
            }
        }
    }

    inv_unlock_mutex(compass_rate_obj.mutex);
    return result;
}

/** This function will populate compass_obj.raw and compass_obj.delta_time with 
* raw compass data and sets the intial compass reading. It also sets compass_obj
* @param[out] got_data Set to 1, if function was able to set compass data, 0 if not.
*/
static inv_error_t inv_try_compass(int *got_data)
{
    inv_error_t result;
    unsigned long ctime;
    long comp_raw[3];
    int i;

    *got_data = 0;

    /* Do nothing if compass isn't present or turned off */
    if (!inv_compass_present()) {
        return INV_SUCCESS;
    }

    /* Check if time to read compass data */
    ctime = inv_get_tick_count();
    compass_obj.delta_time = ctime - compass_rate_obj.polltime;
    if ( compass_obj.delta_time < compass_rate_obj.pollrate ) {
        return INV_SUCCESS;
    }

    result = inv_get_compass_data(comp_raw);
    /* FIXME, did this ever work when error was returned? */
    if (result)
        return result;

    /* comp_raw is board units. compass_obj.raw is board units * 2^16. */
    for (i = 0; i < 3; i++) 
        compass_obj.raw[i] = (comp_raw[i] << 16);
    /* external slave wants the data even if there is an error */
    debug_rawmag(compass_obj.raw);

    compass_rate_obj.polltime = ctime;

    /* Save the intial compass value to make bias convergence faster in local
     *  body high fields */
    if (IS_INV_ADVFEATURES_ENABLED(inv_obj)) {
        if (inv_obj.adv_fusion->got_init_compass_bias == 0) {
            inv_obj.adv_fusion->got_init_compass_bias = 1;
            for (i = 0; i < 3; i++) {
                inv_obj.adv_fusion->init_compass_bias[i] = compass_obj.raw[i];
                compass_obj.init_bias[i] = compass_obj.raw[i];
            }
        }
    }

    *got_data = 1;

    return result;
}

/**  ------------- SUPERVISOR STATIC FUNCTIONS -----------   **/

static inv_error_t enable_dmp_9x_fusion(void)
{
    inv_error_t result;
    const unsigned char reg[4] = { 0x50, 0, 0, 0 };
    if (!inv_dmpkey_supported(KEY_D_2_12))
        return INV_ERROR_FEATURE_NOT_ENABLED;
    result = inv_set_mpu_memory(KEY_D_2_12, sizeof(reg), reg);
    if (result)
        LOG_RESULT_LOCATION(result);
    return result;
}

static inv_error_t disable_dmp_9x_fusion(void)
{
    inv_error_t result;
    const unsigned char reg[4] = { 0, 0, 0, 0 };
    if (!inv_dmpkey_supported(KEY_D_2_12))
        return INV_ERROR_FEATURE_NOT_ENABLED;
    result = inv_set_mpu_memory(KEY_D_2_12, sizeof(reg), reg);
    if (result)
        LOG_RESULT_LOCATION(result);
    return result;
}

static inv_error_t supervisor_update_bias(long *bias)
{
    inv_error_t result;

    if (!compass_obj.got_bias) {
        compass_obj.got_bias = true;
        result = enable_dmp_9x_fusion();
        if (result) {
            LOG_RESULT_LOCATION(result);
            return result;
        }
    }

    result = inv_set_new_compass_bias(bias);
    if (result) {
        LOG_RESULT_LOCATION(result);
        return result;
    }
    debug_bias(bias);

    compass_obj.bias[0] = bias[0];
    compass_obj.bias[1] = bias[1];
    compass_obj.bias[2] = bias[2];
    return INV_SUCCESS;
}

static inv_error_t supervisor_lost_bias(void)
{
    inv_error_t result = INV_SUCCESS;
    if (compass_obj.got_bias) {
        compass_obj.got_bias = false;
        result = disable_dmp_9x_fusion();
    }
    return result;
}

/** ------------------ USTORE DELEGATES ------------------ **/

static inv_error_t load_compass_supervisor(void)
{
    inv_error_t result1, result2;
    long loaded_bias[3];
    bool loaded_bias_valid;
    result1 = inv_uload_mem((void *)loaded_bias,
                            sizeof(loaded_bias));
    result2 = inv_uload_mem((void *)&loaded_bias_valid,
                            sizeof(loaded_bias_valid));
    if (result1)
        return result1;
    if (result2)
        return result2;
    if (loaded_bias_valid){
        result1 = supervisor_update_bias(loaded_bias);
    }
    return result1;
}

static inv_error_t store_compass_supervisor(void)
{
    inv_error_t result1, result2;
    result1 = inv_ustore_mem((const void *)compass_obj.bias,
                             sizeof(compass_obj.bias));
    result2 = inv_ustore_mem((const void *)&compass_obj.got_bias,
                             sizeof(compass_obj.got_bias));
    if (result1)
        return result1;
    return result2;
}

