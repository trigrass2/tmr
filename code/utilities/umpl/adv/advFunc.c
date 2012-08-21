/*
 $License:
    Copyright (C) 2011 InvenSense Corporation, All Rights Reserved.
 $
 */
#include "advFunc.h"
#include "mlMathFunc.h"
#include "mlmath.h"
#include "mldl.h"
#include "dmpKey.h"

/** @internal
 * Does the cross product of compass by gravity, then converts that
 * to the world frame using the quaternion, then computes the angle that
 * is made.
 * @param[in] compass Compass Vector (Body Frame), length 3
 * @param[in] grav Gravity Vector (Body Frame), length 3
 * @param[in] quat Quaternion, Length 4
 * @return Angle Cross Product makes after quaternion rotation.
 */
float inv_compass_angle(const long *compass, const long *grav, const float *quat)
{
    float cgcross[4],q1[4],q2[4],qi[4];
    float angW;

    // Compass cross Gravity
    cgcross[0] = 0.f;
    cgcross[1] = (float)compass[1] * grav[2] - (float)compass[2] * grav[1];
    cgcross[2] = (float)compass[2] * grav[0] - (float)compass[0] * grav[2];
    cgcross[3] = (float)compass[0] * grav[1] - (float)compass[1] * grav[0];
 
    // Now convert cross product into world frame
    inv_q_multf(quat,cgcross,q1);
    inv_q_invertf(quat,qi);
    inv_q_multf(q1,qi,q2);

    // This is the unfiltered heading correction
    angW = -atan2f(q2[2],q2[1]);
    return angW;
}

inv_error_t inv_set_dmp_quaternion(long *q)
{
    inv_error_t result = INV_SUCCESS;
    unsigned char reg[16];

    inv_int32_to_big8(q[0],reg);
    inv_int32_to_big8(q[1],&reg[4]);
    inv_int32_to_big8(q[2],&reg[8]);
    inv_int32_to_big8(q[3],&reg[12]);
    if (inv_dmpkey_supported(KEY_D_0_192))
        result = inv_set_mpu_memory(KEY_D_0_192,16,reg);
    return result;
}
