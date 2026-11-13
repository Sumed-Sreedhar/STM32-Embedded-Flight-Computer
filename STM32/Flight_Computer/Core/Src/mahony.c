#include "mahony.h"
#include <math.h>

#define RAD_TO_DEG 57.2957795f
#define MAHONY_KP 1.5f

void Mahony_Init(mahony_t *filter)
{
    filter->q0 = 1.0f;
    filter->q1 = 0.0f;
    filter->q2 = 0.0f;
    filter->q3 = 0.0f;
    filter->roll_deg = 0.0f;
    filter->pitch_deg = 0.0f;
    filter->yaw_deg = 0.0f;
}

void Mahony_Update(mahony_t *filter, float gx, float gy, float gz,
                   float ax, float ay, float az, float mx, float my, float mz,
                   float dt)
{
    float norm, gravity_x, gravity_y, gravity_z;
    float magnetic_x, magnetic_y, magnetic_z;
    float field_x, field_y, field_z, horizontal_field;
    float error_x = 0.0f, error_y = 0.0f, error_z = 0.0f;
    float q0 = filter->q0, q1 = filter->q1;
    float q2 = filter->q2, q3 = filter->q3;

    if (dt <= 0.0f || dt > 0.1f) return;

    // Accelerometer measures gravity; its cross-product error corrects tilt.
    norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm > 0.01f) {
        ax /= norm;
        ay /= norm;
        az /= norm;
        gravity_x = 2.0f * (q1 * q3 - q0 * q2);
        gravity_y = 2.0f * (q0 * q1 + q2 * q3);
        gravity_z = q0*q0 - q1*q1 - q2*q2 + q3*q3;
        error_x = ay * gravity_z - az * gravity_y;
        error_y = az * gravity_x - ax * gravity_z;
        error_z = ax * gravity_y - ay * gravity_x;
    }

    // The magnetic field reference corrects heading and limits gyro yaw drift.
    norm = sqrtf(mx * mx + my * my + mz * mz);
    if (norm > 0.01f) {
        mx /= norm;
        my /= norm;
        mz /= norm;
        field_x = 2.0f * (mx*(0.5f-q2*q2-q3*q3) + my*(q1*q2-q0*q3) + mz*(q1*q3+q0*q2));
        field_y = 2.0f * (mx*(q1*q2+q0*q3) + my*(0.5f-q1*q1-q3*q3) + mz*(q2*q3-q0*q1));
        horizontal_field = sqrtf(field_x * field_x + field_y * field_y);
        field_z = 2.0f * (mx*(q1*q3-q0*q2) + my*(q2*q3+q0*q1) + mz*(0.5f-q1*q1-q2*q2));
        magnetic_x = 2.0f * (horizontal_field*(0.5f-q2*q2-q3*q3) + field_z*(q1*q3-q0*q2));
        magnetic_y = 2.0f * (horizontal_field*(q1*q2-q0*q3) + field_z*(q0*q1+q2*q3));
        magnetic_z = 2.0f * (horizontal_field*(q0*q2+q1*q3) + field_z*(0.5f-q1*q1-q2*q2));
        error_x += my*magnetic_z - mz*magnetic_y;
        error_y += mz*magnetic_x - mx*magnetic_z;
        error_z += mx*magnetic_y - my*magnetic_x;
    }

    gx = gx / RAD_TO_DEG + MAHONY_KP * error_x;
    gy = gy / RAD_TO_DEG + MAHONY_KP * error_y;
    gz = gz / RAD_TO_DEG + MAHONY_KP * error_z;

    filter->q0 += (-q1*gx - q2*gy - q3*gz) * 0.5f * dt;
    filter->q1 += ( q0*gx + q2*gz - q3*gy) * 0.5f * dt;
    filter->q2 += ( q0*gy - q1*gz + q3*gx) * 0.5f * dt;
    filter->q3 += ( q0*gz + q1*gy - q2*gx) * 0.5f * dt;

    norm = sqrtf(filter->q0*filter->q0 + filter->q1*filter->q1 +
                 filter->q2*filter->q2 + filter->q3*filter->q3);
    if (norm < 0.001f) {
        Mahony_Init(filter);
        return;
    }
    filter->q0 /= norm;
    filter->q1 /= norm;
    filter->q2 /= norm;
    filter->q3 /= norm;

    filter->roll_deg = atan2f(2.0f*(filter->q0*filter->q1 + filter->q2*filter->q3),
                              1.0f-2.0f*(filter->q1*filter->q1+filter->q2*filter->q2)) * RAD_TO_DEG;
    filter->pitch_deg = asinf(fmaxf(-1.0f, fminf(1.0f,
                              2.0f*(filter->q0*filter->q2-filter->q3*filter->q1)))) * RAD_TO_DEG;
    filter->yaw_deg = atan2f(2.0f*(filter->q0*filter->q3+filter->q1*filter->q2),
                             1.0f-2.0f*(filter->q2*filter->q2+filter->q3*filter->q3)) * RAD_TO_DEG;
}
