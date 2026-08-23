#include "attitude_filter.h"
#include <math.h>

void attitude_filter_init(attitude_filter_t *filter, float alpha) {
	filter->q.w = 1.0f;
	filter->q.x = 0.0f;
	filter->q.y = 0.0f;
	filter->q.z = 0.0f;

	filter->alpha = alpha;
}

void attitude_filter_update(attitude_filter_t *filter, const icm_20948_scaled_data_t *data, float dt) {
	// Gyro integration
	Quaternion q_dot = quat_derivative(filter->q, data->x_gyro, data->y_gyro, data->z_gyro);
	Quaternion quat_gyro = quat_add(filter->q, quat_multiply_scalar(q_dot, dt));
	quat_normalize(&quat_gyro);

	// Accel based pitch/roll
	float roll_intermediate = atan2f(data->y_accel, data->z_accel);
	float pitch_intermediate = atan2f(-data->x_accel,
	  sqrtf(data->y_accel * data->y_accel + data->z_accel * data->z_accel)
	);

	// Magnetometer tilt compensation
	float mx = data->x_magnet;
	float my = data->y_magnet;
	float mz = data->z_magnet;

	float cr_mag = cosf(roll_intermediate);
	float sr_mag = sinf(roll_intermediate);
	float cp_mag = cosf(pitch_intermediate);
	float sp_mag = sinf(pitch_intermediate);

	float mag_x_horizontal = mx * cp_mag + mz * sp_mag;
	float mag_y_horizontal = mx * sr_mag * sp_mag + my * cr_mag - mz * sr_mag * cp_mag;

	float yaw_intermediate = atan2f(-mag_y_horizontal, mag_x_horizontal);

	// Build accel/mag quaternion from roll/pitch/yaw
	float cr = cosf(roll_intermediate * 0.5f);
	float sr = sinf(roll_intermediate * 0.5f);
	float cp = cosf(pitch_intermediate * 0.5f);
	float sp = sinf(pitch_intermediate * 0.5f);
	float cy = cosf(yaw_intermediate * 0.5f);
	float sy = sinf(yaw_intermediate * 0.5f);

	Quaternion quat_accel;
	quat_accel.w = cr*cp*cy + sr*sp*sy;
	quat_accel.x = sr*cp*cy - cr*sp*sy;
	quat_accel.y = cr*sp*cy + sr*cp*sy;
	quat_accel.z = cr*cp*sy - sr*sp*cy;

	// Complementary filter
	filter->q.w = (1.0f - filter->alpha) * quat_gyro.w + filter->alpha * quat_accel.w;
	filter->q.x = (1.0f - filter->alpha) * quat_gyro.x + filter->alpha * quat_accel.x;
	filter->q.y = (1.0f - filter->alpha) * quat_gyro.y + filter->alpha * quat_accel.y;
	filter->q.z = (1.0f - filter->alpha) * quat_gyro.z + filter->alpha * quat_accel.z;

	quat_normalize(&filter->q);
}

void attitude_filter_get_euler(const attitude_filter_t *filter, float *roll, float *pitch, float *yaw) {
    const Quaternion q = filter->q;

    *roll = atan2f(2.0f * (q.w * q.x + q.y * q.z),
                    1.0f - 2.0f * (q.x * q.x + q.y * q.y));

    float sin_pitch = 2.0f * (q.w * q.y - q.z * q.x);
    if (sin_pitch > 1.0f) {
        sin_pitch = 1.0f;
    } else if (sin_pitch < -1.0f) {
        sin_pitch = -1.0f;
    }

    if (fabsf(sin_pitch) >= 1.0f) {
        *pitch = copysignf(M_PI / 2.0f, sin_pitch); // Gimbal lock
    } else {
        *pitch = asinf(sin_pitch);
    }

    *yaw = atan2f(2.0f * (q.w * q.z + q.x * q.y),
                   1.0f - 2.0f * (q.y * q.y + q.z * q.z));

    *roll  *= 180.0f / M_PI;
    *pitch *= 180.0f / M_PI;
    *yaw   *= 180.0f / M_PI;
}
