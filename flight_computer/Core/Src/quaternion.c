#include "quaternion.h"
#include <math.h>

void quat_normalize(Quaternion *q) {
	float len = sqrtf((q->w * q->w) + (q->x * q->x) + (q->y * q->y) + (q->z * q->z));

	if (len > 1e-6f) { // epsilon to allow for division if len is very small
		q->w /= len;
		q->x /= len;
		q->y /= len;
		q->z /= len;
	}
}

Quaternion quat_multiply(const Quaternion a, const Quaternion b) {
	Quaternion q;

	q.w = (a.w * b.w) - (a.x * b.x) - (a.y * b.y) - (a.z * b.z);
	q.x = (a.w * b.x) + (a.x * b.w) + (a.y * b.z) - (a.z * b.y);
	q.y = (a.w * b.y) - (a.x * b.z) + (a.y * b.w) + (a.z * b.x);
	q.z = (a.w * b.z) + (a.x * b.y) - (a.y * b.x) + (a.z * b.w);

	return q;
}

Quaternion quat_derivative(const Quaternion q, float gx, float gy, float gz) {
	// q = [w, x, y, z], w = [0, gx, gy, gz]
	// Expanded form of 0.5f * quat_multiply(q, w)

	Quaternion qDot;

	qDot.w = 0.5f * (-(q.x * gx) - (q.y * gy) - (q.z * gz));
	qDot.x = 0.5f * ( (q.w * gx) + (q.y * gz) - (q.z * gy));
	qDot.y = 0.5f * ( (q.w * gy) - (q.x * gz) + (q.z * gx));
	qDot.z = 0.5f * ( (q.w * gz) + (q.x * gy) - (q.y * gx));

	return qDot;
}

Quaternion quat_multiply_scalar(const Quaternion q, float scalar) {
	Quaternion new_q;

	new_q.w = q.w * scalar;
	new_q.x = q.x * scalar;
	new_q.y = q.y * scalar;
	new_q.z = q.z * scalar;

	return new_q;
}

Quaternion quat_add(const Quaternion a, const Quaternion b) {
	Quaternion sum;

	sum.w = a.w + b.w;
	sum.x = a.x + b.x;
	sum.y = a.y + b.y;
	sum.z = a.z + b.z;

	return sum;
}

Quaternion quat_sqr(const Quaternion q) {
	return quat_multiply(q, q);
}

