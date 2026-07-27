#ifndef SRC_QUATERNION_H_
#define SRC_QUATERNION_H_

typedef struct {
	float w;
	float x;
	float y;
	float z;
} Quaternion;

void quat_normalize(Quaternion *q);
Quaternion quat_multiply(Quaternion a, Quaternion b);
Quaternion quat_derivative(Quaternion q, float gx, float gy, float gz);

#endif /* SRC_QUATERNION_H_ */
