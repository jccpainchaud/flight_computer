#ifndef INC_ATTITUDE_FILTER_H_
#define INC_ATTITUDE_FILTER_H_

#include "icm_20948.h"
#include "quaternion.h"

typedef struct {
	Quaternion q;
	float alpha;
} attitude_filter_t;

void attitude_filter_init(attitude_filter_t *filter, float alpha);
void attitude_filter_update(attitude_filter_t *filter, const icm_20948_scaled_data_t *data, float dt);
void attitude_filter_get_euler(const attitude_filter_t *filter, float *roll, float *pitch, float *yaw);

#endif /* INC_ATTITUDE_FILTER_H_ */
