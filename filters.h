#ifndef FILTERS_H_
#define	FILTERS_H_

#include "core.h"

// gyr - [degree/sec]

typedef struct {
    float   acc_x,
            acc_y,
            acc_z,
            gyr_x,
            gyr_y,
            gyr_z;
}imu_filter_input_t;

typedef struct
{
    float w;
    float x;
    float y;
    float z;
}quaternion_t;

typedef struct
{
    float roll;
    float pitch;
    float yaw;
}euler_angles_t;

#endif	/* FILTERS_H_ */

