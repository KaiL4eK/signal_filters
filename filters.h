#ifndef FILTERS_H_
#define	FILTERS_H_

#include <math.h>
#include <stdbool.h>
#include <dsp.h>

extern float inv_sqrt ( float );

typedef struct {
    float   acc_x,  // Degree
            acc_y,
            acc_z,
            gyr_x,  // Degree in second
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

void filter_initialize ( float sample_rate );
void complementary_filter_set_angle_rate( float rate_a );
void complementary_filter_position_execute( imu_filter_input_t *g_a, euler_angles_t *angles );

void madgwick_filter_set_angle_rate( float beta );
void madgwick_filter_position_execute ( imu_filter_input_t *g_a, euler_angles_t *angles );
void madgwick_filter_set_inv_sqrt_method_manual( bool manual );
void madgwick_filter_reset_values( void );

void lowpass_filter_set_velocity_rate( float rate_a );
void lowpass_filter_velocity_execute ( imu_filter_input_t *g_a, euler_angles_t *velocity );


#endif	/* FILTERS_H_ */

