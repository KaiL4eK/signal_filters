/*
 * File:   filters.c
 * Author: alex
 *
 * Created on March 14, 2017, 5:29 PM
 */

#ifndef RADIANS_TO_DEGREES
    #define RADIANS_TO_DEGREES          57.295779513f
#endif

#ifndef DEGREES_TO_RADIANS
    #define DEGREES_TO_RADIANS          0.017453293f
#endif

#include "filters.h"

static float m_sample_rate                      = 0;

float (*count_inv_sqrt)( float );

void filter_initialize ( float sample_rate )
{
    m_sample_rate = sample_rate;
    count_inv_sqrt = inv_sqrt;
}

static float complementary_filter_angle_rate_a  = 0.95f;
static float complementary_filter_angle_rate_b  = 0.05f;

void complementary_filter_set_angle_rate( float rate_a )
{
    if ( rate_a >= 1.0f )
        return;
    
    complementary_filter_angle_rate_a = rate_a;
    complementary_filter_angle_rate_b = 1.0f - rate_a;
}

void complementary_filter_position_execute( imu_filter_input_t *g_a, euler_angles_t *angles )
{    
    float   accel_angle_roll    = 0;
    float   accel_angle_pitch   = 0;
    
    if ( g_a->acc_x != 0 && g_a->acc_y != 0 )
    {
        accel_angle_roll    = atan2(-g_a->acc_x, sqrt(g_a->acc_y * g_a->acc_y + g_a->acc_z * g_a->acc_z)) * RADIANS_TO_DEGREES;
        accel_angle_pitch   = atan2( g_a->acc_y, sqrt(g_a->acc_x * g_a->acc_x + g_a->acc_z * g_a->acc_z)) * RADIANS_TO_DEGREES;
    }
    
    angles->pitch   = (complementary_filter_angle_rate_a * (g_a->gyr_x * m_sample_rate + angles->pitch)) + 
                            (complementary_filter_angle_rate_b * accel_angle_pitch);
    angles->roll    = (complementary_filter_angle_rate_a * (g_a->gyr_y * m_sample_rate + angles->roll))  + 
                            (complementary_filter_angle_rate_b * accel_angle_roll);
    angles->yaw     =                                       g_a->gyr_z * m_sample_rate + angles->yaw;
}

static float lowpass_filter_velocity_rate_a = 0.7f;
static float lowpass_filter_velocity_rate_b = 0.3f;

void lowpass_filter_set_velocity_rate( float rate_a )
{
    if ( rate_a >= 1.0f )
        return;
    
    lowpass_filter_velocity_rate_a = rate_a;
    lowpass_filter_velocity_rate_b = 1.0f - rate_a;
}

void lowpass_filter_velocity_execute ( imu_filter_input_t *g_a, euler_angles_t *velocity )
{
    velocity->pitch            = lowpass_filter_velocity_rate_a * velocity->pitch + lowpass_filter_velocity_rate_b * (g_a->gyr_x);
    velocity->roll             = lowpass_filter_velocity_rate_a * velocity->roll  + lowpass_filter_velocity_rate_b * (g_a->gyr_y);
    velocity->yaw              = lowpass_filter_velocity_rate_a * velocity->yaw   + lowpass_filter_velocity_rate_b * (g_a->gyr_z);    
}

float beta = 0.1f;

void madgwick_filter_set_angle_rate( float beta_ )
{
    beta = beta_;
}

float inv_sqrt_manual( float x )
{
    return 1.0/sqrt( x );
}

void madgwick_filter_set_inv_sqrt_method_manual( bool manual )
{
    if ( manual )
        count_inv_sqrt = inv_sqrt_manual;
    else
        count_inv_sqrt = inv_sqrt;
}

volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	// quaternion of sensor frame relative to auxiliary frame

void madgwick_filter_reset_values( void )
{
    q0 = 1.0;
    q1 = q2 = q3 = 0.0;
}

void madgwick_filter_position_execute ( imu_filter_input_t *g_a, euler_angles_t *angles )
{
    float gx = g_a->gyr_x * DEGREES_TO_RADIANS;
    float gy = g_a->gyr_y * DEGREES_TO_RADIANS;
    float gz = g_a->gyr_z * DEGREES_TO_RADIANS;
    
//---------------------------------------------------------------------------------------------------
// Variable definitions
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((g_a->acc_x == 0.0f) && (g_a->acc_y == 0.0f) && (g_a->acc_z == 0.0f))) 
    {
		// Normalise accelerometer measurement
		recipNorm = count_inv_sqrt(g_a->acc_x * g_a->acc_x + g_a->acc_y * g_a->acc_y + g_a->acc_z * g_a->acc_z);
		float ax = g_a->acc_x * recipNorm;
		float ay = g_a->acc_y * recipNorm;
		float az = g_a->acc_z * recipNorm;   

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
		recipNorm = count_inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * m_sample_rate;
	q1 += qDot2 * m_sample_rate;
	q2 += qDot3 * m_sample_rate;
	q3 += qDot4 * m_sample_rate;

	// Normalise quaternion
	recipNorm = count_inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

	float sqw = q0 * q0;
	float sqx = q1 * q1;
	float sqy = q2 * q2;
	float sqz = q3 * q3;
	
	float rotxrad = atan2(2.0 * ( q2 * q3 + q1 * q0 ) , ( -sqx - sqy + sqz + sqw ));
	float rotyrad = asin(-2.0 * ( q1 * q3 - q2 * q0 ));
	float rotzrad = atan2(2.0 * ( q1 * q2 + q3 * q0 ) , (  sqx - sqy - sqz + sqw ));
	
	angles->pitch  = rotxrad * RADIANS_TO_DEGREES;
	angles->roll   = rotyrad * RADIANS_TO_DEGREES;
	angles->yaw    = rotzrad * RADIANS_TO_DEGREES;
}
