#pragma once

#include <stdbool.h>
#include <stdint.h>

#if defined(__cplusplus)
extern "C" {
#endif

struct mems_complementary_filter;
typedef struct mems_complementary_filter mems_complementary_filter_t;

/*!
 * IMU sample type feed into filter.
 */
typedef struct imu_sample {
    //! In nanoseconds
    int64_t timestamp;

    //! Acceleration in meters per second squared (m/s²)
    float ax, ay, az;

    //! Gyro in radians per second (rad/s)
    float wx, wy, wz;
} imu_sample_t;

/*!
 * IMU sample type feed into filter.
 */
typedef struct mag_sample {
    //! In nanoseconds
    int64_t timestamp;

    //! Magnetic in GS(GAUSS).
    float mx, my, mz;
} mag_sample_t;

/** \brief The struct of orientation. We represent it in quaternion.
 */
typedef struct quaternion {
    //! In nanoseconds
    int64_t timestamp;

    float x;              //  The x element of a quaternion.
    float y;              //  The y element of a quaternion.
    float z;              //  The z element of a quaternion.
    float w;              //  The w element of a quaternion.
} quaternion_t;

/** \brief The struct of parameters. It will be used to initialize or reset the filter.
 */
typedef struct parameters {
    float gravity;                 // Default is constant 9.81.
    float angular_velocity_threshold;  //  A threshold which decide filter's state, default is 0.005.
    float acceleration_threshold;  // Default is 0.15.
    float delta_angular_velocity_threshold; // Default is 0.01.
    float gain_acc;                //  Accelerometer gain for the complementary filter, belongs in [0, 1]. Default is 0.01.
    float gain_mag;                //  Magnetometer gain for the complementary filter, belongs in [0, 1]. Default is 0.01.
    float bias_alpha;              //  Gain of the bias estimator, belongs in [0, 1]. Default is 0.01.
    float default_gyro_bias[3];    //  The initial bias of gyroscope, which can accelerate convergence. Default is [0, 0, 0]. 
    float default_acc_bias[3];     //  The initial bias of accelerometer. Reserved yet. Default is [0, 0, 0].
    bool do_bias_estimation;       //  Whether to do bias estimation of the angular velocity (gyroscope readings) or not. Default is true.
    bool do_adaptive_gain;         //  Whether to do adaptive gain or not. Default is true.
    bool use_magnetometer;         //  Whether to use magnetometer.
    bool output_relative_yaw;      //  Output orientation relative to start yaw angle, otherwise absolute to ENU.
} parameters_t;


bool filter_create(const parameters_t* const _config, mems_complementary_filter_t** _filter);

bool filter_destroy(mems_complementary_filter_t* _filter);

bool filter_reset(const parameters_t* const _config, mems_complementary_filter_t* const _filter);

bool filter_add_imu_measurement(mems_complementary_filter_t* const _filter, const imu_sample_t* const _imu_sample);

bool filter_add_mag_measurement(mems_complementary_filter_t* const _filter, const mag_sample_t* const _mag_sample);

bool filter_add_measurement(mems_complementary_filter_t* const _filter,
                            const imu_sample_t* const _imu_sample, const mag_sample_t* const _mag_sample);

bool filter_get_orientation(mems_complementary_filter_t* const _filter, quaternion_t* const _quat);

#if defined(__cplusplus)
}
#endif