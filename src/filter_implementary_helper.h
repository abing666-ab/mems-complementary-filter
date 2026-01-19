#pragma once

#include "mems_complementary_filter_interface.h"

#if defined(__cplusplus)
extern "C" {
#endif

struct mems_complementary_filter {
    virtual ~mems_complementary_filter() = default;

    virtual bool Reset(const parameters_t* const _config) = 0;
    
    virtual bool AddMeasurement(const imu_sample_t* const _imu_sample) = 0;
    virtual bool AddMeasurement(const mag_sample_t* const _mag_sample) = 0;
    virtual bool AddMeasurement(const imu_sample_t* const _imu_sample, const mag_sample_t* const _mag_sample) = 0;

    virtual bool GetOrientation(quaternion_t* const _quat) = 0;
    virtual bool GetBiases(filter_biases_t* const _biases) = 0;
};

#if defined(__cplusplus)
}
#endif