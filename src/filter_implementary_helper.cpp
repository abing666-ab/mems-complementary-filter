#include "filter_implementary_helper.h"

bool filter_destroy(mems_complementary_filter_t* _filter) {
    delete _filter;
    _filter = nullptr;
    return true;
}

bool filter_reset(const parameters_t* const _config, mems_complementary_filter_t* const _filter) {
    return _filter->Reset(_config);
}

bool filter_add_imu_measurement(mems_complementary_filter_t* const _filter, const imu_sample_t* const _imu_sample) {
    return _filter->AddMeasurement(_imu_sample);
}

bool filter_add_mag_measurement(mems_complementary_filter_t* const _filter, const mag_sample_t* const _mag_sample) {
    return false;
}

bool filter_add_measurement(mems_complementary_filter_t* const _filter,
                            const imu_sample_t* const _imu_sample, const mag_sample_t* const _mag_sample) {
    return _filter->AddMeasurement(_imu_sample, _mag_sample);
}

bool filter_get_orientation(mems_complementary_filter_t* const _filter, quaternion_t* const _quat) {
    return _filter->GetOrientation(_quat);
}

bool filter_get_biases(mems_complementary_filter_t* const _filter, filter_biases_t* const _biases) {
    return _filter->GetBiases(_biases);
}