#pragma once

#include "filter_implementary_helper.h"

#include <mutex>

namespace complementary_filter {

class MemsComplementaryFilter final : public mems_complementary_filter {
  public:
    MemsComplementaryFilter(const parameters_t* const _config);
    virtual ~MemsComplementaryFilter() {}

    bool Reset(const parameters_t* const _config) override;

    bool AddMeasurement(const imu_sample_t* const _imu_sample) override;

    bool AddMeasurement(const mag_sample_t* const _mag_sample) override;

    bool AddMeasurement(const imu_sample_t* const _imu_sample, const mag_sample_t* const _mag_sample) override;

    bool GetOrientation(quaternion_t* const _quat) override;

  private:
    /** \brief Set the initial states of filter.
     *  \param[in] acceleraton Measurements of accelerometer.
     *  \param[in] angular_velocity Measurements of gyroscope.
     *  \author a_ji
     */    
    void SetInitialStates(const float* const _acceleration);

    void SetInitialStates(const float* const _acceleration, const float* const _magnetic_field);

    void Update(const int64_t& _ts, const float* const _acceleration, const float* const _angular_velocity);

    void Update(const int64_t& _ts, const float* const _acceleration,
                const float* const _angular_velocity, const float* const _magnetic_field);

    bool CheckStates(const float* const _acceleraton, const float* const _angular_velocity);

    void UpdateBiases(const float* const _acceleraton, const float* const _angular_velocity);

    void Prediction(const float _dt, const float* const _angular_velocity, float* const _orientation);

    void AccCorrection(const float* const _acceleraton, const float* const _pred_ori, float* const _delta);

    void MagCorrection(const float* const _magnetic_field, const float* const _ori, float* const _delta);

    void AdaptiveGain(const float _alpha, const float* const _acceleraton, float * const _adaptive_gain);

    void ScaleDeltaOrientation(const float _gain, float* const _delta);

    std::mutex mutex_;

    int64_t prev_time_;

    float k_gravity_;

    // Bias estimation steady state thresholds
    float k_angular_velocity_threshold_;
    float k_acceleration_threshold_;
    float k_delta_angular_velocity_threshold_;

    // Gain parameter for the complementary filter, belongs in [0, 1].
    float gain_acc_;

    // Gain parameter for the complementary filter, belongs in [0, 1].
    float gain_mag_;

    // Bias estimation gain parameter, belongs in [0, 1].
    float bias_alpha_;

    // Parameter whether to do bias estimation or not.
    bool do_bias_estimation_;
    // Parameter whether to do adaptive gain or not.
    bool do_adaptive_gain_;
    // Whether to use magnetometer.
    bool use_magnetometer_;
    // Output orientation relative to start yaw angle, otherwise absolute to ENU.
    bool output_relative_yaw_;

    // Filter state.
    bool initialized_;
    bool steady_state_;

    // The orientation as a Hamilton quaternion (q0 is the scalar).
    // Represents the orientation of the fixed imu body frame(qbw).
    float orientation_[4];
    float qw1w2_[4];

    // Previous angular velocities.
    float omega_prev_[3];
    // Bias in angular velocities.
    float omega_bias_[3];
    // Bias in acceleration.
    float acc_bias_[3];
};

}