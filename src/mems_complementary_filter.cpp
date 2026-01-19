#include "mems_complementary_filter.h"
#include "utility.h"

#include <string.h>
#include <cmath>
#include <iostream>

namespace complementary_filter {

MemsComplementaryFilter::MemsComplementaryFilter(const parameters_t* const _config) {
    float zero_vector[3] = {0., 0., 0.};
    k_gravity_ = _config->gravity;
    k_angular_velocity_threshold_ = _config->angular_velocity_threshold;
    k_acceleration_threshold_ = _config->acceleration_threshold;
    k_delta_angular_velocity_threshold_ = _config->delta_angular_velocity_threshold;
    gain_acc_ = _config->gain_acc;
    gain_mag_ = _config->gain_mag;
    bias_alpha_ = _config->bias_alpha;
    do_bias_estimation_ = _config->do_bias_estimation;
    do_adaptive_gain_ = _config->do_adaptive_gain;
    use_magnetometer_ = _config->use_magnetometer;
    output_relative_yaw_ = _config->output_relative_yaw;
    initialized_ = false;
    steady_state_ = false;
    orientation_[0] = 1.f; orientation_[1] = 0.f; orientation_[2] = 0.f; orientation_[3] = 0.f;
    qw1w2_[0] = 1.f; qw1w2_[1] = 0.f; qw1w2_[2] = 0.f; qw1w2_[3] = 0.f;
    memcpy(omega_prev_, zero_vector, sizeof(zero_vector));
    memcpy(omega_bias_, _config->default_gyro_bias, sizeof(_config->default_gyro_bias));
    memcpy(acc_bias_, _config->default_acc_bias, sizeof(_config->default_acc_bias));  
}

bool MemsComplementaryFilter::Reset(const parameters_t* const _config) {
    std::unique_lock<std::mutex> lock(mutex_);

    float zero_vector[3] = {0., 0., 0.};
    gain_acc_ = _config->gain_acc;
    bias_alpha_ = _config->bias_alpha;
    do_bias_estimation_ = _config->do_bias_estimation;
    do_adaptive_gain_ = _config->do_adaptive_gain;
    use_magnetometer_ = _config->use_magnetometer;
    output_relative_yaw_ = _config->output_relative_yaw;
    initialized_ = false;
    steady_state_ = false;
    orientation_[0] = 1.f; orientation_[1] = 0.f; orientation_[2] = 0.f; orientation_[3] = 0.f;
    qw1w2_[0] = 1.f; qw1w2_[1] = 0.f; qw1w2_[2] = 0.f; qw1w2_[3] = 0.f;
    memcpy(omega_prev_, zero_vector, sizeof(zero_vector));
    memcpy(omega_bias_, _config->default_gyro_bias, sizeof(_config->default_gyro_bias));    
    return true;
}

bool MemsComplementaryFilter::AddMeasurement(const imu_sample_t* const _imu_sample) {
    std::unique_lock<std::mutex> lock(mutex_);
    const float acc[3] = {_imu_sample->ax, _imu_sample->ay, _imu_sample->az};
    const float omega[3] = {_imu_sample->wx, _imu_sample->wy, _imu_sample->wz};
    Update(_imu_sample->timestamp, acc, omega);
    return true;
}

bool MemsComplementaryFilter::AddMeasurement(const mag_sample_t* const _mag_sample) {

    return false;
}

bool MemsComplementaryFilter::AddMeasurement(const imu_sample_t* const _imu_sample, const mag_sample_t* const _mag_sample) {
    if (!use_magnetometer_) {
        return false;
    }

    std::unique_lock<std::mutex> lock(mutex_);
    const float acc[3] = {_imu_sample->ax, _imu_sample->ay, _imu_sample->az};
    const float omega[3] = {_imu_sample->wx, _imu_sample->wy, _imu_sample->wz};
    const float mag[3] = {_mag_sample->mx, _mag_sample->my, _mag_sample->mz};
    Update(_imu_sample->timestamp, acc, omega, mag);
    return true;
}

bool MemsComplementaryFilter::GetOrientation(quaternion_t* const _quat) {
    std::unique_lock<std::mutex> lock(mutex_);

    if (_quat == nullptr) {
        return false;
    }

    _quat->timestamp = prev_time_;

    if (output_relative_yaw_) {
        float qbw2[4] = {0.f};
        quaternion_multiplication(orientation_, qw1w2_, qbw2);
        _quat->w = qbw2[0];
        _quat->x = -qbw2[1];
        _quat->y = -qbw2[2];
        _quat->z = -qbw2[3];
    } else {
        _quat->w = orientation_[0];
        _quat->x = -orientation_[1];
        _quat->y = -orientation_[2];
        _quat->z = -orientation_[3];
    }

    return true;
}

bool MemsComplementaryFilter::GetBiases(filter_biases_t* const _biases) {
    std::unique_lock<std::mutex> lock(mutex_);

    if (_biases == nullptr) {
        return false;
    }
    
    _biases->timestamp = prev_time_;
    _biases->bax = acc_bias_[0];
    _biases->bay = acc_bias_[1];
    _biases->baz = acc_bias_[2];
    _biases->bwx = omega_bias_[0];
    _biases->bwy = omega_bias_[1];
    _biases->bwz = omega_bias_[2];

    return true;
}

void MemsComplementaryFilter::SetInitialStates(const float* const _acceleration) {
    float acc_normalized[3] = {_acceleration[0], _acceleration[1], _acceleration[2]};
    normalization(3, acc_normalized);
    
    if (acc_normalized[2] >= 0) {
        orientation_[0] = std::sqrt((acc_normalized[2] + 1.f) * 0.5f);
        orientation_[1] = -acc_normalized[1] / (2.f * orientation_[0]);
        orientation_[2] = acc_normalized[0] / (2.f * orientation_[0]);
        orientation_[3] = 0.f;
    } else {
        orientation_[1] = std::sqrt((1.f - acc_normalized[2]) * 0.5f);
        orientation_[0] = -acc_normalized[1] / (2.f * orientation_[1]);
        orientation_[2] = 0.f;
        orientation_[3] = acc_normalized[0] / (2.f * orientation_[1]);
    }
}

void MemsComplementaryFilter::SetInitialStates(const float* const _acceleration, const float* const _magnetic_field) {
    SetInitialStates(_acceleration);

    // [lx, ly, lz] is the magnetic field reading, rotated into the intermediary
    // frame by the inverse of q_acc.
    // l = R(q_acc)^-1 m
    const float& mx = _magnetic_field[0];
    const float& my = _magnetic_field[1];
    const float& mz = _magnetic_field[2];
    const float& qw = orientation_[0];
    const float& qx = orientation_[1];
    const float& qy = orientation_[2];
    const float& qz = orientation_[3];

    float lx = (qw * qw + qx * qx - qy * qy) * mx + 2.f * (qx * qy) * my - 2.f * (qw * qy) * mz;
    float ly = 2.f * (qx * qy) * mx + (qw * qw - qx * qx + qy * qy) * my + 2.f * (qw * qx) * mz;
    const float gamma = lx * lx + ly * ly;
    const float beta = std::sqrt(gamma + lx * std::sqrt(gamma));
    const float beta_minus = std::sqrt(gamma - lx * std::sqrt(gamma));

    float qw_mag, qz_mag;
    if (lx >= 0.f) {
        qw_mag = beta / (std::sqrt(2.f * gamma));
        qz_mag = ly / (std::sqrt(2.f) * beta);
    } else {
        qw_mag = ly / (std::sqrt(2.f)* beta_minus);
        qz_mag = beta_minus / (std::sqrt(2.f * gamma));
    }

    const float acc_initialized_orientation[4] = {qw, qx, qy, qz};
    float delta_orientation[4] = {qw_mag, 0.f, 0.f, qz_mag};
    normalization(4, delta_orientation);
    quaternion_multiplication(acc_initialized_orientation, delta_orientation, orientation_);
}

void MemsComplementaryFilter::Update(const int64_t& _ts, const float* const _acceleration, const float* const _angular_velocity) {
    float acceleration_unbias[3] = {_acceleration[0] - acc_bias_[0], _acceleration[1] - acc_bias_[1], _acceleration[2] - acc_bias_[2]};
    // First time or exception handling.
    if (!initialized_) {
        prev_time_ = _ts;
        SetInitialStates(acceleration_unbias);
        initialized_ = true;
        return;
    }

    // Bias estimation.
    if (do_bias_estimation_) {
        UpdateBiases(acceleration_unbias, _angular_velocity);
    }

    // Prediction.
    const float&& dt = (float)time_ns_to_s(_ts - prev_time_);
    float predicted_orientation[4];
    Prediction(dt, _angular_velocity, predicted_orientation);
    prev_time_ = _ts;

    // Correction.
    float delta_orientation[4];
    AccCorrection(acceleration_unbias, predicted_orientation, delta_orientation);

    // Fusion.
    float gain = gain_acc_;
    if (do_adaptive_gain_) {
        AdaptiveGain(gain_acc_, acceleration_unbias, &gain);
    }
    ScaleDeltaOrientation(gain,  delta_orientation);
    quaternion_multiplication(predicted_orientation, delta_orientation, orientation_);
}

void MemsComplementaryFilter::Update(const int64_t& _ts,
    const float* const _acceleration, const float* const _angular_velocity, const float* const _magnetic_field) {
    float acceleration_unbias[3] = {_acceleration[0] - acc_bias_[0], _acceleration[1] - acc_bias_[1], _acceleration[2] - acc_bias_[2]};

    // First time or exception handling.
    if (!initialized_) {
        prev_time_ = _ts;
        SetInitialStates(acceleration_unbias, _magnetic_field);
        initialized_ = true;
        if (output_relative_yaw_) {
            float initialed_qw1b[4] = {orientation_[0], -orientation_[1], -orientation_[2], -orientation_[3]};
            
            float euler[3] = {0.f};
            quaternion_2_euler(initialed_qw1b, euler);
            std::cout << "initial euler: " << euler[0] * 57.3 << "  " << euler[1] * 57.3 << "  " << euler[2] * 57.3 << std::endl;
            euler[0] = 0.f; euler[1] = 0.f;
            euler_2_quaternion(euler, qw1w2_);
        }
        return;
    }

    // Bias estimation.
    if (do_bias_estimation_) {
        UpdateBiases(acceleration_unbias, _angular_velocity);
    }

    // Prediction.
    const float&& dt = (float)time_ns_to_s(_ts - prev_time_);
    float predicted_orientation[4];
    Prediction(dt, _angular_velocity, predicted_orientation);
    prev_time_ = _ts;
    
    // Accelerometer Correction.
    float delta_orientation[4];
    AccCorrection(acceleration_unbias, predicted_orientation, delta_orientation);
    float gain = gain_acc_;
    if (do_adaptive_gain_) {
        AdaptiveGain(gain_acc_, acceleration_unbias, &gain);
    }
    ScaleDeltaOrientation(gain,  delta_orientation);
    quaternion_multiplication(predicted_orientation, delta_orientation, orientation_);

    // Magnetometer Correction.
    float acc_corrected_orientation[4];
    memcpy(acc_corrected_orientation, orientation_, sizeof(orientation_));
    MagCorrection(_magnetic_field, acc_corrected_orientation, delta_orientation);
    ScaleDeltaOrientation(gain_mag_,  delta_orientation);
    quaternion_multiplication(acc_corrected_orientation, delta_orientation, orientation_);

    // if (output_relative_yaw_) {
    //     float initialed_qw1b[4] = {orientation_[0], -orientation_[1], -orientation_[2], -orientation_[3]};
    //     float yaw = std::atan2(2.f * (initialed_qw1b[1] * initialed_qw1b[2] + initialed_qw1b[0] * initialed_qw1b[3]),
    //         1.f - 2.f * (initialed_qw1b[2] * initialed_qw1b[2] + initialed_qw1b[3] * initialed_qw1b[3]));
    //     std::cout << "current yaw: " << yaw * 57.3 << std::endl;

    //     float euler[3] = {0.f};
    //     quaternion_2_euler(initialed_qw1b, euler);
    //     std::cout << "current euler: " << euler[0] * 57.3 << "  " << euler[1] * 57.3 << "  " << euler[2] * 57.3 << std::endl;
    // }
}

bool MemsComplementaryFilter::CheckStates(const float* const _acceleraton, const float* const _angular_velocity) {
    const float acc_magnitude = std::sqrt(_acceleraton[0]*_acceleraton[0] + _acceleraton[1]*_acceleraton[1] + _acceleraton[2]*_acceleraton[2]);
    
    if (std::fabs(acc_magnitude - k_gravity_) > k_acceleration_threshold_) {
        steady_state_ = false;
        return false;
    }

    if (std::fabs(_angular_velocity[0] - omega_prev_[0]) > k_delta_angular_velocity_threshold_ ||
        std::fabs(_angular_velocity[1] - omega_prev_[1]) > k_delta_angular_velocity_threshold_ ||
        std::fabs(_angular_velocity[2] - omega_prev_[2]) > k_delta_angular_velocity_threshold_) {
        steady_state_ = false;
        return false;
    }

    if (std::fabs(_angular_velocity[0] - omega_bias_[0]) > k_angular_velocity_threshold_ ||
        std::fabs(_angular_velocity[1] - omega_bias_[1]) > k_angular_velocity_threshold_ ||
        std::fabs(_angular_velocity[2] - omega_bias_[2]) > k_angular_velocity_threshold_) {
        steady_state_ = false;
        return false;
    }

    steady_state_ = true;
    return true;
}

void MemsComplementaryFilter::UpdateBiases(const float* const _acceleraton, const float* const _angular_velocity) {
    CheckStates(_acceleraton, _angular_velocity);

    // Low-pass filter
    if (steady_state_) {
        omega_bias_[0] += bias_alpha_ * (_angular_velocity[0] - omega_bias_[0]);
        omega_bias_[1] += bias_alpha_ * (_angular_velocity[1] - omega_bias_[1]);
        omega_bias_[2] += bias_alpha_ * (_angular_velocity[2] - omega_bias_[2]);
    }

    omega_prev_[0] = _angular_velocity[0];
    omega_prev_[1] = _angular_velocity[1];
    omega_prev_[2] = _angular_velocity[2];
}

void MemsComplementaryFilter::Prediction(const float _dt, const float* const _angular_velocity, float* const _orientation) {
    const float omega_sub_bias[3] = {_angular_velocity[0] - omega_bias_[0],
        _angular_velocity[1] - omega_bias_[1], _angular_velocity[2] - omega_bias_[2]};

    // derivate(qt) = 0.5 * omega(q) * qt;
    // q(t+1) = q(t) + derive(qt) * dt;
    const float q0 =  orientation_[0];
    const float q1 =  orientation_[1];
    const float q2 =  orientation_[2];
    const float q3 =  orientation_[3];
    _orientation[0] = q0 + 0.5f * _dt * (omega_sub_bias[0] * q1 + omega_sub_bias[1] * q2 + omega_sub_bias[2] * q3);
    _orientation[1] = q1 + 0.5f * _dt * (-omega_sub_bias[0] * q0 - omega_sub_bias[1] * q3 + omega_sub_bias[2] * q2);
    _orientation[2] = q2 + 0.5f * _dt * (omega_sub_bias[0] * q3 - omega_sub_bias[1] * q0 - omega_sub_bias[2] * q1);
    _orientation[3] = q3 + 0.5f * _dt * (-omega_sub_bias[0] * q2 + omega_sub_bias[1] * q1 - omega_sub_bias[2] * q0);
    normalization(4, _orientation);
}

void MemsComplementaryFilter::AccCorrection(const float* const _acceleraton, const float* const _pred_ori, float* const _delta) {
    float acc_normalized[3] = {_acceleraton[0], _acceleraton[1], _acceleraton[2]};
    normalization(3, acc_normalized);

    float acc_earth_frame[3];
    float inv_pred_ori[4] = {_pred_ori[0], -_pred_ori[1], -_pred_ori[2], -_pred_ori[3]};
    rotate_vector(acc_normalized, inv_pred_ori, acc_earth_frame);

    // R(_delta) * [0 0 1]^t = acc_earth_frame^t, with delta_q3 = 0.
    _delta[0] = std::sqrt((acc_earth_frame[2] + 1.f) * 0.5f);
    _delta[1] = -acc_earth_frame[1] / (2.f * _delta[0]);
    _delta[2] = acc_earth_frame[0] / (2.f * _delta[0]);
    _delta[3] = 0.;
}

void MemsComplementaryFilter::MagCorrection(const float* const _magnetic_field, const float* const _ori, float* const _delta) {
    float l[3] = {0.f, 0.f, 0.f};
    float inv_ori[4] = {_ori[0], -_ori[1], -_ori[2], -_ori[3]};

    rotate_vector(_magnetic_field, inv_ori, l);
    float gamma = l[0] * l[0] + l[1] * l[1];
    float beta = std::sqrt(gamma + l[0] * std::sqrt(gamma));
    _delta[0] = beta / (std::sqrt(2.f * gamma));
    _delta[1] = 0.f;
    _delta[2] = 0.f;
    _delta[3] = l[1] / (std::sqrt(2.f) * beta);
}

void MemsComplementaryFilter::AdaptiveGain(const float _alpha, const float* const _acceleraton, float * const _adaptive_gain) {
    constexpr float err_threshold1 = 0.008f, err_threshold2 = 0.015f;
    constexpr float a = 1.f / (err_threshold1 - err_threshold2);
    constexpr float b = 1.f - a * err_threshold1;
    const float acc_magnitude = std::sqrt(_acceleraton[0]*_acceleraton[0] + _acceleraton[1]*_acceleraton[1] + _acceleraton[2]*_acceleraton[2]);
    const float error = std::fabs(acc_magnitude - k_gravity_) / k_gravity_;

    float factor;
    if (error < err_threshold1) {
        factor = 1.;
    } else if (error < err_threshold2) {
        factor = a * error + b;
    } else {
        factor = 0.;
    }
    *_adaptive_gain = factor * _alpha;
}

void MemsComplementaryFilter::ScaleDeltaOrientation(const float _gain, float* const _delta) {
    const float unit_quaternion[4] = { 1.f, 0.f, 0.f, 0.f };
    if (_delta[0] < 0.f) {
        slerp(_gain, unit_quaternion, _delta);
    } else {
        lerp(_gain, unit_quaternion, _delta);
    }
}

}

bool filter_create(const parameters_t* const _config, mems_complementary_filter_t** _filter) {
    mems_complementary_filter* filter = new complementary_filter::MemsComplementaryFilter(_config);
    *_filter = filter;
    return true;
}