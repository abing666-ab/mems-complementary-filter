#include <cmath>

#include "utility.h"

namespace complementary_filter {

void normalization(const int _dimension, float* const _vector) {
    float norm = 0.;
    for (int i = 0; i < _dimension; ++i) {
        norm += _vector[i] * _vector[i];
    }
    norm = std::sqrt(norm);
    for (int i = 0; i < _dimension; ++i) {
        _vector[i] /= norm;
    }
}

void quaternion_multiplication(const float* const _q0, const float* const _q1, float* const _q2) {
    _q2[0] = _q0[0]*_q1[0] - _q0[1]*_q1[1] - _q0[2]*_q1[2] - _q0[3] * _q1[3];
    _q2[1] = _q0[0]*_q1[1] + _q0[1]*_q1[0] + _q0[2]*_q1[3] - _q0[3] * _q1[2];
    _q2[2] = _q0[0]*_q1[2] - _q0[1]*_q1[3] + _q0[2]*_q1[0] + _q0[3] * _q1[1];
    _q2[3] = _q0[0]*_q1[3] + _q0[1]*_q1[2] - _q0[2]*_q1[1] + _q0[3] * _q1[0];
    normalization(4, _q2);
}

void lerp(const float _alpha, const float* const _q0, float* const _q1) {
    const float inv_alpha = 1. - _alpha;
    _q1[0] = inv_alpha * _q0[0] + _alpha * _q1[0];
    _q1[1] = inv_alpha * _q0[1] + _alpha * _q1[1];
    _q1[2] = inv_alpha * _q0[2] + _alpha * _q1[2];
    _q1[3] = inv_alpha * _q0[3] + _alpha * _q1[3];
    normalization(4, _q1);
}

void slerp(const float _alpha, const float* const _q0, float* const _q1) {
    const float omega = acos(_q0[0]*_q1[0] + _q0[1]*_q1[1] + _q0[2]*_q1[2] + _q0[3]*_q1[3]);
    const float A = sin((1. - _alpha) * omega) / sin(omega);
    const float B = sin(_alpha * omega) / sin(omega);
    _q1[0] = A * _q0[0] + B * _q1[0];
    _q1[1] = A * _q0[1] + B * _q1[1];
    _q1[2] = A * _q0[2] + B * _q1[2];
    _q1[3] = A * _q0[3] + B * _q1[3];
    normalization(4, _q1);
}

void rotate_vector(const float* const _vector_in, const float* const _q, float* const _vector_out) {
    _vector_out[0] = (_q[0]*_q[0] + _q[1]*_q[1] - _q[2]*_q[2] - _q[3]*_q[3])*_vector_in[0] + 2.*(_q[1]*_q[2] - _q[0]*_q[3])*_vector_in[1] + 2.*(_q[1]*_q[3] + _q[0]*_q[2])*_vector_in[2];
    _vector_out[1] = 2.*(_q[1]*_q[2] + _q[0]*_q[3])*_vector_in[0] + (_q[0]*_q[0] - _q[1]*_q[1] + _q[2]*_q[2] - _q[3]*_q[3])*_vector_in[1] + 2.*(_q[2]*_q[3] - _q[0]*_q[1])*_vector_in[2];
    _vector_out[2] = 2.*(_q[1]*_q[3] - _q[0]*_q[2])*_vector_in[0] + 2.*(_q[2]*_q[3] + _q[0]*_q[1])*_vector_in[1] + (_q[0]*_q[0] - _q[1]*_q[1] - _q[2]*_q[2] + _q[3]*_q[3])*_vector_in[2];
}

void quaternion_2_euler(const float* const _q, float* const _e) {
	const float sinp = 2. * (_q[0] * _q[2] - _q[3] * _q[1]);
	// Check that pitch is not at a singularity
	if (std::fabs(sinp) >= 1.) {
		_e[2] = 0.;

		const float R01 = 2. * (_q[1] * _q[2] - _q[0] * _q[3]);
		const float R02 = 2. * (_q[1] * _q[3] + _q[0] * _q[2]);
		if (sinp >= 0) {	// Gimbal locked down
			const float delta = std::atan2(R01, R02);
			_e[1] = M_PI_2;
			_e[0] = delta;
		} else {	// Gimbal locked up
			const float delta = std::atan2(-R01, -R02);
			_e[1] = -M_PI_2;
			_e[0] = delta;
		}
	} else {
		// Pitch, y-axis rotation.
		_e[1] = std::asin(sinp);

		// Roll, x-axis rotation.
		const float sinr_cosp = 2. * (_q[0] * _q[1] + _q[2] * _q[3]);
		const float cosr_cosp = 1. - 2. * (_q[1] * _q[1] + _q[2] * _q[2]);
		_e[0] = std::atan2(sinr_cosp / std::cos(_e[1]), cosr_cosp / std::cos(_e[1]));

		// Yaw, z-axis rotation.
		const float siny_cosp = 2. * (_q[0] * _q[3] + _q[1] * _q[2]);
		const float cosy_cosp = 1. - 2. * (_q[2] * _q[2] + _q[3] * _q[3]);
		// std::cout << "siny_cosp: " << siny_cosp << " cosy_cosp: " << cosy_cosp << std::endl;
		_e[2] = std::atan2(siny_cosp / std::cos(_e[1]), cosy_cosp / std::cos(_e[1]));
	}
}

void euler_2_quaternion(const float* const _e, float* const _q) {
    const float& roll = _e[0];
    const float& pitch = _e[1];
    const float& yaw = _e[2];

    const float&& cr = std::cos(0.5f * roll);
    const float&& sr = std::sin(0.5f * roll);
    const float&& cp = std::cos(0.5f * pitch);
    const float&& sp = std::sin(0.5f * pitch);
    const float&& cy = std::cos(0.5f * yaw);
    const float&& sy = std::sin(0.5f * yaw);

    _q[0] = cr * cp * cy + sr * sp * sy;
    _q[1] = sr * cp * cy - cr * sp * sy;
    _q[2] = cr * sp * cy + sr * cp * sy;
    _q[3] = cr * cp * sy - sr * sp * cy;
}

}