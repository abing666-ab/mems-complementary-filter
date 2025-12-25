#pragma once

#include <stdbool.h>

namespace complementary_filter {

#define U_TIME_1S_IN_NS (1000 * 1000 * 1000)

typedef int64_t time_duration_ns;

/*!
 * Convert nanoseconds duration to double seconds.
 *
 * @see timepoint_ns
 * @ingroup aux_util
 */
static inline double
time_ns_to_s(time_duration_ns ns)
{
	return (double)(ns) / (double)(U_TIME_1S_IN_NS);
}

/** \brief Normalize a vector, could be a quaternion or gravity vector.
 * \param[in] dimension Dimension of the vector which wants to be normalized.
 * \param[in out] vector The vector.
 * \author a_ji
 */
void normalization(const int _dimension, float* const _vector);

/** \brief Quaternion multiplication operator.
 * \param[in] q0 The first quaternion.
 * \param[in] q1 The second quaternion.
 * \param[out] q2 The miltiplication result.
 * \author a_ji
 */
void quaternion_multiplication(const float* const _q0, const float* const _q1, float* const _q2);

/** \brief Linear interpolation of two quaternions.
 *           result = (1 - alpha)*q0 + alpha*q1
 * \param[in] alpha The interpolation parameter.
 * \param[in] q0 The first quaternion.
 * \param[in out] q1 The second quaternion and the result.
 * \author a_ji
 */
void lerp(const float _alpha, const float* const _q0, float* const _q1);

/** \brief Spherical linear interpolation of two quaternions.
 *           cos(omega) = q0 dot q1;   
 *           result = sin((1 - alpha) * omega)/sin(omega) * q0 + sin(alpha * omega)/sin(omega) * q1
 * \param[in] alpha The interpolation parameter.
 * \param[in] q0 The first quaternion.
 * \param[in out] q1 The second quaternion and the result.
 * \author a_ji
 */
void slerp(const float _alpha, const float* const _q0, float* const _q1);

/** \brief Rotate a vector by giving rotation quaternion (quaternion must be unitary).
 * \param[in] vector_in The original vector.
 * \param[in] q The transformation.
 * \param[in out] vector_out The expected vector.
 * \author a_ji
 */
void rotate_vector(const float* const _vector_in, const float* const _q, float* const _vector_out);

/** \brief Convert a quaternion to euler angle.
 * \param[in] q The quaternion in form [w, x, y, z].
 * \param[out] e The euler angle [roll, pitch, yaw].
 * \author a_ji
 */
void quaternion_2_euler(const float* const _q, float* const _e);

/** \brief Convert an euler angle to quaternion.
 * \param[in] q The euler angle in form [roll, pitch, yaw].
 * \param[out] e The quaternion in form [w, x, y, z].
 * \author a_ji
 */
void euler_2_quaternion(const float* const _e, float* const _q);

}