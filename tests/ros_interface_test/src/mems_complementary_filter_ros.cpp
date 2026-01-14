#include <chrono>
#include <iostream>

#include "mems_complementary_filter_ros.h"

inline void quaternion_2_euler(const quaternion_t* const _q, Eigen::Vector3f& _e) {
	const double sinp = 2. * (_q->w * _q->y - _q->z * _q->x);
	// Check that pitch is not at a singularity
	if (fabs(sinp) >= 1.) {
		_e[2] = 0.;

		const double R01 = 2. * (_q->x * _q->y - _q->w * _q->z);
		const double R02 = 2. * (_q->x * _q->z + _q->w * _q->y);
		if (sinp >= 0) {	// Gimbal locked down
			const double delta = atan2(R01, R02);
			_e[1] = M_PI_2;
			_e[0] = delta;
		} else {	// Gimbal locked up
			const double delta = atan2(-R01, -R02);
			_e[1] = -M_PI_2;
			_e[0] = delta;
		}
	} else {
		// Pitch, y-axis rotation.
		_e[1] = asin(sinp);

		// Roll, x-axis rotation.
		const double sinr_cosp = 2. * (_q->w * _q->x + _q->y * _q->z);
		const double cosr_cosp = 1. - 2. * (_q->x * _q->x + _q->y * _q->y);
		_e[0] = atan2(sinr_cosp / cos(_e[1]), cosr_cosp / cos(_e[1]));

		// Yaw, z-axis rotation.
		const double siny_cosp = 2. * (_q->w * _q->z + _q->x * _q->y);
		const double cosy_cosp = 1. - 2. * (_q->y * _q->y + _q->z * _q->z);
		// std::cout << "siny_cosp: " << siny_cosp << " cosy_cosp: " << cosy_cosp << std::endl;
		_e[2] = atan2(siny_cosp / cos(_e[1]), cosy_cosp / cos(_e[1]));
	}
}

MemsComplementaryFilterROS::MemsComplementaryFilterROS(const ros::NodeHandle& _nh, const ros::NodeHandle& _nh_private) :
	nh_(_nh),
	nh_private_(_nh_private),
	gain_acc_(0.01),
    gain_mag_(0.01),
	bias_alpha_(0.01),
	do_bias_estimation_(true),
	do_adaptive_gain_(true),
    use_magnetometer_(false),
	publish_tf_(false),
	supervise_states_(false),
	fixed_frame_("odom"),
	initialized_filter_(false),
	pub_states_cnt_(0),
	p_filter_(nullptr),
	transform_(Eigen::Quaterniond::Identity()) {
	
	ImportParameters();

	const parameters_t parameters {
		.gravity = 9.81f,
		.angular_velocity_threshold = 0.02f,
		.acceleration_threshold = 0.15f,
		.delta_angular_velocity_threshold = 0.01f,
		.gain_acc = gain_acc_,
        .gain_mag = gain_mag_,
		.bias_alpha = bias_alpha_,
		// .default_gyro_bias = {0.00236651, -0.00196377, -4.22027e-05},
		// .default_acc_bias = {0.0549468, 0.511661, 0.014574},
		.default_gyro_bias = {0.f, 0.f, 0.f},
		.default_acc_bias = {0.01f, 0.f, -0.02f},
		.do_bias_estimation = do_bias_estimation_,
		.do_adaptive_gain = do_adaptive_gain_,
        .use_magnetometer = use_magnetometer_,
        .output_relative_yaw = output_relative_yaw_
	};

	filter_create(&parameters, &p_filter_);

	orientation_publisher_ = nh_.advertise<sensor_msgs::Imu>(ros::names::resolve("imu") + "/data", 10);
	// imu_subscriber_ = nh_.subscribe(ros::names::resolve("imu") + "/data_raw", 10, &MemsComplementaryFilterROS::IMUcallback, this);
    imu_subscriber_.reset(new ImuSubscriber(nh_, ros::names::resolve("imu") + "/data_raw", 10));
    if (use_magnetometer_) {
        mag_subscriber_.reset(new MagSubscriber(nh_, ros::names::resolve("imu") + "/mag", 10));
        sync_.reset(new Synchronizer(SyncPolicy(10), *imu_subscriber_, *mag_subscriber_));
        sync_->registerCallback(boost::bind(&MemsComplementaryFilterROS::IMUmAGcallback, this, _1, _2));
    } else {
        imu_subscriber_->registerCallback(&MemsComplementaryFilterROS::IMUcallback, this);        
    }
}

MemsComplementaryFilterROS::~MemsComplementaryFilterROS() {
    filter_destroy(p_filter_);
}

void MemsComplementaryFilterROS::ImportParameters() {
    if (!nh_private_.getParam("fixed_frame", fixed_frame_)) {
        fixed_frame_ = "odom";
    }
    if (!nh_private_.getParam("publish_tf", publish_tf_)) {
        publish_tf_ = false;
    }
    if (!nh_private_.getParam("supervise_states", supervise_states_)) {
        supervise_states_ = false;
    }
    if (!nh_private_.getParam("gain_acc", gain_acc_)) {
        gain_acc_ = 0.01f;
    }
    if (!nh_private_.getParam("gain_mag", gain_mag_)) {
        gain_acc_ = 0.01f;
    }
    if (!nh_private_.getParam("bias_alpha", bias_alpha_)) {
        bias_alpha_ = 0.01f;
    }
    if (!nh_private_.getParam("do_bias_estimation", do_bias_estimation_)) {
        do_bias_estimation_ = true;
    }
    if (!nh_private_.getParam("do_adaptive_gain", do_adaptive_gain_)) {
        do_adaptive_gain_ = true;
    }
    if (!nh_private_.getParam("do_adaptive_gain", do_adaptive_gain_)) {
        do_adaptive_gain_ = true;
    }
    if (!nh_private_.getParam("use_magnetometer", use_magnetometer_)) {
        use_magnetometer_ = false;
    }
    if (!nh_private_.getParam("output_relative_yaw", output_relative_yaw_)) {
        output_relative_yaw_ = false;
    }

    {
        float x, y, z;
        if (!nh_private_.getParam("transform_x_axisd", x)) {
            x = 0.f;
        }
        if (!nh_private_.getParam("transform_y_axisd", y)) {
            y = 0.f;
        }
        if (!nh_private_.getParam("transform_z_axisd", z)) {
            z = 0.f;
        }
        Eigen::AngleAxisf rotation_x(x/180.*M_PI, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf rotation_y(y/180.*M_PI, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf rotation_z(z/180.*M_PI, Eigen::Vector3f::UnitZ());
        Eigen::Matrix3f rotation_matrix = rotation_z.toRotationMatrix() * rotation_y.toRotationMatrix() * rotation_x.toRotationMatrix();
        transform_ = Eigen::Quaternionf(rotation_matrix);
    }
}

void MemsComplementaryFilterROS::IMUcallback(const sensor_msgs::Imu& _imu_data) {
    const auto& timestamp = _imu_data.header.stamp;
    const auto& acceleration = _imu_data.linear_acceleration;
    const auto& angular_velocity = _imu_data.angular_velocity;

    const imu_sample_t measurements {
        .timestamp = timestamp.toNSec(),
        .ax = acceleration.x,
        .ay = acceleration.y,
        .az = acceleration.z,
        .wx = angular_velocity.x,
        .wy = angular_velocity.y,
        .wz = angular_velocity.z
    };

    // const imu_sample_t measurements {
    //     .timestamp = timestamp.toNSec(),
    //     .ax = acceleration.x,
    //     .ay = acceleration.z,
    //     .az = -acceleration.y,
    //     .wx = angular_velocity.x,
    //     .wy = -angular_velocity.z,
    //     .wz = angular_velocity.y
    // };

    // std::chrono::steady_clock::time_point ts = std::chrono::steady_clock::now();
    filter_add_imu_measurement(p_filter_, &measurements);
    // std::chrono::steady_clock::time_point te = std::chrono::steady_clock::now();
    // auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(te - ts);
    // std::cout << "update time used: " << time_used.count() << " s." << std::endl;

    quaternion_t quat;
    if (filter_get_orientation(p_filter_, &quat)) {
        PublishOrientation(_imu_data, quat);
        Eigen::Vector3f euler(0.f, 0.f, 0.f);
        quaternion_2_euler(&quat, euler);
        // std::cout << "get e: " << euler[0] << " " << euler[1] << " " << euler[2] << std::endl;
    }

    // get_complementary_filter_orientation_euler(filter_, &angles);

    // if (supervise_states_) {
        // publish_states();
    // }    
}

void MemsComplementaryFilterROS::IMUmAGcallback(const ImuMsg::ConstPtr& _imu_msg, const MagMsg::ConstPtr& _mag_msg) {
    const auto& timestamp = _imu_msg->header.stamp;
    const auto& acceleration = _imu_msg->linear_acceleration;
    const auto& angular_velocity = _imu_msg->angular_velocity;
    const auto& magnetic_field = _mag_msg->magnetic_field;

    const imu_sample_t imu_measurements {
        .timestamp = timestamp.toNSec(),
        .ax = acceleration.x,
        .ay = acceleration.y,
        .az = acceleration.z,
        .wx = angular_velocity.x,
        .wy = angular_velocity.y,
        .wz = angular_velocity.z
    };
    
    const mag_sample_t mag_measurements {
        .timestamp = timestamp.toNSec(),
        .mx = magnetic_field.x,
        .my = magnetic_field.y,
        .mz = magnetic_field.z
    };

    // const imu_sample_t imu_measurements {
    //     .timestamp = timestamp.toNSec(),
    //     .ax = acceleration.x,
    //     .ay = acceleration.z,
    //     .az = -acceleration.y,
    //     .wx = angular_velocity.x,
    //     .wy = -angular_velocity.z,
    //     .wz = angular_velocity.y
    // };

    // const mag_sample_t mag_measurements {
    //     .timestamp = timestamp.toNSec(),
    //     .mx = magnetic_field.x,
    //     .my = magnetic_field.z,
    //     .mz = -magnetic_field.y
    // };

    filter_add_measurement(p_filter_, &imu_measurements, &mag_measurements);

    quaternion_t quat;
    if (filter_get_orientation(p_filter_, &quat)) {
        PublishOrientation(*_imu_msg, quat);
        Eigen::Vector3f euler(0.f, 0.f, 0.f);
        quaternion_2_euler(&quat, euler);
        // std::cout << "get e: " << euler[0] << " " << euler[1] << " " << euler[2] << std::endl;
    }
}

void MemsComplementaryFilterROS::PublishOrientation(const sensor_msgs::Imu& _imu_data, const quaternion_t& _orientation) {
	auto Qbw = Eigen::Quaternionf(_orientation.w, _orientation.x, _orientation.y, _orientation.z);
	Qbw = Qbw * transform_.inverse();
	auto qr = HamiltonToTFQuaternion(Qbw);
	auto result = _imu_data;
    tf::quaternionTFToMsg(qr, result.orientation);
    orientation_publisher_.publish(result);
}

inline tf::Quaternion MemsComplementaryFilterROS::HamiltonToTFQuaternion(const quaternion_t& _orientation) const {
	return tf::Quaternion(_orientation.x, _orientation.y, _orientation.z, _orientation.w);
}

inline tf::Quaternion MemsComplementaryFilterROS::HamiltonToTFQuaternion(const Eigen::Quaternionf& _orientation) const {
	return tf::Quaternion(_orientation.x(), _orientation.y(), _orientation.z(), _orientation.w());
}