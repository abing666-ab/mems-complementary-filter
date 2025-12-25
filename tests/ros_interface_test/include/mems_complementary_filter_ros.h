#pragma once

#include "mems_complementary_filter_interface.h"

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Geometry>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>


class MemsComplementaryFilterROS {
public:
    MemsComplementaryFilterROS(const ros::NodeHandle& _nh, const ros::NodeHandle& _nh_private);
    virtual ~MemsComplementaryFilterROS();

private:
    typedef sensor_msgs::Imu ImuMsg;
    typedef sensor_msgs::MagneticField MagMsg;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu,
                                                            MagMsg>
        MySyncPolicy;
    typedef message_filters::sync_policies::ApproximateTime<ImuMsg, MagMsg>
        SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
    typedef message_filters::Subscriber<ImuMsg> ImuSubscriber;
    typedef message_filters::Subscriber<MagMsg> MagSubscriber;

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    // ros::Subscriber imu_subscriber_;
    boost::shared_ptr<Synchronizer> sync_;
    boost::shared_ptr<ImuSubscriber> imu_subscriber_;
    boost::shared_ptr<MagSubscriber> mag_subscriber_;   

    ros::Publisher orientation_publisher_;
    tf::TransformBroadcaster tf_broadcaster_;

	float gain_acc_;
    float gain_mag_;
	float bias_alpha_;
	bool do_bias_estimation_;
	bool do_adaptive_gain_;
    bool use_magnetometer_;
    bool output_relative_yaw_;
    bool publish_tf_;
    bool supervise_states_;
    std::string fixed_frame_;

    bool initialized_filter_;
    int pub_states_cnt_;
    ros::Time prev_time_;
    mems_complementary_filter_t* p_filter_;
    Eigen::Quaternionf transform_;

    void ImportParameters();
    void IMUcallback(const sensor_msgs::Imu& _imu_data);
    void IMUmAGcallback(const ImuMsg::ConstPtr& _imu_msg_raw, const MagMsg::ConstPtr& _mag_msg);
    void PublishOrientation(const sensor_msgs::Imu& _imu_data, const quaternion_t& _orientation);
    void PublishStates();
    inline tf::Quaternion HamiltonToTFQuaternion(const quaternion_t& _orientation) const;
    inline tf::Quaternion HamiltonToTFQuaternion(const Eigen::Quaternionf& _orientation) const;
};