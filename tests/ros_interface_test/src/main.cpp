#include "mems_complementary_filter_ros.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "mems_complementary_filter_ros");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    MemsComplementaryFilterROS filter(nh, nh_private);

    ros::spin();
    return 0;
}