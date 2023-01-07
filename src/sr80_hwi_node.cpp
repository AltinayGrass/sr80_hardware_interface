#include "../include/sr80_hw_interface.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sr80_hwi_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2);
    //ROS_INFO("Debug %s", "M1");
    spinner.start();
    //ROS_INFO("Debug %s", "M2");
    SR80::HardwareInterface sr80_hw_interface(nh);
    //ROS_INFO("Debug %s", "M3");
    ros::waitForShutdown();

    return 0;
}