#include "../include/sr80_hw_interface.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sr80_hw_interface_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2);
    spinner.start();

    SR80::HardwareInterface sr80_hw_interface(nh);

    ros::waitForShutdown();

    return 0;
}