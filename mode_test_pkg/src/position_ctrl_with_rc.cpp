/**
 * @file position_ctrl_with_rc.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */
 
#include "mode_test_pkg/px4Lib.h"


int main(int argc, char **argv)
{
    // std::cout << "aaaaaaaa" << std::endl;
	ros::init(argc, argv, "offb_node");
	ros::NodeHandle nh("~"); // it is a node
	// std::cout << "bbbbbbbbb" << std::endl;
	px4Core::px4_Core px4_contrller;
    // std::cout << "cccccccccc" << std::endl;
	if(!px4_contrller.init(nh))
    {
        ROS_ERROR("Could not initialize PX4Controller node!");
        return -1;
    }
 
    if(!px4_contrller.arm())
    {
        ROS_ERROR("Could not arm PX4/FCU!");
        return -1;
    }

    // Loop and process commands/state
    px4_contrller.spin();
	
    return 0;
}

