// Copyright (c) 2019, HIT-E304 CORPORATION. All rights reserved.
// Full license terms provided in LICENSE.md file.

/**
    px4_controller ROS node. Implements simple waypoint based controller for PX4/Mavros flightstack.
    It accepts input from either RC controllers (Radio) or from Auto motion planner that decides
    what direction the drone should fly. Once control signal is received it sets a waypoint at the
    right distance in correct direction. Also, allows finer grain controls over drone position.
    Authors/maintainers: Xidi Xue, Fulin Song, Dongsheng Zhang, Wenhao Mo
*/
 
#include "mode_test_pkg/px4Lib.h"


int main(int argc, char **argv)
{
	ros::init(argc, argv, "offb_node");
	ros::NodeHandle nh("~"); // it is a node
	
	px4Core::px4_Core px4_contrller;

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

