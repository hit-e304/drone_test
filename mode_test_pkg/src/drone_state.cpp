/*
功能：读取各topic的内容显示出来
*/


#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <tf/tf.h>
#include "opencvtest/contours.h"
#include <mavros_msgs/RCIn.h>
#include <sensor_msgs/NavSatFix.h>

mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;
mavros_msgs::RCIn current_RC_in;
opencvtest::img_pro_info camera_data;
sensor_msgs::NavSatFix gps_raw_fix;
geometry_msgs::PoseStamped target_pose;

void state_subCallback(const mavros_msgs::State::ConstPtr& Smsg)
{ // state callback
    current_state = *Smsg;
}

void local_pos_subCallback(const geometry_msgs::PoseStamped::ConstPtr& curr_p) // local position callback
{
	current_pose = *curr_p;
}

void RC_subCallback(const mavros_msgs::RCIn::ConstPtr& RCmsg)
{
    current_RC_in = *RCmsg;
}

void cam_subCallback(const opencvtest::img_pro_info::ConstPtr& cam_msg)
{
    camera_data = *cam_msg;
}

void gps_raw_subCallback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg)
{
    gps_raw_fix = *gps_msg;
}

void target_pose_subCallback(const geometry_msgs::PoseStamped::ConstPtr& tg_pose_msg)
{
    target_pose = *tg_pose_msg;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "drone_state");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe("mavros/state", 1, state_subCallback);
    ros::Subscriber local_position_sub = nh.subscribe("mavros/local_position/pose", 1, local_pos_subCallback);
	ros::Subscriber RC_sub = nh.subscribe("mavros/rc/in", 1, RC_subCallback);
	ros::Subscriber cam_sub = nh.subscribe("/contours_topic", 1, cam_subCallback);
	ros::Subscriber gps_raw_sub = nh.subscribe("/mavros/global_position/raw/fix", 1, gps_raw_subCallback);

    ros::Subscriber target_pose_sub = nh.subscribe("/mavros/setpoint_position/local", 1, target_pose_subCallback);
    

    ros::Rate rate(20);

    while (ros::ok())
    {
        /* code for loop body */
        ROS_INFO("The Drone's mode is :");
        std::cout << "mode : "<< current_state.mode << std::endl;
        ROS_INFO("The Drone's current pose is :");
        std::cout << current_pose.pose << std::endl;
        ROS_INFO("The Drone's target pose is :");
        std::cout << target_pose.pose << std::endl;
        ROS_INFO("The Drone's RCin is :");
        std::cout << current_RC_in << std::endl;
        ROS_INFO("The camera_data is :");
        std::cout << camera_data << std::endl;
        ROS_INFO("The GPS FIX status is :");
        std::cout << gps_raw_fix.status << std::endl;

        ros::spinOnce();
        rate.sleep();
    }
    
    
}