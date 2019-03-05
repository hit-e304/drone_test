#ifndef PX4LIB_H
#define PX4LIB_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <tf/tf.h>
// #include <mavros_msgs/Thrust.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/RCIn.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/CommandLong.h>
#include "opencvtest/contours.h"
#include "tf/transform_datatypes.h"

// define 4 modes
#define init_mode 0
#define hover_mode 1
#define mission_mode 2
#define rc_landing_mode 3

// picture size is 480 * 640
#define picture_centerX 0
#define picture_centerY 0

namespace px4Core
{
class px4_Core
{
private:
    // RC compensation input 
    float rc_d_roll;
    float rc_d_pitch;
    float rc_d_yaw;
    float rc_d_thrust;
    tfScalar yaw,pitch,roll; //RC oreintation
    tf::Quaternion q;

    // init status as init_mdoe
    int status;

    int offb_flag; // if it's offboard mode now, offb_flag = 1
    int armed_flag;// if the vehicle is armed, armed_flag = 1
    int delta_pixels[2]; // delta pixels in X and Z direction
    float kp; // the proportional between "delta_pixels" and "delta loccal position(meters)"

    mavros_msgs::State current_state; // show pixkawk status
    mavros_msgs::RCIn current_RC_in; // get the radio input
    geometry_msgs::PoseStamped current_pose; // pose of the vehicle
    sensor_msgs::NavSatFix gps_raw_fix; // data of gps
    opencvtest::img_pro_info camera_data; // data of box center and some other flags
    geometry_msgs::Quaternion px4_xyzw;

    ros::Subscriber state_sub;
    ros::Publisher local_pos_pub;

    ros::ServiceClient arming_client;
    ros::ServiceClient take_off_client;
    ros::ServiceClient set_mode_client;
    ros::Subscriber RC_sub;
    ros::Subscriber cam_sub;

    ros::Subscriber local_position_sub;

    ros::Subscriber gps_raw_sub;

    mavros_msgs::SetMode offb_set_mode;
    mavros_msgs::CommandBool arm_cmd;

    geometry_msgs::PoseStamped pose; // vehicle pose variables: position and oreintation


private:
    enum class ControllerState
    {
        Noop,
        Armed,
        Takeoff,
        Navigating
    };

    void state_cb(const mavros_msgs::State::ConstPtr& msg);
    void RC_subCallback(const mavros_msgs::RCIn::ConstPtr& RCmsg);
    void local_pos_subCallback(const geometry_msgs::PoseStamped::ConstPtr& curr_p);
    void gps_raw_subCallback(const sensor_msgs::NavSatFix::ConstPtr& gps_data);
    void cam_subCallback(const opencvtest::img_pro_info::ConstPtr& cam_data);

    void parseArguments(const ros::NodeHandle& nodeHandle);
    geometry_msgs::Vector3 Quaternion2Euler(const geometry_msgs::Quaternion msg);

private:
    ros::NodeHandle nh; // it is a node

    float loop_rate;
    float wait_for_arming_sec;

    ControllerState controller_state_ = ControllerState::Noop;


public:
    bool init(ros::NodeHandle& nodeHandle);
    bool arm();
    void spin();
};
}



#endif