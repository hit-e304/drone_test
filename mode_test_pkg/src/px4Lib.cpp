#include "mode_test_pkg/px4Lib.h"

namespace px4Core
{

void px4_Core::state_cb(const mavros_msgs::State::ConstPtr& msg){ // state callback
    current_state = *msg;
}

void px4_Core::RC_subCallback(const mavros_msgs::RCIn::ConstPtr& RCmsg) // radio data callback
{
	current_RC_in = *RCmsg;
}

void px4_Core::local_pos_subCallback(const geometry_msgs::PoseStamped::ConstPtr& curr_p) // local position callback
{
	current_pose = *curr_p;
}

void px4_Core::gps_raw_subCallback(const sensor_msgs::NavSatFix::ConstPtr& gps_data){ // GPS data callback
	gps_raw_fix = *gps_data;
}

void px4_Core::cam_subCallback(const opencvtest::img_pro_info::ConstPtr& cam_data){ // camera data callback
	camera_data = *cam_data;
}


// a transformation from Quaternion to Eluer angle
geometry_msgs::Vector3 px4_Core::Quaternion2Euler(const geometry_msgs::Quaternion msg)
{
    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg, quat);
 
    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll_, pitch_, yaw_;
    tf::Matrix3x3(quat).getRPY(roll_, pitch_, yaw_);
 
    // the found angles are written in a geometry_msgs::Vector3
    geometry_msgs::Vector3 rpy;
    rpy.x = roll_ ;
    rpy.y = pitch_ ;
    rpy.z = yaw_ ;

	return rpy;
}


void px4_Core::parseArguments(const ros::NodeHandle& nodeHandle){
    loop_rate = 20.0;
    wait_for_arming_sec = 30.0;

    // RC compensation input 
    rc_d_roll = 0;
    rc_d_pitch = 0;
    rc_d_yaw = 0;
    rc_d_thrust = 0;

    roll = 0;
    pitch = 0;

    // init status as init_mdoe
    status = init_mode;

    offb_flag = 0; // if it's offboard mode now, offb_flag = 1
    armed_flag = 0;// if the vehicle is armed, armed_flag = 1
    delta_pixels[0] = 0; // delta pixels in X and Z direction
    delta_pixels[1] = 0;
    kp = 0.01; // the proportional between "delta_pixels" and "delta loccal position(meters)"

    offb_set_mode.request.custom_mode = "OFFBOARD"; // set mdoe
    arm_cmd.request.value = true; // set arm positive

    pose.pose.position.x = 0 ;
    pose.pose.position.y = 0 ;
    pose.pose.position.z = 2 ;

    nodeHandle.param("spin_rate", loop_rate, 20.0f);
    ROS_ASSERT(loop_rate > 0);

}


bool px4_Core::init(ros::NodeHandle& nodeHandle){
    parseArguments(nodeHandle);

    state_sub = nodeHandle.subscribe<mavros_msgs::State>("mavros/state", 1, &px4_Core::state_cb,this);
    if(!state_sub){
        ROS_INFO("Could not subscribe to /mavros/state");
        return false;
    }

    local_position_sub = nodeHandle.subscribe("mavros/local_position/pose", 1, &px4_Core::local_pos_subCallback,this);
    if(!local_position_sub){
        ROS_INFO("Could not subscribe to /mavros/local_position/pose");
        return false;     
    }

    gps_raw_sub = nodeHandle.subscribe("/mavros/global_position/raw/fix", 1, &px4_Core::gps_raw_subCallback,this);
    if(!gps_raw_sub){
        ROS_INFO("Could not subscribe to /mavros/local_position/pose");
        return false;
    }

    RC_sub = nodeHandle.subscribe("mavros/rc/in", 1, &px4_Core::RC_subCallback,this);
    if(!RC_sub){
        ROS_INFO("Could not subscribe to /mavros/rc/in");
        return false;
    }

    cam_sub = nodeHandle.subscribe("/contours_topic", 1, &px4_Core::cam_subCallback,this);


    local_pos_pub = nodeHandle.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 5);

    arming_client = nodeHandle.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    take_off_client = nodeHandle.serviceClient<mavros_msgs::CommandLong>("mavros/cmd/takeoff");
    set_mode_client = nodeHandle.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    return true;
}


bool px4_Core::arm(){
    // The setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(loop_rate);

    //1.waiting connecting to FCU via USB;
    while(!current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("pixhawk has been connected via USB!\n");
    // waiting for GPS fixed
    while(ros::ok() && gps_raw_fix.status.status){
        ros::spinOnce();
        rate.sleep();
    }
    //3.switching to offboard mode and try to arm;
    ros::Time last_request = ros::Time::now();
    ros::Time init_start   = ros::Time::now();
    while(!(offb_flag * armed_flag) && ros::ok() && (ros::Time::now() - init_start < ros::Duration(wait_for_arming_sec))){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
                offb_flag = 1;
            }
            last_request = ros::Time::now();
        } 
        else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) && arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                    armed_flag = 1;
                    return true;
                }
                last_request = ros::Time::now();
            }
            else if(current_state.armed){
                ROS_INFO("Vehicle was already armed");
                controller_state_ = ControllerState::Armed;
                return true;
            }

        }
        local_pos_pub.publish(pose);//the setpoint publishing rate MUST be faster than 2Hz,or pxiahwk will be switched from offboard mode to RTL mode
        ros::spinOnce();
        rate.sleep();
    }

    return false;

}

void px4_Core::spin(){
    // The setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(loop_rate);

    if(controller_state_ != ControllerState::Armed)
    {
        ROS_INFO("Cannot spin in PX4Controller node! Vehicle is not armed!");
        return;
    }

    controller_state_ = ControllerState::Takeoff;

    while(ros::ok()){
        if(controller_state_ == ControllerState::Takeoff){
        	if(2 - current_pose.pose.position.z > 0.2){ // if the vehicle altitude is higher than 1.8m, then we think it's time to switching to hover mode...
                //printf("error: %f \n",2 - current_pose.pose.position.z);
                //printf("pose: %f \n",pose.pose.position.z);
            }
            else{
                controller_state_ == ControllerState::Navigating;
                ROS_INFO("switching to mission mode...");
            }
            local_pos_pub.publish(pose);
        }
        else if(controller_state_ == ControllerState::Navigating){
            // RC compensation 
            rc_d_roll = float(current_RC_in.channels.at(0)-1513) / (-840) * 5;  //invert with -840
            rc_d_pitch =  float(current_RC_in.channels.at(1)-1513) / (-840) * 5;
            rc_d_yaw =  float(current_RC_in.channels.at(3)-1514) / (-840) * 5;
            rc_d_thrust = float(current_RC_in.channels.at(2)-1513) / 840 * 5;

            yaw = Quaternion2Euler(current_pose.pose.orientation).z + rc_d_yaw;
            //printf("tg_yaw : %f \n",yaw * 180 / 3.14159265358979);	
            q.setRPY(roll, pitch, yaw);
            pose.pose.position.x = current_pose.pose.position.x + rc_d_pitch;

            pose.pose.orientation.x = q[0];
            pose.pose.orientation.y = q[1];
            pose.pose.orientation.z = q[2];
            pose.pose.orientation.w = q[3];

            if(!camera_data.out_flag){ // if the box is in camera sight
                delta_pixels[0] = camera_data.x_pos - picture_centerX;
                delta_pixels[1] = camera_data.y_pos - picture_centerY;
                pose.pose.position.y = current_pose.pose.position.y +  (-kp * delta_pixels[0]) + rc_d_roll;
                pose.pose.position.z = current_pose.pose.position.z +  (kp * delta_pixels[1])+ rc_d_thrust;
                printf("out_flag:  %d \n",camera_data.out_flag);
                printf("error_y: %f , error_z: %f \n",(-kp * delta_pixels[0]) + rc_d_roll , (-kp * delta_pixels[1])+ rc_d_thrust);
            }
            else{
                pose.pose.position.y = pose.pose.position.y + rc_d_roll;
                pose.pose.position.z = pose.pose.position.z + rc_d_thrust;
                ROS_INFO("box is out of sight!");  // if the box is in camera sight
            }
        }
        else{
            ;
        }
        
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

}


}