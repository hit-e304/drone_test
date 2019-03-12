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

<<<<<<< HEAD
	geometry_msgs::PoseStamped pose; // vehicle pose variables: position and oreintation
    pose.pose.position.x = 0 ;
    pose.pose.position.y = 0 ;
    pose.pose.position.z = 2 ;


	while(ros::ok()){
		//****tatus mechine*******************
		if(status == init_mode){
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
			while(!(offb_flag * armed_flag) && ros::ok()){
				if( current_state.mode != "OFFBOARD" &&
					(ros::Time::now() - last_request > ros::Duration(5.0))){
					if( set_mode_client.call(offb_set_mode) &&
						offb_set_mode.response.mode_sent){
						ROS_INFO("Offboard enabled");
						offb_flag = 1;
					}
					last_request = ros::Time::now();
				} 
				else {
					if( !current_state.armed &&
						(ros::Time::now() - last_request > ros::Duration(5.0))){
						if( arming_client.call(arm_cmd) &&
							arm_cmd.response.success){
							ROS_INFO("Vehicle armed");
							armed_flag = 1;
						}
						last_request = ros::Time::now();
					}
				}
				local_pos_pub.publish(pose);//the setpoint publishing rate MUST be faster than 2Hz,or pxiahwk will be switched from offboard mode to RTL mode
				ros::spinOnce();
				rate.sleep();
			}
			//3.transfer to hover_mode
			status = hover_mode;
			ROS_INFO("switching to hover mode...");
		}
		else if(status == hover_mode){
			if(2 - current_pose.pose.position.z > 0.2){ // if the vehicle altitude is higher than 1.8m, then we think it's time to switching to hover mode...
				printf("error: %f \n",2 - current_pose.pose.position.z);
				printf("pose: %f \n",pose.pose.position.z);
			}
			else{
				status = mission_mode;
				ROS_INFO("switching to mission mode...");
			}
			local_pos_pub.publish(pose);
		}
		else if(status == mission_mode){
			if(!camera_data.out_flag){ // if the box is in camera sight
				// RC compensation 
				rc_d_roll = float(current_RC_in.channels.at(0)-1513) / (-840) * 5;  //invert with -840
				rc_d_pitch =  float(current_RC_in.channels.at(1)-1513) / (-840) * 5;
				rc_d_yaw =  float(current_RC_in.channels.at(3)-1514) / (-840) * 5;
				rc_d_thrust = float(current_RC_in.channels.at(2)-1513) / 840 * 5;
				printf("rc_d_yaw : %f \n",rc_d_yaw * 180 / 3.14);


				roll = 0;
				pitch = 0;
				px4_xyzw.x = current_pose.pose.orientation.x;
				px4_xyzw.y = current_pose.pose.orientation.y;
				px4_xyzw.z = current_pose.pose.orientation.z;
				px4_xyzw.w = current_pose.pose.orientation.w;
				// printf("x_cur: %f \n",px4_xyzw.x);
				// printf("y_cur: %f \n",px4_xyzw.y);
				// printf("z_cur: %f \n",px4_xyzw.z);
				// printf("w_cur: %f \n",px4_xyzw.w);
				yaw = Quaternion2Euler(px4_xyzw).z + rc_d_yaw;
				// yaw = 30 * 3.14 / 180 + rc_d_yaw;
				printf("tg_yaw : %f \n",yaw * 180 / 3.14159265358979);	
				q.setRPY(roll, pitch, yaw);

				pose.pose.position.x = current_pose.pose.position.x + rc_d_pitch;
				// pose.pose.position.y = pose.pose.position.y + rc_d_roll ;
				// pose.pose.position.z = pose.pose.position.z + rc_d_thrust ;
				pose.pose.orientation.x = q[0];
				pose.pose.orientation.y = q[1];
				pose.pose.orientation.z = q[2];
				pose.pose.orientation.w = q[3];
				delta_pixels[0] = camera_data.x_pos - picture_centerX;
				delta_pixels[1] = camera_data.y_pos - picture_centerY;
				pose.pose.position.y = current_pose.pose.position.y +  (-kp * delta_pixels[0]) + rc_d_roll;
				pose.pose.position.z = current_pose.pose.position.z +  (kp * delta_pixels[1])+ rc_d_thrust;
				printf("out_flag:  %d \n",camera_data.out_flag);
				printf("error_y: %f , error_z: %f \n",(-kp * delta_pixels[0]) + rc_d_roll , (-kp * delta_pixels[1])+ rc_d_thrust);
			}
			else{
				rc_d_roll = float(current_RC_in.channels.at(0)-1513) / (-840) * 5;  //invert with -840
				rc_d_pitch =  float(current_RC_in.channels.at(1)-1513) / (-840) * 5;
				rc_d_yaw =  float(current_RC_in.channels.at(3)-1514) / (-840) * 5;
				rc_d_thrust = float(current_RC_in.channels.at(2)-1513) / 840 * 5;
				// printf("rc_d_yaw : %f \n",rc_d_yaw * 180 / 3.14);

				roll = 0;
				pitch = 0;
				px4_xyzw.x = current_pose.pose.orientation.x;
				px4_xyzw.y = current_pose.pose.orientation.y;
				px4_xyzw.z = current_pose.pose.orientation.z;
				px4_xyzw.w = current_pose.pose.orientation.w;
				// printf("x_cur: %f \n",px4_xyzw.x);
				// printf("y_cur: %f \n",px4_xyzw.y);
				// printf("z_cur: %f \n",px4_xyzw.z);
				// printf("w_cur: %f \n",px4_xyzw.w);
				yaw = Quaternion2Euler(px4_xyzw).z + rc_d_yaw;
				printf("tg_yaw : %f \n",yaw * 180 / 3.14159265358979);	
				q.setRPY(roll, pitch, yaw);

				pose.pose.position.x = current_pose.pose.position.x + rc_d_pitch;
				pose.pose.position.y = pose.pose.position.y + rc_d_roll ;
				pose.pose.position.z = pose.pose.position.z + rc_d_thrust ;
				pose.pose.orientation.x = q[0];
				pose.pose.orientation.y = q[1];
				pose.pose.orientation.z = q[2];
				pose.pose.orientation.w = q[3];
				ROS_INFO("box is out of sight!");  // if the box is in camera sight
			}
			printf("tg_yaw_prepub : %f \n",yaw * 180 / 3.14);
			local_pos_pub.publish(pose); // publish target position
		}
		else{
			;
		}
		
		ros::spinOnce();
		rate.sleep();
	}
=======
	if(!px4_contrller.init(nh))
    {
        ROS_ERROR("Could not initialize PX4Controller node!");
        return -1;
    }
>>>>>>> master
 
    if(!px4_contrller.arm())
    {
        ROS_ERROR("Could not arm PX4/FCU!");
        return -1;
    }

    // Loop and process commands/state
    px4_contrller.spin();
	
    return 0;
}

