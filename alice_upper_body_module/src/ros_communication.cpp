/*
 * ros_communication.cpp
 *
 *  Created on: May 30, 2018
 *      Author: robotemperor
 */

#include "alice_upper_body_module/alice_upper_body_module.h"

using namespace alice_upper_body_module;

UpperBodyModule::UpperBodyModule()
: control_cycle_msec_(8)
{
	running_ = false;
	gazebo_check = false;
	is_moving_head_ = false;
	is_moving_waist_ = false;

	enable_       = false;
	module_name_  = "upper_body_module";
	control_mode_ = robotis_framework::PositionControl;

	// Dynamixel initialize ////
	result_["waist_pitch"]  = new robotis_framework::DynamixelState(); // joint 9
	result_["waist_yaw"] = new robotis_framework::DynamixelState(); // joint 10

	result_["head_pitch"]   = new robotis_framework::DynamixelState(); // joint 7
	result_["head_yaw"]   = new robotis_framework::DynamixelState(); // joint 8

	///////////////////////////
	// motion control variables
	waist_kinematics_ = new heroehs_math::KinematicsEulerAngle;
	end_to_rad_waist_ = new heroehs_math::CalRad;

	head_kinematics_  = new heroehs_math::KinematicsEulerAngle;
	head_point_kinematics_ = new heroehs_math::Kinematics;
	end_to_rad_head_  = new heroehs_math::CalRad;

	traj_time_test = 4;
	new_count_ = 1 ;


	end_to_rad_waist_ -> cal_end_point_tra_alpha -> current_time = traj_time_test;
	end_to_rad_waist_ -> cal_end_point_tra_betta -> current_time = 0;
	end_to_rad_waist_ -> cal_end_point_tra_kamma -> current_time = traj_time_test;

	end_to_rad_head_ -> cal_end_point_tra_alpha -> current_time = traj_time_test;
	end_to_rad_head_ -> cal_end_point_tra_betta -> current_time = traj_time_test;
	end_to_rad_head_ -> cal_end_point_tra_kamma -> current_time = traj_time_test;

	end_to_rad_waist_ -> cal_end_point_tra_alpha -> final_time = traj_time_test;
	end_to_rad_waist_ -> cal_end_point_tra_betta -> final_time = 0;
	end_to_rad_waist_ -> cal_end_point_tra_kamma -> final_time = traj_time_test;

	end_to_rad_head_ -> cal_end_point_tra_alpha -> final_time = traj_time_test;
	end_to_rad_head_ -> cal_end_point_tra_betta -> final_time = traj_time_test;
	end_to_rad_head_ -> cal_end_point_tra_kamma -> final_time = traj_time_test;


	filter_head = new control_function::Filter;
	temp_head_roll  = 0;
	temp_head_pitch = 0;
	temp_head_yaw   = 0;
	temp_pre_roll = 0;
	temp_pre_pitch = 0;
	temp_pre_yaw = 0;

	command = "tracking";

}
UpperBodyModule::~UpperBodyModule()
{
	queue_thread_.join();
}
// ros message communication thread
void UpperBodyModule::queueThread()
{
	ros::NodeHandle ros_node;
	ros::CallbackQueue callback_queue;
	ros_node.setCallbackQueue(&callback_queue);
	// publish topics


	// subscribe topics

	// test desired pose
	head_test = ros_node.subscribe("/desired_pose_head", 5, &UpperBodyModule::desiredPoseHeadMsgCallback, this);
	waist_test = ros_node.subscribe("/desired_pose_waist", 5, &UpperBodyModule::desiredPoseWaistMsgCallback, this);

	//desired_pose_all_sub = ros_node.subscribe("/desired_pose_all", 5, &UpperBodyModule::desiredPoseAllMsgCallback, this);

	ros::WallDuration duration(control_cycle_msec_ / 1000.0);
	while(ros_node.ok())
		callback_queue.callAvailable(duration);

}
// TEST /////////////////////////////////////////////
void UpperBodyModule::desiredPoseWaistMsgCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	waist_end_point_(3, 1) = msg->data[0]; // yaw  트레젝토리 6 * 8 은 xyz yaw(z) pitch(y) roll(x) 이며 8은 처음 위치 나중 위치 / 속도 속도 / 가속도 가속도 / 시간 시간 / 임
	waist_end_point_(4, 1) = msg->data[1]; // pitch
	waist_end_point_(3, 7) = msg->data[2];
	waist_end_point_(4, 7) = msg->data[3];
	is_moving_waist_ = true;
}
void UpperBodyModule::desiredPoseHeadMsgCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	head_end_point_(3, 1)  = msg->data[0]; // yaw  트레젝토리 6 * 8 은 xyz yaw(z) pitch(y) roll(x) 이며 8은 처음 위치 나중 위치 / 속도 속도 / 가속도 가속도 / 시간 시간 / 임
	head_end_point_(4, 1)  = msg->data[1]; // pitch
	head_end_point_ (3, 7) = msg->data[2];
	head_end_point_ (4, 7) = msg->data[3];
	is_moving_head_ = true;
}
void UpperBodyModule::environmentDetectorMsgCallback(const std_msgs::String::ConstPtr& msg)
{

}
void UpperBodyModule::headMovingMsgCallback(const std_msgs::String::ConstPtr& msg)
{
		command = msg -> data;
}



