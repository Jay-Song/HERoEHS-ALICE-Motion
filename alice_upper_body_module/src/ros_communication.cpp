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
	//result_["waist_pitch"]  = new robotis_framework::DynamixelState(); // joint 10
	result_["waist_yaw"] = new robotis_framework::DynamixelState(); // joint 9

	result_["head_pitch"]   = new robotis_framework::DynamixelState(); // joint 7
	result_["head_yaw"]   = new robotis_framework::DynamixelState(); // joint 8

	result_["l_shoulder_pitch"] = new robotis_framework::DynamixelState();  // joint 1
	result_["r_shoulder_pitch"] = new robotis_framework::DynamixelState();  // joint 2

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

	// tracking
	command = 2;
	pidController_x = new control_function::PID_function(0.008,90*DEGREE2RADIAN,-90*DEGREE2RADIAN,0,0,0);
	pidController_y = new control_function::PID_function(0.008,80*DEGREE2RADIAN,-30*DEGREE2RADIAN,0,0,0);

	control_angle_yaw = 0;
	control_angle_pitch = 0;
	control_angle_yaw_temp = 0;
	control_angle_pitch_temp = 0;
	pre_current_x = 0;
	pre_current_y = 0;
	frame_x = 672;
	frame_y = 376;
	int margin_desired_x = 0;
	int margin_desired_y = 20;
	desired_x = (frame_x/2) + margin_desired_x;
	desired_y = (frame_y/2) + margin_desired_y;
	current_x = desired_x;
	current_y = desired_y;

	//balance param
	balance_updating_duration_sec_ = 2.0;
	balance_updating_sys_time_sec_ = 0;
	balance_update_ = false;
	gain_x_p_adjustment = new heroehs_math::FifthOrderTrajectory;
	gain_x_d_adjustment = new heroehs_math::FifthOrderTrajectory;
	gain_y_p_adjustment = new heroehs_math::FifthOrderTrajectory;
	gain_y_d_adjustment = new heroehs_math::FifthOrderTrajectory;
	x_p_gain = 0;
	x_d_gain = 0;
	y_p_gain = 0;
	y_d_gain = 0;


	leg_check = 0;

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
	environment_detector_sub = ros_node.subscribe("/heroehs/environment_detector", 5, &UpperBodyModule::environmentDetectorMsgCallback, this);
	head_moving_sub = ros_node.subscribe("/heroehs/alice/head_command", 5, &UpperBodyModule::headMovingMsgCallback, this);

	// test desired pose
	head_test = ros_node.subscribe("/desired_pose_head", 5, &UpperBodyModule::desiredPoseHeadMsgCallback, this);
	waist_test = ros_node.subscribe("/desired_pose_waist", 5, &UpperBodyModule::desiredPoseWaistMsgCallback, this);

	//test ball
	ball_test_sub = ros_node.subscribe("/ball_test", 5, &UpperBodyModule::ballTestMsgCallback, this);
	ball_param_sub = ros_node.subscribe("/ball_param", 5, &UpperBodyModule::ballTestParamMsgCallback, this);


	//walking status
	walking_module_status_sub = ros_node.subscribe("/heroehs/status", 10, &UpperBodyModule::walkingModuleStatusMsgCallback, this);


	//desired_pose_all_sub = ros_node.subscribe("/desired_pose_all", 5, &UpperBodyModule::desiredPoseAllMsgCallback, this);

	ros::WallDuration duration(control_cycle_msec_ / 1000.0);
	while(ros_node.ok())
		callback_queue.callAvailable(duration);

}
// TEST /////////////////////////////////////////////
void UpperBodyModule::walkingModuleStatusMsgCallback(const robotis_controller_msgs::StatusMsg::ConstPtr& msg)  //string
{

	if(!msg->status_msg.compare("Walking_Started"))
	{
		leg_check = 1;
	}
	if(!msg->status_msg.compare("Walking_Finished"))
	{
		leg_check = 0;
	}
	printf("leg_check :: %d\n", leg_check);
}
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
void UpperBodyModule::environmentDetectorMsgCallback(const alice_msgs::FoundObjectArray::ConstPtr& msg)
{
	for(int i = 0; i < msg->length; i++)
	{
		if(!msg->data[i].name.compare("ball"))
		{
			current_x = msg->data[i].roi.x_offset + msg->data[i].roi.width/2;
			current_y = msg->data[i].roi.y_offset + msg->data[i].roi.height/2;
		}
	}

printf("X :: %f\n", current_x);
printf("Y :: %f\n", current_y);
}
void UpperBodyModule::headMovingMsgCallback(const std_msgs::UInt8::ConstPtr& msg)
{
	command = msg -> data;
	//printf("!!!!!!");
}
//test
void UpperBodyModule::ballTestMsgCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	/*	current_x = msg->data[0];
	current_y = msg->data[1];*/

}
void UpperBodyModule::ballTestParamMsgCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	balance_update_ = true;
	balance_updating_duration_sec_ = msg->data[0];
	x_p_gain = msg->data[1];
	x_d_gain = msg->data[2];
	y_p_gain = msg->data[3];
	y_d_gain = msg->data[4];

	printf("sec value ::  %f \n",balance_updating_duration_sec_ );
	printf("P value ::  %f \n",x_p_gain );
	printf("D P value ::  %f \n",x_d_gain );
}



