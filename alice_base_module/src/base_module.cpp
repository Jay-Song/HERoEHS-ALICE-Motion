/*
 * base_module.cpp
 *
 *  Created on: May 16, 2018
 *      Author: robotemperor
 */

/*
 * base_module.cpp
 *
 *  Created on: 2017. 10. 14.
 *      Author: robotemperor
 */
#include <stdio.h>
#include "alice_base_module/base_module.h"
using namespace alice;

BaseModule::BaseModule()
: control_cycle_msec_(8)
{
	running_ = false;
	enable_       = false;
	gazebo_check  = false;
	module_name_  = "base_module";
	control_mode_ = robotis_framework::PositionControl;


	// Dynamixel initialize ////
	/*
	result_["l_shoulder_pitch"] = new robotis_framework::DynamixelState();  // joint 1
	result_["r_shoulder_pitch"] = new robotis_framework::DynamixelState();  // joint 2
	result_["l_shoulder_roll"]  = new robotis_framework::DynamixelState();  // joint 3

	result_["r_shoulder_roll"]  = new robotis_framework::DynamixelState();  // joint 4
	result_["l_elbow_pitch"]    = new robotis_framework::DynamixelState();  // joint 5
	result_["r_elbow_pitch"]    = new robotis_framework::DynamixelState();  // joint 6

	result_["waist_yaw"]        = new robotis_framework::DynamixelState();  // joint 9
	result_["waist_roll"]       = new robotis_framework::DynamixelState();  // joint 10

	result_["l_hip_pitch"]      = new robotis_framework::DynamixelState();  // joint 11
	result_["l_hip_roll"]       = new robotis_framework::DynamixelState();  // joint 13
	result_["l_hip_yaw"]        = new robotis_framework::DynamixelState();  // joint 15
	result_["l_knee_pitch"]     = new robotis_framework::DynamixelState();  // joint 17
	result_["l_ankle_pitch"]    = new robotis_framework::DynamixelState();  // joint 19
	result_["l_ankle_roll"]     = new robotis_framework::DynamixelState();  // joint 21

	result_["r_hip_pitch"]      = new robotis_framework::DynamixelState();  // joint 12
	result_["r_hip_roll"]       = new robotis_framework::DynamixelState();  // joint 14
	result_["r_hip_yaw"]        = new robotis_framework::DynamixelState();  // joint 16
	result_["r_knee_pitch"]     = new robotis_framework::DynamixelState();  // joint 18
	result_["r_ankle_pitch"]    = new robotis_framework::DynamixelState();  // joint 20
	result_["r_ankle_roll"]     = new robotis_framework::DynamixelState();  // joint 22

	result_["head_yaw"]         = new robotis_framework::DynamixelState();  // joint 23
	result_["head_pitch"]       = new robotis_framework::DynamixelState();  // joint 24
	result_["head_roll"]        = new robotis_framework::DynamixelState();  // joint 25
	 */

	// TEST
	result_["l_hip_pitch"]      = new robotis_framework::DynamixelState();  // joint 11
	result_["l_hip_roll"]       = new robotis_framework::DynamixelState();  // joint 13
	result_["l_hip_yaw"]        = new robotis_framework::DynamixelState();  // joint 15
	result_["l_knee_pitch"]     = new robotis_framework::DynamixelState();  // joint 17
	result_["l_ankle_pitch"]    = new robotis_framework::DynamixelState();  // joint 19
	result_["l_ankle_roll"]     = new robotis_framework::DynamixelState();  // joint 21

	result_["r_hip_pitch"]      = new robotis_framework::DynamixelState();  // joint 12
	result_["r_hip_roll"]       = new robotis_framework::DynamixelState();  // joint 14
	result_["r_hip_yaw"]        = new robotis_framework::DynamixelState();  // joint 16
	result_["r_knee_pitch"]     = new robotis_framework::DynamixelState();  // joint 18
	result_["r_ankle_pitch"]    = new robotis_framework::DynamixelState();  // joint 20
	result_["r_ankle_roll"]     = new robotis_framework::DynamixelState();  // joint 22

	result_["waist_yaw"]        = new robotis_framework::DynamixelState();  // joint 9
	//result_["waist_pitch"]       = new robotis_framework::DynamixelState();  // joint 10

//	result_["head_yaw"]         = new robotis_framework::DynamixelState();  // joint 7
//	result_["head_pitch"]       = new robotis_framework::DynamixelState();  // joint 8

	//result_["l_shoulder_pitch"] = new robotis_framework::DynamixelState();  // joint 1
	//result_["r_shoulder_pitch"] = new robotis_framework::DynamixelState();  // joint 2
	//result_["l_shoulder_roll"]  = new robotis_framework::DynamixelState();  // joint 3

	//result_["r_shoulder_roll"]  = new robotis_framework::DynamixelState();  // joint 4
	//result_["l_elbow_pitch"]    = new robotis_framework::DynamixelState();  // joint 5
	//result_["r_elbow_pitch"]    = new robotis_framework::DynamixelState();  // joint 6


	//init
	new_count_ = 1;
	is_moving_state = false;
	mov_time_state = 0;

	joint_name_to_ini_pose_state_.clear();
	joint_name_to_ini_pose_goal_.clear();
	joint_name_to_id_.clear();
	joint_id_to_name_.clear();
}
BaseModule::~BaseModule()
{
	queue_thread_.join();
}
void BaseModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
	control_cycle_msec_ = control_cycle_msec;
	queue_thread_ = boost::thread(boost::bind(&BaseModule::queueThread, this));

	for (std::map<std::string, robotis_framework::Dynamixel*>::iterator it = robot->dxls_.begin();
			it != robot->dxls_.end(); it++)
	{
		std::string joint_name = it->first;
		robotis_framework::Dynamixel* dxl_info = it->second;

		joint_name_to_id_[joint_name] = dxl_info->id_;
		joint_id_to_name_[dxl_info->id_] = joint_name;
		motion_trajectory[joint_name] = new heroehs_math::FifthOrderTrajectory();
	}
	ROS_INFO("< -------  Initialize Module : Base Module !!  ------->");
}

void BaseModule::parse_init_pose_data_(const std::string &path)
{
	YAML::Node doc; // YAML file class 선언!
	try
	{
		// load yaml
		doc = YAML::LoadFile(path.c_str()); // 파일 경로를 입력하여 파일을 로드 한다.

	}catch(const std::exception& e) // 에러 점검
	{
		ROS_ERROR("Fail to load yaml file!");
		return;
	}
	double mov_time_ = 0.0;

	mov_time_= doc["mov_time"].as<double>(); // YAML 에 string "mov_time"을 읽어온다.
	mov_time_state = mov_time_;

	YAML::Node tar_pose_node = doc["tar_pose"];// YAML 에 string "tar_pose"을 읽어온다.
	for (YAML::iterator it = tar_pose_node.begin(); it != tar_pose_node.end(); ++it) //tar_pose_node 벡터의 처음과 시작 끝까지 for 문으로 반복한다.
	{
		int id;
		double value;
		// 한 줄에서 int 와 double 을 분리한다.
		id = it->first.as<int>();
		value = it->second.as<double>();
		joint_name_to_ini_pose_goal_[joint_id_to_name_[id]] = value * DEGREE2RADIAN; // YAML에 로드된 초기 포즈 값을 라디안으로 바꾸고, eigen matrix 에 id 개수만큼 열을 생성한다.
	}
}
void BaseModule::parse_init_offset_pose_data_(const std::string &path, const std::string &data)
{
	YAML::Node doc; // YAML file class 선언!
	try
	{
		// load yaml
		doc = YAML::LoadFile(path.c_str()); // 파일 경로를 입력하여 파일을 로드 한다.

	}catch(const std::exception& e) // 에러 점검
	{
		ROS_ERROR("Fail to load yaml file!");
		return;
	}

	mov_time_state = 4.0;

	YAML::Node tar_pose_node;
	if(data.compare("init_offset_pose") == 0) // offset 이  0 이여야 한다.
	{
		tar_pose_node = doc["offset"];// YAML 에 string "tar_pose"을 읽어온다.
	}
	if(data.compare("init_offset_pose_zero") == 0) // offset 이  0 이여야 한다.
	{
		tar_pose_node = doc["init_pose_for_offset_tuner"];// YAML 에 string "tar_pose"을 읽어온다.
	}
	for (YAML::iterator it = tar_pose_node.begin(); it != tar_pose_node.end(); ++it) //tar_pose_node 벡터의 처음과 시작 끝까지 for 문으로 반복한다.
	{
		std::string joint_name = it->first.as<std::string>();
		double value;
		// 한 줄에서 int 와 double 을 분리한다.
		value = it->second.as<double>();
		joint_name_to_ini_pose_goal_[joint_name] = value; //
	}
}
void BaseModule::initPoseMsgCallback(const std_msgs::String::ConstPtr& msg) // GUI 에서 init pose topic을 sub 받아 실
{
	std::string init_pose_path;// 로스 패키지에서 YAML파일의 경로를 읽어온다.
	printf("%s", msg->data.c_str());
	if(msg->data.compare("init_pose") == 0)
	{
		init_pose_path = ros::package::getPath("alice_base_module") + "/data/init_pose.yaml";// 로스 패키지에서 YAML파일의 경로를 읽어온다.
		ROS_INFO("FILE LOAD  ::  init_pose");
	}
	else
		return;

	parse_init_pose_data_(init_pose_path); // YAML 파일 로드
	is_moving_state = true;
	ROS_INFO("FILE LOAD complete");
}
void BaseModule::queueThread()
{
	ros::NodeHandle ros_node;
	ros::CallbackQueue callback_queue;

	ros_node.setCallbackQueue(&callback_queue);
	/* subscribe topics */
	// for gui
	ros::Subscriber ini_pose_msg_sub = ros_node.subscribe("/init_pose", 5, &BaseModule::initPoseMsgCallback, this);
	ros::WallDuration duration(control_cycle_msec_ / 1000.0);

	while(ros_node.ok())
		callback_queue.callAvailable(duration);
}
bool BaseModule::isRunning()
{
	return running_;
}
void BaseModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
		std::map<std::string, double> sensors)
{
	if (enable_ == false)
	{
		return;
	}

	//// read current position ////
	if(new_count_ == 1)
	{
		ROS_INFO("Base_Process Start!");
		new_count_ ++;

		for (std::map<std::string, robotis_framework::Dynamixel*>::iterator state_iter = dxls.begin();
				state_iter != dxls.end(); state_iter++)
		{
			std::string joint_name = state_iter->first;
			if(gazebo_check == true)
				result_[joint_name]->goal_position_ = result_[joint_name]->present_position_; // 가제보 상 초기위치 0
			else
				/*if(dxls[joint_name]->direction_==1)
				{
					result_[joint_name]->goal_position_ = dxls[joint_name]->dxl_state_->present_position_; // 다이나믹셀에서 읽어옴
					joint_name_to_ini_pose_state_[joint_name] = dxls[joint_name]->dxl_state_->present_position_; // 초기위치 저장
					//printf("value == 1 ::  %s  :: %d \n", joint_name.c_str(), dxls[joint_name]->direction_);
				}

				else
				{
					result_[joint_name]->goal_position_ = dxls[joint_name]->dxl_state_->present_position_; // 다이나믹셀에서 읽어옴
					joint_name_to_ini_pose_state_[joint_name] = dxls[joint_name]->dxl_state_->present_position_; // 초기위치 저장
					//printf("value == - 1 ::  %s  :: %f \n", joint_name.c_str(), -dxls[joint_name]->dxl_state_->present_position_);
				}*/
			{
				result_[joint_name]->goal_position_ = dxls[joint_name]->dxl_state_->present_position_; // 다이나믹셀에서 읽어옴
				joint_name_to_ini_pose_state_[joint_name] = dxls[joint_name]->dxl_state_->present_position_; // 초기위치 저장
			}
			//printf("value1 ::  %s  :: %f", joint_name.c_str(), dxls[joint_name]->dxl_state_->present_position_);
		} // 등록된 다이나믹셀의 위치값을 읽어와서 goal position 으로 입력
		ROS_INFO("Base module :: Read position and Send goal position");
	}
	// trajectory is working joint space control
	if(is_moving_state == false)
	{
		ROS_INFO("Base Stay");
		for (std::map<std::string, robotis_framework::Dynamixel*>::iterator state_iter = dxls.begin();
				state_iter != dxls.end(); state_iter++)
		{
			std::string joint_name = state_iter->first;
		} // 등록된 다이나믹셀  goal position 으로 입력
	}
	else
	{
		//ROS_INFO("Base Trajectory Start");
		for (std::map<std::string, robotis_framework::Dynamixel*>::iterator state_iter = dxls.begin();
				state_iter != dxls.end(); state_iter++)
		{
			std::string joint_name = state_iter->first;
			result_[joint_name]->goal_position_ =  motion_trajectory[joint_name]->fifth_order_traj_gen(joint_name_to_ini_pose_state_[joint_name],
					joint_name_to_ini_pose_goal_[joint_name],0,0,0,0,0,mov_time_state);

			/*		if(dxls[joint_name]->direction_==1)
			{
				result_[joint_name]->goal_position_ =  motion_trajectory[joint_name]->fifth_order_traj_gen(joint_name_to_ini_pose_state_[joint_name],
						joint_name_to_ini_pose_goal_[joint_name],0,0,0,0,0,mov_time_state);
			}

			else
			{
				result_[joint_name]->goal_position_ =  -motion_trajectory[joint_name]->fifth_order_traj_gen(joint_name_to_ini_pose_state_[joint_name],
						joint_name_to_ini_pose_goal_[joint_name],0,0,0,0,0,mov_time_state);
			}*/
			is_moving_state = motion_trajectory[joint_name]->is_moving_traj;

			printf("value2 ::  %s :: %f \n", joint_name.c_str(), result_[joint_name]->goal_position_);
		} // 등록된 다이나믹셀  goal position 으로 입력
	}
}
void BaseModule::stop()
{
	return;
}
void BaseModule::go_to_init_pose(std::string data)
{

	std::string init_pose_path;// 로스 패키지에서 YAML파일의 경로를 읽어온다.
	if(data.compare("init_offset_pose") == 0 || data.compare("init_offset_pose_zero") == 0)
	{
		init_pose_path = ros::package::getPath("alice_manager") + "/config/offset.yaml";// 로스 패키지에서 YAML파일의 경로를 읽어온다.
		parse_init_offset_pose_data_(init_pose_path, data); // YAML 파일 로드
		ROS_INFO("FILE LOAD  ::  %s", data.c_str());
	}

	new_count_ = 1;
	is_moving_state = true;
	ROS_INFO("FILE LOAD complete");
}




