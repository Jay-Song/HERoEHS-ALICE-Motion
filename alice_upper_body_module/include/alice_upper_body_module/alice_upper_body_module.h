/*
 * alice_upper_body_module.h
 *
 *  Created on: May 30, 2018
 *      Author: robotemperor
 */

#ifndef ALICE_HEROEHS_ALICE_MOTION_ALICE_UPPER_BODY_MODULE_INCLUDE_ALICE_UPPER_BODY_MODULE_H_
#define ALICE_HEROEHS_ALICE_MOTION_ALICE_UPPER_BODY_MODULE_INCLUDE_ALICE_UPPER_BODY_MODULE_H_

#include <map>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <boost/thread.hpp>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <float.h> // isnan()

#include "robotis_framework_common/motion_module.h"
//library
#include "robotis_math/robotis_math.h"
#include "heroehs_math/fifth_order_trajectory_generate.h"
#include "heroehs_math/kinematics.h"
#include "heroehs_math/end_point_to_rad_cal.h"
#include "alice_balance_control/control_function.h"
#include "alice_balance_control/alice_balance_control.h"
//#include "alice_balance_control/cop_calculation_function.h"

//message
//m - standard
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Imu.h>

//m - personal
#include "alice_msgs/BalanceParam.h"
#include "alice_msgs/ForceTorque.h"
#include "alice_msgs/FoundObjectArray.h"
#include "robotis_controller_msgs/StatusMsg.h"




namespace alice_upper_body_module
{

class UpperBodyModule: public robotis_framework::MotionModule, public robotis_framework::Singleton<UpperBodyModule>
{
public:
	UpperBodyModule();
	virtual ~UpperBodyModule();

	/* ROS Framework Functions */
	void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
	void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);

	void stop();
	bool isRunning();

	bool gazebo_check;
	double traj_time_test;
	// publisher
	//ros::Publisher  current_orientation_z_pub;

	// Subscriber
	ros::Subscriber head_test;
	ros::Subscriber waist_test;

	ros::Subscriber environment_detector_sub;
	ros::Subscriber head_moving_sub;

	ros::Subscriber ball_test_sub;
	ros::Subscriber ball_param_sub;

	ros::Subscriber walking_module_status_sub;


	void desiredPoseWaistMsgCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
	void desiredPoseHeadMsgCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
	void environmentDetectorMsgCallback(const alice_msgs::FoundObjectArray::ConstPtr& msg);
	void headMovingMsgCallback(const std_msgs::UInt8::ConstPtr& msg);


	void ballTestMsgCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
	void ballTestParamMsgCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);

	void walkingModuleStatusMsgCallback(const robotis_controller_msgs::StatusMsg::ConstPtr& msg);

	bool leg_check;

private:
	void queueThread();
	bool running_;
	int control_cycle_msec_;

	boost::thread queue_thread_;

	std::map<std::string, int> joint_name_to_id_;
	std::map<int, std::string> joint_id_to_name_;

	int new_count_;
	bool is_moving_head_;
	bool is_moving_waist_;

	double limitCheck(double calculated_value, double max, double min);
	//waist kinematics
	heroehs_math::KinematicsEulerAngle *waist_kinematics_;
	heroehs_math::CalRad *end_to_rad_waist_;     //
	Eigen::MatrixXd waist_end_point_;            // (6*8)
	Eigen::MatrixXd result_rad_waist_;           // (6*1)

	//head kinematics
	heroehs_math::KinematicsEulerAngle *head_kinematics_;
	heroehs_math::Kinematics *head_point_kinematics_;
	heroehs_math::CalRad *end_to_rad_head_;      //
	Eigen::MatrixXd head_end_point_;             // (6*8)
	Eigen::MatrixXd result_rad_head_;            // (6*1)

	//head low pass filter variables
	control_function::Filter *filter_head;
	double temp_head_roll, temp_head_pitch, temp_head_yaw;
	double temp_pre_roll, temp_pre_pitch, temp_pre_yaw;

	//algorithm
	void algorithm_process(uint8_t command_);
	void scanning_motion();
	void finding_motion();
	void tracking_function();
	uint8_t command;

	//tracking control function
	void updateBalanceParameter();
	double balance_updating_duration_sec_;
	double balance_updating_sys_time_sec_;
	bool balance_update_;
	control_function::PID_function *pidController_x;
	control_function::PID_function *pidController_y;
	heroehs_math::FifthOrderTrajectory *gain_x_p_adjustment;
	heroehs_math::FifthOrderTrajectory *gain_x_d_adjustment;
	heroehs_math::FifthOrderTrajectory *gain_y_p_adjustment;
	heroehs_math::FifthOrderTrajectory *gain_y_d_adjustment;

	double x_p_gain, x_d_gain, y_p_gain, y_d_gain;

	double current_x,current_y;
	double pre_current_x,pre_current_y;
	int frame_x, frame_y;
	double desired_x, desired_y;

	double control_angle_yaw, control_angle_pitch;
	double control_angle_yaw_temp, control_angle_pitch_temp;

	//ball detecting
	bool ball_detected;

	//motion
	double current_time_scanning;
	int motion_num_scanning;
	double current_time_finding;
	int motion_num_finding;

	//log file
	void logSaveFile();




};

}






#endif /* ALICE_HEROEHS_ALICE_MOTION_ALICE_UPPER_BODY_MODULE_INCLUDE_ALICE_UPPER_BODY_MODULE_H_ */
