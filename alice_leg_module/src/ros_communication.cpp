/*
 * ros_communication.cpp
 *
 *  Created on: May 27, 2018
 *      Author: robotemperor
 */
#include <stdio.h>
#include "alice_leg_module/alice_leg_module.h"

using namespace alice_leg_module;

AliceLegModule::AliceLegModule()
: control_cycle_msec_(8)
{
	running_ = false;
	gazebo_check = false;
	is_moving_l_ = false;
	is_moving_r_ = false;
	is_moving_one_joint_ = false;
	enable_       = false;
	module_name_  = "alice_leg_module";
	control_mode_ = robotis_framework::PositionControl;

	new_count_ = 0;
	// Dynamixel initialize //
	result_["l_hip_pitch"] = new robotis_framework::DynamixelState();  // joint 11
	result_["l_hip_roll"]  = new robotis_framework::DynamixelState();  // joint 13

	result_["l_hip_yaw"]   = new robotis_framework::DynamixelState();  // joint 15
	result_["l_knee_pitch"] = new robotis_framework::DynamixelState();  // joint 17
	result_["l_ankle_pitch"] = new robotis_framework::DynamixelState();  // joint 19
	result_["l_ankle_roll"]  = new robotis_framework::DynamixelState();  // joint 21

	result_["r_hip_pitch"] = new robotis_framework::DynamixelState();  // joint 12
	result_["r_hip_roll"]  = new robotis_framework::DynamixelState();  // joint 14
	result_["r_hip_yaw"]   = new robotis_framework::DynamixelState();  // joint 16
	result_["r_knee_pitch"] = new robotis_framework::DynamixelState();  // joint 18
	result_["r_ankle_pitch"] = new robotis_framework::DynamixelState();  // joint 20
	result_["r_ankle_roll"]  = new robotis_framework::DynamixelState();  // joint 22

	///////////////////////////
	l_kinematics_ = new heroehs_math::Kinematics;
	r_kinematics_ = new heroehs_math::Kinematics;
	end_to_rad_l_ = new heroehs_math::CalRad;
	end_to_rad_r_ = new heroehs_math::CalRad;

	//////////////////////////
	/*	currentGyroX = 0;
	currentGyroY = 0;
	currentGyroZ = 0;*/
	//////////////////////////
	traj_time_ = 4.0;

	result_end_l_.resize(6,1);
	result_end_r_.resize(6,1);
	result_end_l_.fill(0);
	result_end_r_.fill(0);

	result_end_l_.coeffRef(0,0) = 0;
	result_end_l_.coeffRef(1,0) = 0.09;
	result_end_l_.coeffRef(2,0) = -0.57;
	result_end_l_.coeffRef(3,0) = 0*DEGREE2RADIAN;
	result_end_l_.coeffRef(4,0) = 0*DEGREE2RADIAN;
	result_end_l_.coeffRef(5,0) = 0*DEGREE2RADIAN;

	result_end_r_.coeffRef(0,0) = 0;
	result_end_r_.coeffRef(1,0) = -0.09;
	result_end_r_.coeffRef(2,0) = -0.57;
	result_end_r_.coeffRef(3,0) = 0*DEGREE2RADIAN;
	result_end_r_.coeffRef(4,0) = 0*DEGREE2RADIAN;
	result_end_r_.coeffRef(5,0) = 0*DEGREE2RADIAN;

	result_mat_cob_ = result_mat_cob_modified_ = Eigen::Matrix4d::Identity();

	result_mat_l_ = robotis_framework::getTransformationXYZRPY(result_end_l_.coeff(0,0), result_end_l_.coeff(1,0), result_end_l_.coeff(2,0),
			result_end_l_.coeff(3,0), result_end_l_.coeff(4,0), result_end_l_.coeff(5,0));
	result_mat_r_ = robotis_framework::getTransformationXYZRPY(result_end_r_.coeff(0,0), result_end_r_.coeff(1,0), result_end_r_.coeff(2,0),
			result_end_r_.coeff(3,0), result_end_r_.coeff(4,0), result_end_r_.coeff(5,0));

	result_mat_l_modified_ = result_mat_l_;
	result_mat_r_modified_ = result_mat_r_;

	/*	balance_updating_duration_sec_ = 2.0;
	balance_updating_sys_time_sec_ = 2.0;
	balance_update_= false;
	// gyro joint space
	gyro_pitch_function = new control_function::PID_function(0.008,15*DEGREE2RADIAN,-15*DEGREE2RADIAN,0,0,0);
	gain_pitch_p_adjustment = new heroehs_math::FifthOrderTrajectory;
	gain_pitch_d_adjustment = new heroehs_math::FifthOrderTrajectory;

	updating_duration = 0;
	gyro_pitch_p_gain  = 0;
	gyro_pitch_d_gain  = 0;


	tf_current_gyro_x = 0;
	tf_current_gyro_y = 0;
	tf_current_gyro_z = 0;

	cop_cal = new  diana::CopCalculationFunc;*/

	/*	center_change_ = new diana_motion::CenterChange;
	temp_change_value_center = 0;
	temp_time_center_change  = 0;
	temp_time_edge_change = 0;
	temp_turn_type = "basic";
	temp_change_type = "basic";*/

	/*	currentFX_l=0.0;
	currentFY_l=0.0;
	currentFZ_l=0.0;
	currentTX_l=0.0;
	currentTY_l=0.0;
	currentTZ_l=0.0;
	currentFX_r=0.0;
	currentFY_r=0.0;
	currentFZ_r=0.0;
	currentTX_r=0.0;
	currentTY_r=0.0;
	currentTZ_r=0.0;
	// cop compensation
	cop_compensation = new diana::CopCompensationFunc();
	gain_copFz_p_adjustment = new heroehs_math::FifthOrderTrajectory;
	gain_copFz_d_adjustment = new heroehs_math::FifthOrderTrajectory;
	updating_duration_cop = 0;
	copFz_p_gain = 0;
	copFz_d_gain = 0;*/
	balance_updating_duration_sec_ = 2.0;
	balance_updating_sys_time_sec_ = 2.0;
	balance_update_ = false;

	zmp_cal = new alice::ZmpCalculationFunc;
	currentFX_l=0.0;
	currentFY_l=0.0;
	currentFZ_l=0.0;
	currentTX_l=0.0;
	currentTY_l=0.0;
	currentTZ_l=0.0;
	currentFX_r=0.0;
	currentFY_r=0.0;
	currentFZ_r=0.0;
	currentTX_r=0.0;
	currentTY_r=0.0;
	currentTZ_r=0.0;
	// cop compensation
	zmp_compensation = new alice::ZmpCompensationFunc();
	gain_zmpFz_p_adjustment = new heroehs_math::FifthOrderTrajectory;
	gain_zmpFz_d_adjustment = new heroehs_math::FifthOrderTrajectory;
	updating_duration_cop = 0;
	zmpFz_p_gain = 0;
	zmpFz_d_gain = 0;
}
AliceLegModule::~AliceLegModule()
{
	queue_thread_.join();
}

void AliceLegModule::queueThread()
{
	ros::NodeHandle ros_node;
	ros::CallbackQueue callback_queue;
	ros_node.setCallbackQueue(&callback_queue);
	/* publisher topics */
	zmp_fz_pub = ros_node.advertise<std_msgs::Float64MultiArray>("/zmp_fz",100);


	/* subscribe topics */
	get_ft_data_sub_ = ros_node.subscribe("/alice/force_torque_data", 100, &AliceLegModule::ftDataMsgCallback, this);
	// for gui
	set_balance_param_sub_ = ros_node.subscribe("/alice/balance_parameter", 5, &AliceLegModule::setBalanceParameterCallback, this);



	ros::WallDuration duration(control_cycle_msec_ / 1000.0);
	while(ros_node.ok())
		callback_queue.callAvailable(duration);
}
/*
void AliceLegModule::imuDataMsgCallback(const sensor_msgs::Imu::ConstPtr& msg) // gyro data get
{
	currentGyroX = (double) msg->angular_velocity.x;
	currentGyroY = (double) msg->angular_velocity.y;
	currentGyroZ = (double) msg->angular_velocity.z;
	gyroRotationTransformation(currentGyroZ, currentGyroY, currentGyroX);
	balance_ctrl_.setCurrentGyroSensorOutput(tf_current_gyro_x, tf_current_gyro_y);
}
void AliceLegModule::gyroRotationTransformation(double gyro_z, double gyro_y, double gyro_x)
{
	Eigen::MatrixXd tf_gyro_value;
	tf_gyro_value.resize(3,1);
	tf_gyro_value.fill(0);
	tf_gyro_value(0,0) =  gyro_x;
	tf_gyro_value(1,0) =  gyro_y;
	tf_gyro_value(2,0) =  gyro_z;

	tf_gyro_value = (robotis_framework::getRotationZ(-M_PI/2)*robotis_framework::getRotationY(-M_PI))*tf_gyro_value;
	tf_current_gyro_x = tf_gyro_value(0,0);
	tf_current_gyro_y = tf_gyro_value(1,0);
	tf_current_gyro_z = tf_gyro_value(2,0);
}*/
void AliceLegModule::ftDataMsgCallback(const alice_msgs::ForceTorque::ConstPtr& msg)// force torque sensor data get
{
	currentFX_l = (double) msg->force_x_raw_l;
	currentFY_l = (double) msg->force_y_raw_l;
	currentFZ_l = (double) msg->force_z_raw_l;


	currentTX_l = (double) msg->torque_x_raw_l;
	currentTY_l = (double) msg->torque_y_raw_l;
	currentTZ_l = (double) msg->torque_z_raw_l;

	currentFX_r = (double) msg->force_x_raw_r;
	currentFY_r = (double) msg->force_y_raw_r;
	currentFZ_r = (double) msg->force_z_raw_r;


	currentTX_r = (double) msg->torque_x_raw_r;
	currentTY_r = (double) msg->torque_y_raw_r;
	currentTZ_r = (double) msg->torque_z_raw_r;

	if(currentFX_l && currentFY_l && currentFZ_l && currentTX_l && currentTY_l && currentTZ_l)
	{
		zmpFz_p_gain = 0;
		zmpFz_d_gain = 0;
	} // cop control disable

	zmp_cal->ftSensorDataLeftGet(currentFX_l, currentFY_l, currentFZ_l, currentTX_l, currentTY_l, currentTZ_l);
	zmp_cal->ftSensorDataRightGet(currentFX_r, currentFY_r, currentFZ_r, currentTX_r, currentTY_r, currentTZ_r);
	zmp_cal->jointStateGetForTransForm(l_kinematics_->joint_radian, r_kinematics_->joint_radian);
	zmp_cal->ZmpCalculationResult();
	//cop_compensation->centerOfPressureReferencePoint(temp_turn_type,   cop_cal->cf_px_l, cop_cal->cf_py_l, cop_cal->cf_pz_l,
			//cop_cal->cf_px_r, cop_cal->cf_py_r, cop_cal->cf_pz_r, temp_change_value_center);
}
void AliceLegModule::setBalanceParameterCallback(const alice_msgs::BalanceParam::ConstPtr& msg)
{
	if(balance_update_ == true)
	{
		ROS_ERROR("the previous task is not finished");
		return;
	}
	ROS_INFO("SET BALANCE_PARAM");

	balance_updating_duration_sec_ = 2.0;
	if(msg->updating_duration < 0)
		balance_updating_duration_sec_ = 2.0;
	else
		balance_updating_duration_sec_ = msg->updating_duration;

	/*desired_balance_param_.cob_x_offset_m         = 0;
	desired_balance_param_.cob_y_offset_m         = 0;
	desired_balance_param_.foot_roll_gyro_p_gain  = 0;
	desired_balance_param_.foot_roll_gyro_d_gain  = 0;
	desired_balance_param_.foot_pitch_gyro_p_gain = 0;
	desired_balance_param_.foot_pitch_gyro_d_gain = 0;

	updating_duration = msg->updating_duration;
	gyro_pitch_p_gain  = msg->foot_pitch_gyro_p_gain;
	gyro_pitch_d_gain  = msg->foot_pitch_gyro_d_gain;*/

	updating_duration_cop = msg->updating_duration;
	zmpFz_p_gain = msg->foot_zmpFz_p_gain;
	zmpFz_d_gain = msg->foot_zmpFz_d_gain;


	balance_param_update_coeff_.changeTrajectory(0,0,0,0, balance_updating_duration_sec_, 1, 0, 0);

	balance_update_ = true;
	balance_updating_sys_time_sec_ = 0;
	//zmp_compensation->parse_margin_data(); // cop margin data ;
}
/*void AliceLegModule::desiredPoseMsgCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) // GUI 에서 init pose topic을 sub 받아 실행
{
	for(int joint_num_= 0; joint_num_< 6; joint_num_++)
	{
		leg_end_point_l_(joint_num_, 1) = msg->data[joint_num_]; // left leg
		leg_end_point_r_(joint_num_, 1) = msg->data[joint_num_+6]; // right leg
		leg_end_point_l_(joint_num_,7)  = msg->data[12];
		leg_end_point_r_(joint_num_,7)  = msg->data[12];
	}
	is_moving_l_ = true;
	is_moving_r_ = true;
}*/

/*void AliceLegModule::desiredPoseAllMsgCallback(const diana_msgs::DesiredPoseCommand::ConstPtr& msg)
{
	for(int joint_num_= 0; joint_num_< 6; joint_num_++)
	{
		leg_end_point_l_(joint_num_, 1) = msg->leg_final_position[joint_num_];
		leg_end_point_r_(joint_num_, 1) = msg->leg_final_position[joint_num_+6];

		leg_end_point_l_(joint_num_, 2) = msg->leg_init_vel[joint_num_];
		leg_end_point_r_(joint_num_, 2) = msg->leg_init_vel[joint_num_+6];

		leg_end_point_l_(joint_num_, 3) = msg->leg_final_vel[joint_num_];
		leg_end_point_r_(joint_num_, 3) = msg->leg_final_vel[joint_num_+6];

		leg_end_point_l_(joint_num_,7)  = msg->time_leg;
		leg_end_point_r_(joint_num_,7)  = msg->time_leg;
	}
	is_moving_l_ = true;
	is_moving_r_ = true;

}*/




