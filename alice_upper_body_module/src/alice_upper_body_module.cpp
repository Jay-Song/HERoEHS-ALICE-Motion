/*
 * alice_upper_body_module.cpp
 *
 *  Created on: May 30, 2018
 *      Author: robotemperor
 */
#include "alice_upper_body_module/alice_upper_body_module.h"

using namespace alice_upper_body_module;

void UpperBodyModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
	control_cycle_msec_ = control_cycle_msec;
	queue_thread_ = boost::thread(boost::bind(&UpperBodyModule::queueThread, this));

	for (std::map<std::string, robotis_framework::Dynamixel*>::iterator it = robot->dxls_.begin();
			it != robot->dxls_.end(); it++)
	{
		std::string joint_name = it->first;
		robotis_framework::Dynamixel* dxl_info = it->second;

		joint_name_to_id_[joint_name] = dxl_info->id_;
		joint_id_to_name_[dxl_info->id_] = joint_name;
	}
	// waist yaw roll
	waist_end_point_.resize(6,8);
	waist_end_point_.fill(0);
	result_rad_waist_.resize(6,1);
	result_rad_waist_.fill(0);

	// head
	head_end_point_.resize(6,8);
	head_end_point_.fill(0);
	result_rad_head_.resize(6,1);
	result_rad_head_.fill(0);

	head_end_point_(4,0) = -10*DEGREE2RADIAN; // pitch 초기값
	head_end_point_(4,1) = -10*DEGREE2RADIAN; //
	end_to_rad_head_->cal_end_point_tra_betta->current_pose = -10*DEGREE2RADIAN;
	end_to_rad_head_->current_pose_change(4,0) = -10*DEGREE2RADIAN;
	temp_pre_pitch = -10*DEGREE2RADIAN; // low pass filter initialize


	for(int joint_num_= 3; joint_num_< 6 ; joint_num_ ++)  // waist 3, 5번 // head 345 초기화
	{
		waist_end_point_(joint_num_, 7) = traj_time_test;
		head_end_point_ (joint_num_, 7) = traj_time_test;
	}
	waist_end_point_(4, 7) = 0;

	ROS_INFO("< -------  Initialize Module : Upper Body Module  [HEAD  && WAIST] !!  ------->");
}
/*
void UpperBodyModule::updateBalanceGyroParameter()
{
	Eigen::MatrixXd value;
	value.resize(1,8);
	value.fill(0);

	value(0,7) = updating_duration;
	value(0,1) = gyro_roll_p_gain;
	gyro_roll_function->kp_ = gain_roll_p_adjustment -> fifth_order_traj_gen_one_value(value);
	value(0,1) = gyro_roll_d_gain;
	gyro_roll_function->kd_ = gain_roll_d_adjustment -> fifth_order_traj_gen_one_value(value);

	value(0,1) = gyro_yaw_p_gain;
	gyro_yaw_function->kp_ = gain_yaw_p_adjustment -> fifth_order_traj_gen_one_value(value);
	value(0,1) = gyro_yaw_d_gain;
	gyro_yaw_function->kd_ = gain_yaw_d_adjustment -> fifth_order_traj_gen_one_value(value);

	value(0,1) = copFz_p_gain;
	cop_compensation_waist->pidControllerFz_x->kp_ = gain_copFz_p_adjustment -> fifth_order_traj_gen_one_value(value);
	cop_compensation_waist->pidControllerFz_y->kp_ = cop_compensation_waist->pidControllerFz_x->kp_;

	value(0,1) = copFz_d_gain;
	cop_compensation_waist->pidControllerFz_x->kd_ = gain_copFz_d_adjustment -> fifth_order_traj_gen_one_value(value);
	cop_compensation_waist->pidControllerFz_y->kd_ = cop_compensation_waist->pidControllerFz_x->kd_;

	value(0,7) = head_enable_time;
	value(0,1) = head_enable;
	result_head_enable = head_balance_enable->fifth_order_traj_gen_one_value(value);

}*/
double UpperBodyModule::limitCheck(double calculated_value, double max, double min)
{
	if(calculated_value > (max*DEGREE2RADIAN))
		return (max*DEGREE2RADIAN);
	else if (calculated_value < (min*DEGREE2RADIAN))
		return (min*DEGREE2RADIAN);
	else
		return calculated_value;
}
bool UpperBodyModule::isRunning()
{
	return running_;
}
void UpperBodyModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
		std::map<std::string, double> sensors)
{
	if (enable_ == false)
	{
		return;
	}
	//updateBalanceGyroParameter();
	//motion();
	/*if(new_count_ == 1)
	{
		new_count_ ++;
		for (std::map<std::string, robotis_framework::Dynamixel*>::iterator state_iter = dxls.begin();
				state_iter != dxls.end(); state_iter++)
		{
			std::string joint_name = state_iter->first;
			if(!joint_name.compare("waist_yaw")|| !joint_name.compare("waist_roll") || !joint_name.compare("head_yaw") ||
					!joint_name.compare("head_pitch") || !joint_name.compare("head_roll"))
			{
				if(gazebo_check == true)
					result_[joint_name]->goal_position_ = result_[joint_name]->present_position_; // 가제보 상 초기위치 0
			}
		} // 등록된 다이나믹셀의 위치값을 읽어와서 goal position 으로 입력
		ROS_INFO("Upper Start");
		initial_tf_current_gyro_orientation_z = tf_current_gyro_orientation_z;
			initial_tf_current_position_x = tf_current_position_x;
		initial_tf_current_position_y = tf_current_position_y;
		initial_tf_current_position_z = tf_current_position_z;
	}*/
	if(is_moving_waist_ == false) // desired pose
	{
		ROS_INFO("Upper body Stay");
	}
	else
	{
		ROS_INFO("Upper Module start!!!!");
		waist_end_point_(3,1)   = limitCheck(result_rad_head_(3,0),60,-60);
		waist_end_point_(4,1)   = limitCheck(result_rad_head_(4,0),85,-15);

		result_rad_waist_ = end_to_rad_waist_ -> cal_end_point_to_rad(waist_end_point_);
		is_moving_waist_ = end_to_rad_waist_ -> is_moving_check;
	}
	///////////////////////////////////////////////////// control //////////////////////////////////////////////////////////
	if(is_moving_head_ == false)
	{
		//ROS_INFO("Upper Head Stay");
	}
	else
	{
		//ROS_INFO("Upper Module head!!!!");
		// limit must be calculated 23 24 25
		head_end_point_(3,1) = limitCheck(head_end_point_(3,1),78,-78);
		head_end_point_(4,1) = limitCheck(head_end_point_(4,1),85,-25);

		result_rad_head_  = end_to_rad_head_  -> cal_end_point_to_rad(head_end_point_);

		is_moving_head_  = end_to_rad_head_  -> is_moving_check;
	}

	temp_head_yaw   = limitCheck(result_rad_head_(3,0),78,-78);
	temp_head_pitch = limitCheck(result_rad_head_(4,0),85,-25);


	result_[joint_id_to_name_[7]]-> goal_position_  =  filter_head->lowPassFilter(temp_head_pitch, temp_pre_pitch, 0.02, 0.008);
	result_[joint_id_to_name_[8]]-> goal_position_  =  filter_head->lowPassFilter(temp_head_yaw, temp_pre_yaw, 0.02, 0.008);

	result_[joint_id_to_name_[9]] -> goal_position_  = -result_rad_waist_ (3,0); // waist pitch
	result_[joint_id_to_name_[10]]-> goal_position_  = -result_rad_waist_ (4,0); // waist yaw

	temp_pre_roll  = temp_head_roll;
	temp_pre_pitch = temp_head_pitch;
	temp_pre_yaw   = temp_head_yaw;
}
void UpperBodyModule::stop()
{
	return;
}





