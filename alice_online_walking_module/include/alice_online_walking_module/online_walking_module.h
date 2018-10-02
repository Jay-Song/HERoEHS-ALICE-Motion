/*
 * diana_online_walking_module.h
 *
 *  Created on: 2018. 2. 27.
 *      Author: Crowban
 */

#ifndef DIANA_ONLINE_WALKING_MODULE_DIANA_ONLINE_WALKING_MODULE_H_
#define DIANA_ONLINE_WALKING_MODULE_DIANA_ONLINE_WALKING_MODULE_H_

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <boost/thread.hpp>

#include "alice_online_walking_module/alice_online_walking.h"
#include "robotis_framework_common/motion_module.h"

#include "robotis_controller_msgs/StatusMsg.h"

#include "alice_walking_module_msgs/RobotPose.h"
#include "alice_walking_module_msgs/GetReferenceStepData.h"
#include "alice_walking_module_msgs/AddStepDataArray.h"
#include "alice_walking_module_msgs/StartWalking.h"
#include "alice_walking_module_msgs/IsRunning.h"
#include "alice_walking_module_msgs/RemoveExistingStepData.h"

#include "alice_walking_module_msgs/SetBalanceParam.h"
#include "alice_walking_module_msgs/SetJointFeedBackGain.h"

#include "alice_walking_module_msgs/WalkingJointStatesStamped.h"

#include "diana_msgs/ForceTorque.h"

namespace alice
{
class OnlineWalkingModule : public robotis_framework::MotionModule, public robotis_framework::Singleton<OnlineWalkingModule>
{
public:
  OnlineWalkingModule();
  virtual ~OnlineWalkingModule();

  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);

  void onModuleEnable();
  void onModuleDisable();

  void stop();
  bool isRunning();

private:
  void publishRobotPose(void);
  void publishStatusMsg(unsigned int type, std::string msg);
  void publishDoneMsg(std::string msg);
  void publishWalkingTuningData();

  /* ROS Topic Callback Functions */
  void imuDataOutputCallback(const sensor_msgs::Imu::ConstPtr &msg);
  void ftDataOutputCallback(const diana_msgs::ForceTorque::ConstPtr &msg);

  /* ROS Service Callback Functions */
  bool setBalanceParamServiceCallback(alice_walking_module_msgs::SetBalanceParam::Request  &req,
      alice_walking_module_msgs::SetBalanceParam::Response &res);

  bool setJointFeedBackGainServiceCallback(alice_walking_module_msgs::SetJointFeedBackGain::Request  &req,
      alice_walking_module_msgs::SetJointFeedBackGain::Response &res);

  bool getReferenceStepDataServiceCallback(alice_walking_module_msgs::GetReferenceStepData::Request  &req,
      alice_walking_module_msgs::GetReferenceStepData::Response &res);
  bool addStepDataServiceCallback(alice_walking_module_msgs::AddStepDataArray::Request  &req,
      alice_walking_module_msgs::AddStepDataArray::Response &res);
  bool startWalkingServiceCallback(alice_walking_module_msgs::StartWalking::Request  &req,
      alice_walking_module_msgs::StartWalking::Response &res);
  bool IsRunningServiceCallback(alice_walking_module_msgs::IsRunning::Request  &req,
      alice_walking_module_msgs::IsRunning::Response &res);
  bool removeExistingStepDataServiceCallback(alice_walking_module_msgs::RemoveExistingStepData::Request  &req,
      alice_walking_module_msgs::RemoveExistingStepData::Response &res);

  int convertStepDataMsgToStepData(alice_walking_module_msgs::StepData& src, robotis_framework::StepData& des);
  int convertStepDataToStepDataMsg(robotis_framework::StepData& src, alice_walking_module_msgs::StepData& des);

  void setBalanceParam(alice_walking_module_msgs::BalanceParam& balance_param_msg);

  void updateBalanceParam();

  bool checkBalanceOnOff();

  void queueThread();

  void setJointFeedBackGain(alice_walking_module_msgs::JointFeedBackGain& msg);
  void updateJointFeedBackGain();

  //yitake zmp output
  void realZmpCalculate(Eigen::Matrix4d g_right_foot, Eigen::Matrix4d g_left_foot, Eigen::MatrixXd g_right_force, Eigen::MatrixXd g_left_force);

  std::map<std::string, int> joint_name_to_index_;

  bool            gazebo_;
  int             control_cycle_msec_;
  boost::thread   queue_thread_;
  boost::mutex    process_mutex_;

  Eigen::MatrixXd desired_matrix_g_to_pelvis_;
  Eigen::MatrixXd desired_matrix_g_to_rfoot_;
  Eigen::MatrixXd desired_matrix_g_to_lfoot_;

  bool previous_running_, present_running;

  ros::Publisher robot_pose_pub_;
  ros::Publisher status_msg_pub_;
  ros::Publisher pelvis_base_msg_pub_;
  ros::Publisher done_msg_pub_;

  ros::Publisher walking_joint_states_pub_;
  ros::Publisher imu_orientation_states_pub_;
  ros::Publisher ft_states_pub_;
  alice_walking_module_msgs::WalkingJointStatesStamped walking_joint_states_msg_;

  alice_walking_module_msgs::RobotPose  robot_pose_msg_;
  bool balance_update_with_loop_;
  double balance_update_duration_;
  double balance_update_sys_time_;
  robotis_framework::FifthOrderPolynomialTrajectory balance_update_tra_;


  bool joint_feedback_update_with_loop_;
  double joint_feedback_update_duration_;
  double joint_feedback_update_sys_time_;
  robotis_framework::FifthOrderPolynomialTrajectory joint_feedback_update_tra_;

  alice_walking_module_msgs::JointFeedBackGain previous_joint_feedback_gain_;
  alice_walking_module_msgs::JointFeedBackGain current_joint_feedback_gain_;
  alice_walking_module_msgs::JointFeedBackGain desired_joint_feedback_gain_;

  alice_walking_module_msgs::BalanceParam previous_balance_param_;
  alice_walking_module_msgs::BalanceParam current_balance_param_;
  alice_walking_module_msgs::BalanceParam desired_balance_param_;

  heroehs::OnlineWalkingPatternGenerator online_walking;

  //yitaek test
  ros::Publisher reference_zmp_pub_;
  ros::Publisher reference_body_pub_;
  ros::Publisher foot_right_pub_;
  ros::Publisher foot_left_pub_;

  geometry_msgs::Vector3 reference_zmp_msg_;
  geometry_msgs::Vector3 reference_body_msg_;

  geometry_msgs::Vector3 foot_right_msg_;
  geometry_msgs::Vector3 foot_left_msg_;

  //yitaek zmp output
  double real_zmp_x, real_zmp_y;

  ros::Publisher real_zmp_pub_;
  geometry_msgs::Vector3 real_zmp_msg_;

};

}

#endif /* DIANA_ONLINE_WALKING_MODULE_DIANA_ONLINE_WALKING_MODULE_H_ */
