/*
 * kinematics_dynamics.h
 *
 *  Created on: Jun 3, 2018
 *      Author: jaysong
 */

#ifndef ALICE_KINEMATICS_DYNAMICS_KINEMATICS_DYNAMICS_H_
#define ALICE_KINEMATICS_DYNAMICS_KINEMATICS_DYNAMICS_H_


#include "kinematics_dynamics_define.h"
#include "link_data.h"

namespace alice
{


class KinematicsDynamics
{

public:
  KinematicsDynamics();
  ~KinematicsDynamics();

  std::vector<int> findRoute(int to);
  std::vector<int> findRoute(int from, int to);

  double calcTotalMass(int joint_id);
  Eigen::MatrixXd calcMassCenter(int joint_id);
  Eigen::MatrixXd calcCenterOfMass(Eigen::MatrixXd mc);

  void calcJointsCenterOfMass(int joint_id);

  void calcForwardKinematics(int joint_ID);

  //Eigen::MatrixXd calcJacobian(std::vector<int> idx);
  //Eigen::MatrixXd calcJacobianCOM(std::vector<int> idx);
  //Eigen::MatrixXd calcVWerr(Eigen::MatrixXd tar_position, Eigen::MatrixXd curr_position, Eigen::MatrixXd tar_orientation, Eigen::MatrixXd curr_orientation);

  //bool calcInverseKinematics(int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation , int max_iter, double ik_err);
  //bool calcInverseKinematics(int from, int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation, int max_iter, double ik_err);

  // with weight
  //bool calcInverseKinematics(int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation, int max_iter, double ik_err , Eigen::MatrixXd weight);
  //bool calcInverseKinematics(int from, int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation, int max_iter, double ik_err, Eigen::MatrixXd weight);

  bool calcInverseKinematicsForLeg(double *out, double x, double y, double z, double roll, double pitch, double yaw);
  bool calcInverseKinematicsForRightLeg(double *out, double x, double y, double z, double roll, double pitch, double yaw);
  bool calcInverseKinematicsForLeftLeg(double *out, double x, double y, double z, double roll, double pitch, double yaw);

  LinkData *alice_link_data_ [ ALL_JOINT_ID + 1 ];

  double thigh_length_m_;
  double calf_length_m_;
  double ankle_length_m_;
  double leg_side_offset_m_;
};

}



#endif /* ALICE_KINEMATICS_DYNAMICS_KINEMATICS_DYNAMICS_H_ */
