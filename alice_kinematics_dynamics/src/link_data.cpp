/*
 * link_data.cpp
 *
 *  Created on: Jun 3, 2018
 *      Author: jaysong
 */

#include "alice_kinematics_dynamics/link_data.h"

namespace alice
{

LinkData::LinkData()
{
  name_ = "";

  parent_ = -1;
  sibling_ = -1;
  child_ = -1;

  mass_ = 0.0;

  relative_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
  joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
  center_of_mass_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
  joint_center_of_mass_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
  inertia_ =robotis_framework::getInertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  joint_limit_max_ = 100.0;
  joint_limit_min_ = -100.0;

  joint_angle_ = 0.0;
  joint_velocity_ = 0.0;
  joint_acceleration_ = 0.0;

  position_ = robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  orientation_ = robotis_framework::convertRPYToRotation( 0.0 , 0.0 , 0.0 );
  transformation_ = robotis_framework::getTransformationXYZRPY( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0);
}

LinkData::~LinkData(){}

}


