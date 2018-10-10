/*
 * alice_kinematics_dynamics.cpp
 *
 *  Created on: Jun 3, 2018
 *      Author: jaysong
 */

#include "alice_kinematics_dynamics/kinematics_dynamics.h"

using namespace alice;

KinematicsDynamics::KinematicsDynamics()
{
  for (int id=0; id<=ALL_JOINT_ID; id++)
    alice_link_data_[id] = new LinkData();

  alice_link_data_[0]->name_               =  "base";
  alice_link_data_[0]->parent_             =  -1;
  alice_link_data_[0]->sibling_            =  -1;
  alice_link_data_[0]->child_              =  29;
  alice_link_data_[0]->mass_               =  0.0;
  alice_link_data_[0]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  alice_link_data_[0]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  alice_link_data_[0]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  alice_link_data_[0]->joint_limit_max_    =  100.0;
  alice_link_data_[0]->joint_limit_min_    =  -100.0;
  alice_link_data_[0]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

  /* ----- passive joint -----*/
  alice_link_data_[29]->name_               =  "passive_x";
  alice_link_data_[29]->parent_             =  0;
  alice_link_data_[29]->sibling_            =  -1;
  alice_link_data_[29]->child_              =  30;
  alice_link_data_[29]->mass_               =  0.0;
  alice_link_data_[29]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  alice_link_data_[29]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  alice_link_data_[29]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  alice_link_data_[29]->joint_limit_max_    =  100.0;
  alice_link_data_[29]->joint_limit_min_    =  -100.0;
  alice_link_data_[29]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

  alice_link_data_[30]->name_               =  "passive_y";
  alice_link_data_[30]->parent_             =  29;
  alice_link_data_[30]->sibling_            =  -1;
  alice_link_data_[30]->child_              =  31;
  alice_link_data_[30]->mass_               =  0.0;
  alice_link_data_[30]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  alice_link_data_[30]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  alice_link_data_[30]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  alice_link_data_[30]->joint_limit_max_    =  100.0;
  alice_link_data_[30]->joint_limit_min_    =  -100.0;
  alice_link_data_[30]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

  alice_link_data_[31]->name_               =  "passive_z";
  alice_link_data_[31]->parent_             =  30;
  alice_link_data_[31]->sibling_            =  -1;
  alice_link_data_[31]->child_              =  32;
  alice_link_data_[31]->mass_               =  0.0;
  alice_link_data_[31]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  alice_link_data_[31]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  alice_link_data_[31]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  alice_link_data_[31]->joint_limit_max_    =  100.0;
  alice_link_data_[31]->joint_limit_min_    =  -100.0;
  alice_link_data_[31]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

  alice_link_data_[32]->name_               =  "passive_yaw";
  alice_link_data_[32]->parent_             =  31;
  alice_link_data_[32]->sibling_            =  -1;
  alice_link_data_[32]->child_              =  33;
  alice_link_data_[32]->mass_               =  0.0;
  alice_link_data_[32]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  alice_link_data_[32]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 1.0 );
  alice_link_data_[32]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  alice_link_data_[32]->joint_limit_max_    =  2.0 * M_PI;
  alice_link_data_[32]->joint_limit_min_    =  -2.0 * M_PI;
  alice_link_data_[32]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

  alice_link_data_[33]->name_               =  "passive_pitch";
  alice_link_data_[33]->parent_             =  32;
  alice_link_data_[33]->sibling_            =  -1;
  alice_link_data_[33]->child_              =  34;
  alice_link_data_[33]->mass_               =  0.0;
  alice_link_data_[33]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  alice_link_data_[33]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 1.0 , 0.0 );
  alice_link_data_[33]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  alice_link_data_[33]->joint_limit_max_    =  2.0 * M_PI;
  alice_link_data_[33]->joint_limit_min_    =  -2.0 * M_PI;
  alice_link_data_[33]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

  alice_link_data_[34]->name_               =  "passive_roll";
  alice_link_data_[34]->parent_             =  33;
  alice_link_data_[34]->sibling_            =  -1;
  alice_link_data_[34]->child_              =  28;
  alice_link_data_[34]->mass_               =  0.0;
  alice_link_data_[34]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  alice_link_data_[34]->joint_axis_         =  robotis_framework::getTransitionXYZ( 1.0 , 0.0 , 0.0 );
  alice_link_data_[34]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  alice_link_data_[34]->joint_limit_max_    =  2.0 * M_PI;
  alice_link_data_[34]->joint_limit_min_    =  -2.0 * M_PI;
  alice_link_data_[34]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

  /* ----- body -----*/
  // pelvis_link
  alice_link_data_[28]->name_               =  "pelvis";
  alice_link_data_[28]->parent_             =  34;
  alice_link_data_[28]->sibling_            =  -1;
  alice_link_data_[28]->child_              =  10;
  alice_link_data_[28]->mass_               =  1;
  alice_link_data_[28]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  alice_link_data_[28]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  alice_link_data_[28]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.1 );
  alice_link_data_[28]->joint_limit_max_    =  100.0;
  alice_link_data_[28]->joint_limit_min_    =  -100.0;
  alice_link_data_[28]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

  //torso p
  alice_link_data_[10]->name_               =  "torso_p";
  alice_link_data_[10]->parent_             =  28;
  alice_link_data_[10]->sibling_            =  11;
  alice_link_data_[10]->child_              =  9;
  alice_link_data_[10]->mass_               =  1;
  alice_link_data_[10]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.0 ,  0.0 , 0.2 );
  alice_link_data_[10]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , -1.0 , 0.0 );
  alice_link_data_[10]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 ,  0.0 , 0.072 );
  alice_link_data_[10]->joint_limit_max_    =  0.6 * M_PI;
  alice_link_data_[10]->joint_limit_min_    =  -0.6 * M_PI;
  alice_link_data_[10]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0, 0.0);

  //torso y
  alice_link_data_[9]->name_               =  "torso_y";
  alice_link_data_[9]->parent_             =  10;
  alice_link_data_[9]->sibling_            =  -1;
  alice_link_data_[9]->child_              =  7;
  alice_link_data_[9]->mass_               =  5.383;
  alice_link_data_[9]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 ,  0.144 );
  alice_link_data_[9]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , -1.0 );
  alice_link_data_[9]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 ,  0.0 );
  alice_link_data_[9]->joint_limit_max_    =  0.6 * M_PI;
  alice_link_data_[9]->joint_limit_min_    =  -0.6 * M_PI;
  alice_link_data_[9]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0, 0.0);

  /* ----- head -----*/
  // head_pitch
  alice_link_data_[7]->name_               =  "head_p";
  alice_link_data_[7]->parent_             =  9;
  alice_link_data_[7]->sibling_            =  1;
  alice_link_data_[7]->child_              =  8;
  alice_link_data_[7]->mass_               =  0.724;
  alice_link_data_[7]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  alice_link_data_[7]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 1.0 , 0.0 );
  alice_link_data_[7]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  alice_link_data_[7]->joint_limit_max_    =  0.5 * M_PI;
  alice_link_data_[7]->joint_limit_min_    =  -0.5 * M_PI;
  alice_link_data_[7]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0, 0.0);

  // head_yaw
  alice_link_data_[8]->name_               =  "head_y";
  alice_link_data_[8]->parent_             =  7;
  alice_link_data_[8]->sibling_            =  -1;
  alice_link_data_[8]->child_              =  27;
  alice_link_data_[8]->mass_               =  1;
  alice_link_data_[8]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  alice_link_data_[8]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 1.0 );
  alice_link_data_[8]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  alice_link_data_[8]->joint_limit_max_    =  0.5 * M_PI;
  alice_link_data_[8]->joint_limit_min_    =  -0.5 * M_PI;
  alice_link_data_[8]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0, 0.0);

  // camera
  alice_link_data_[27]->name_               =  "cam";
  alice_link_data_[27]->parent_             =  8;
  alice_link_data_[27]->sibling_            =  -1;
  alice_link_data_[27]->child_              =  -1;
  alice_link_data_[27]->mass_               =  1;
  alice_link_data_[27]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  alice_link_data_[27]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  alice_link_data_[27]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  alice_link_data_[27]->joint_limit_max_    =  0.5 * M_PI;
  alice_link_data_[27]->joint_limit_min_    =  -0.5 * M_PI;
  alice_link_data_[27]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0, 0.0);

  /*----- left arm -----*/
  // left arm shoulder pitch 1
  alice_link_data_[1]->name_               =  "l_arm_sh_p1";
  alice_link_data_[1]->parent_             =  9;
  alice_link_data_[1]->sibling_            =  2;
  alice_link_data_[1]->child_              =  3;
  alice_link_data_[1]->mass_               =  1.0;
  alice_link_data_[1]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  alice_link_data_[1]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 1.0 , 0.0 );
  alice_link_data_[1]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  alice_link_data_[1]->joint_limit_max_    =  0.65 * M_PI;
  alice_link_data_[1]->joint_limit_min_    =  -0.65 * M_PI;
  alice_link_data_[1]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0, 0.0);

  // left arm shoulder roll
  alice_link_data_[3]->name_               =  "l_arm_sh_r";
  alice_link_data_[3]->parent_             =  1;
  alice_link_data_[3]->sibling_            =  -1;
  alice_link_data_[3]->child_              =  5;
  alice_link_data_[3]->mass_               =  1.0;
  alice_link_data_[3]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  alice_link_data_[3]->joint_axis_         =  robotis_framework::getTransitionXYZ( 1.0 , 0.0 , 0.0 );
  alice_link_data_[3]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  alice_link_data_[3]->joint_limit_max_    =  0.65 * M_PI;
  alice_link_data_[3]->joint_limit_min_    =  -0.65 * M_PI;
  alice_link_data_[3]->inertia_            =  robotis_framework::getInertiaXYZ( 0.00043 , 0.00000 , 0.00000 , 0.00112 , 0.00000 , 0.00113 );

  // left arm elbow yaw
  alice_link_data_[5]->name_               =  "l_arm_el_y";
  alice_link_data_[5]->parent_             =  5;
  alice_link_data_[5]->sibling_            =  -1;
  alice_link_data_[5]->child_              =  23;
  alice_link_data_[5]->mass_               =  1.0;
  alice_link_data_[5]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  alice_link_data_[5]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 1.0 );
  alice_link_data_[5]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  alice_link_data_[5]->joint_limit_max_    =  0.45 * M_PI;
  alice_link_data_[5]->joint_limit_min_    =  -0.45 * M_PI;
  alice_link_data_[5]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0, 0.0);


  // left arm end effector
  alice_link_data_[23]->name_               =  "l_arm_end";
  alice_link_data_[23]->parent_             =  5;
  alice_link_data_[23]->sibling_            =  -1;
  alice_link_data_[23]->child_              =  -1;
  alice_link_data_[23]->mass_               =  0.0;
  alice_link_data_[23]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.145 , 0.045 , 0.0 );
  alice_link_data_[23]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  alice_link_data_[23]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  alice_link_data_[23]->joint_limit_max_    =  100.0;
  alice_link_data_[23]->joint_limit_min_    =  -100.0;
  alice_link_data_[23]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );


  /*----- right arm -----*/
  // right arm shoulder pitch 1
  alice_link_data_[2]->name_               =  "r_arm_sh_p1";
  alice_link_data_[2]->parent_             =  9;
  alice_link_data_[2]->sibling_            =  -1;
  alice_link_data_[2]->child_              =  4;
  alice_link_data_[2]->mass_               =  1.0;
  alice_link_data_[2]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  alice_link_data_[2]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , -1.0 , 0.0 );
  alice_link_data_[2]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  alice_link_data_[2]->joint_limit_max_    =  0.65 * M_PI;
  alice_link_data_[2]->joint_limit_min_    =  -0.65 * M_PI;
  alice_link_data_[2]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

  // right arm shoulder roll
  alice_link_data_[4]->name_               =  "r_arm_sh_r";
  alice_link_data_[4]->parent_             =  2;
  alice_link_data_[4]->sibling_            =  -1;
  alice_link_data_[4]->child_              =  6;
  alice_link_data_[4]->mass_               =  1.0;
  alice_link_data_[4]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  alice_link_data_[4]->joint_axis_         =  robotis_framework::getTransitionXYZ( 1.0 , 0.0 , 0.0 );
  alice_link_data_[4]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  alice_link_data_[4]->joint_limit_max_    =  0.65 * M_PI;
  alice_link_data_[4]->joint_limit_min_    =  -0.65 * M_PI;
  alice_link_data_[4]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

  // right arm elbow yaw
  alice_link_data_[6]->name_               =  "r_arm_el_y";
  alice_link_data_[6]->parent_             =  4;
  alice_link_data_[6]->sibling_            =  -1;
  alice_link_data_[6]->child_              =  24;
  alice_link_data_[6]->mass_               =  1.0;
  alice_link_data_[6]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  alice_link_data_[6]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 1.0 );
  alice_link_data_[6]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  alice_link_data_[6]->joint_limit_max_    =  0.45 * M_PI;
  alice_link_data_[6]->joint_limit_min_    =  -0.45 * M_PI;
  alice_link_data_[6]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );


  // right arm end effector
  alice_link_data_[24]->name_               =  "r_arm_end";
  alice_link_data_[24]->parent_             =  6;
  alice_link_data_[24]->sibling_            =  -1;
  alice_link_data_[24]->child_              =  -1;
  alice_link_data_[24]->mass_               =  0.0;
  alice_link_data_[24]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  alice_link_data_[24]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  alice_link_data_[24]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  alice_link_data_[24]->joint_limit_max_    =  100.0;
  alice_link_data_[24]->joint_limit_min_    =  -100.0;
  alice_link_data_[24]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );


  /* ----- left leg -----*/
  // left leg hip yaw
  alice_link_data_[11]->name_               =  "l_leg_hip_y";
  alice_link_data_[11]->parent_             =  13;
  alice_link_data_[11]->sibling_            =  -1;
  alice_link_data_[11]->child_              =  17;
  alice_link_data_[11]->mass_               =  1.0;
  alice_link_data_[11]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  alice_link_data_[11]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , -1.0 , 0.0 );
  alice_link_data_[11]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  alice_link_data_[11]->joint_limit_max_    =  2.0 * M_PI; //0.45 * M_PI;
  alice_link_data_[11]->joint_limit_min_    =  -2.0 * M_PI; //-0.45 * M_PI;
  alice_link_data_[11]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

  // left leg hip roll
  alice_link_data_[13]->name_               =  "l_leg_hip_r";
  alice_link_data_[13]->parent_             =  15;
  alice_link_data_[13]->sibling_            =  -1;
  alice_link_data_[13]->child_              =  11;
  alice_link_data_[13]->mass_               =  1.0;
  alice_link_data_[13]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  alice_link_data_[13]->joint_axis_         =  robotis_framework::getTransitionXYZ( -1.0 , 0.0 , 0.0 );
  alice_link_data_[13]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  alice_link_data_[13]->joint_limit_max_    =  2.0 * M_PI; //0.3 * M_PI;
  alice_link_data_[13]->joint_limit_min_    =  -2.0 * M_PI; //-0.3 * M_PI;
  alice_link_data_[13]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

  // left leg hip pitch
  alice_link_data_[15]->name_               =  "l_leg_hip_p";
  alice_link_data_[15]->parent_             =  28;
  alice_link_data_[15]->sibling_            =  16;
  alice_link_data_[15]->child_              =  13;
  alice_link_data_[15]->mass_               =  1.0;
  alice_link_data_[15]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.0 , 0.09 , 0.0 );
  alice_link_data_[15]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 1.0 );
  alice_link_data_[15]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  alice_link_data_[15]->joint_limit_max_    =  2.0 * M_PI; //0.4 * M_PI;
  alice_link_data_[15]->joint_limit_min_    =  -2.0 * M_PI; //-0.4 * M_PI;
  alice_link_data_[15]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

  // left leg knee pitch
  alice_link_data_[17]->name_               =  "l_leg_kn_p";
  alice_link_data_[17]->parent_             =  11;
  alice_link_data_[17]->sibling_            =  -1;
  alice_link_data_[17]->child_              =  19;
  alice_link_data_[17]->mass_               =  1.0;
  alice_link_data_[17]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.000 , 0.00 , -0.24 );
  alice_link_data_[17]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0, -1.0 , 0.0 );
  alice_link_data_[17]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  alice_link_data_[17]->joint_limit_max_    =  2.0 * M_PI; //0.7 * M_PI;
  alice_link_data_[17]->joint_limit_min_    =  -2.0 * M_PI; //-0.1 * M_PI;
  alice_link_data_[17]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

  // left leg ankle pitch
  alice_link_data_[19]->name_               =  "l_leg_an_p";
  alice_link_data_[19]->parent_             =  17;
  alice_link_data_[19]->sibling_            =  -1;
  alice_link_data_[19]->child_              =  21;
  alice_link_data_[19]->mass_               =  1.0;
  alice_link_data_[19]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.000 , 0.0 , -0.24 );
  alice_link_data_[19]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 1.0 , 0.0 );
  alice_link_data_[19]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  alice_link_data_[19]->joint_limit_max_    =  2.0 * M_PI; //0.45 * M_PI;
  alice_link_data_[19]->joint_limit_min_    =  -2.0 * M_PI; //-0.45 * M_PI;
  alice_link_data_[19]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

  // left leg ankle roll
  alice_link_data_[21]->name_               =  "l_leg_an_r";
  alice_link_data_[21]->parent_             =  19;
  alice_link_data_[21]->sibling_            =  -1;
  alice_link_data_[21]->child_              =  25;
  alice_link_data_[21]->mass_               =  1.0;
  alice_link_data_[21]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  alice_link_data_[21]->joint_axis_         =  robotis_framework::getTransitionXYZ( 1.0 , 0.0 , 0.0 );
  alice_link_data_[21]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  alice_link_data_[21]->joint_limit_max_    =  2.0 * M_PI; //0.45 * M_PI;
  alice_link_data_[21]->joint_limit_min_    =  -2.0 * M_PI; //-0.45 * M_PI;
  alice_link_data_[21]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );


  // left leg end
  alice_link_data_[25]->name_               =  "l_leg_end";
  alice_link_data_[25]->parent_             =  21;
  alice_link_data_[25]->sibling_            =  -1;
  alice_link_data_[25]->child_              =  -1;
  alice_link_data_[25]->mass_               =  0.0;
  alice_link_data_[25]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , -0.127 );
  alice_link_data_[25]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  alice_link_data_[25]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  alice_link_data_[25]->joint_limit_max_    =  100.0;
  alice_link_data_[25]->joint_limit_min_    =  -100.0;
  alice_link_data_[25]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );



  /* ----- right leg -----*/
  // right leg hip yaw
  alice_link_data_[12]->name_               =  "r_leg_hip_y";
  alice_link_data_[12]->parent_             =  14;
  alice_link_data_[12]->sibling_            =  -1;
  alice_link_data_[12]->child_              =  18;
  alice_link_data_[12]->mass_               =  1.0;
  alice_link_data_[12]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  alice_link_data_[12]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 1.0 , 0.0 );
  alice_link_data_[12]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  alice_link_data_[12]->joint_limit_max_    =  2.0 * M_PI; //0.45 * M_PI;
  alice_link_data_[12]->joint_limit_min_    =  -2.0 * M_PI; //-0.45 * M_PI;
  alice_link_data_[12]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

  // right leg hip roll
  alice_link_data_[14]->name_               =  "r_leg_hip_r";
  alice_link_data_[14]->parent_             =  16;
  alice_link_data_[14]->sibling_            =  -1;
  alice_link_data_[14]->child_              =  12;
  alice_link_data_[14]->mass_               =  1.0;
  alice_link_data_[14]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  alice_link_data_[14]->joint_axis_         =  robotis_framework::getTransitionXYZ( -1.0 , 0.0 , 0.0 );
  alice_link_data_[14]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  alice_link_data_[14]->joint_limit_max_    =  2.0 * M_PI; //0.3 * M_PI;
  alice_link_data_[14]->joint_limit_min_    =  -2.0 * M_PI; //-0.3 * M_PI;
  alice_link_data_[14]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

  // right leg hip pitch
  alice_link_data_[16]->name_               =  "r_leg_hip_p";
  alice_link_data_[16]->parent_             =  28;
  alice_link_data_[16]->sibling_            =  -1;
  alice_link_data_[16]->child_              =  14;
  alice_link_data_[16]->mass_               =  1.0;
  alice_link_data_[16]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.0 , -0.09 , 0.0 );
  alice_link_data_[16]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 1.0 );
  alice_link_data_[16]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  alice_link_data_[16]->joint_limit_max_    =  2.0 * M_PI; //0.4 * M_PI;
  alice_link_data_[16]->joint_limit_min_    =  -2.0 * M_PI; //-0.4 * M_PI;
  alice_link_data_[16]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

  // right leg knee pitch
  alice_link_data_[18]->name_               =  "r_leg_kn_p";
  alice_link_data_[18]->parent_             =  12;
  alice_link_data_[18]->sibling_            =  -1;
  alice_link_data_[18]->child_              =  20;
  alice_link_data_[18]->mass_               =  2.401;
  alice_link_data_[18]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , -0.24 );
  alice_link_data_[18]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 1.0 , 0.0 );
  alice_link_data_[18]->center_of_mass_     =  robotis_framework::getTransitionXYZ( -0.002 , 0.066 , -0.183 );
  alice_link_data_[18]->joint_limit_max_    =  2.0 * M_PI; //0.1 * M_PI;
  alice_link_data_[18]->joint_limit_min_    =  -2.0 * M_PI; //-0.7 * M_PI;
  alice_link_data_[18]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

  // right leg ankle pitch
  alice_link_data_[20]->name_               =  "r_leg_an_p";
  alice_link_data_[20]->parent_             =  18;
  alice_link_data_[20]->sibling_            =  -1;
  alice_link_data_[20]->child_              =  22;
  alice_link_data_[20]->mass_               =  1.0;
  alice_link_data_[20]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , -0.24 );
  alice_link_data_[20]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , -1.0 , 0.0 );
  alice_link_data_[20]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0, 0.0 , 0.0 );
  alice_link_data_[20]->joint_limit_max_    =  2.0 * M_PI; //0.45 * M_PI;
  alice_link_data_[20]->joint_limit_min_    =  -2.0 * M_PI; //-0.45 * M_PI;
  alice_link_data_[20]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

  // right leg ankle roll
  alice_link_data_[22]->name_               =  "r_leg_an_r";
  alice_link_data_[22]->parent_             =  20;
  alice_link_data_[22]->sibling_            =  -1;
  alice_link_data_[22]->child_              =  26;
  alice_link_data_[22]->mass_               =  0.223;
  alice_link_data_[22]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  alice_link_data_[22]->joint_axis_         =  robotis_framework::getTransitionXYZ( 1.0 , 0.0 , 0.0 );
  alice_link_data_[22]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  alice_link_data_[22]->joint_limit_max_    =  2.0 * M_PI; //0.45 * M_PI;
  alice_link_data_[22]->joint_limit_min_    =  -2.0 * M_PI; //-0.45 * M_PI;
  alice_link_data_[22]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

  // right leg end
  alice_link_data_[26]->name_               =  "r_leg_end";
  alice_link_data_[26]->parent_             =  22;
  alice_link_data_[26]->sibling_            =  -1;
  alice_link_data_[26]->child_              =  -1;
  alice_link_data_[26]->mass_               =  0.0;
  alice_link_data_[26]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , -0.127 );
  alice_link_data_[26]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  alice_link_data_[26]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  alice_link_data_[26]->joint_limit_max_    =  100.0;
  alice_link_data_[26]->joint_limit_min_    =  -100.0;
  alice_link_data_[26]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );



  thigh_length_m_= 0.24;
  calf_length_m_ = 0.24;
  ankle_length_m_ = 0.127;
  leg_side_offset_m_ = 0.18;

  KinematicsGraig();
}


double KinematicsDynamics::calcTotalMass(int joint_id)
{
  double mass;

  if (joint_id == -1)
    mass = 0.0;
  else
    mass = alice_link_data_[joint_id]->mass_ + calcTotalMass(alice_link_data_[ joint_id ]->sibling_) + calcTotalMass(alice_link_data_[joint_id]->child_);

  return mass;
}

Eigen::MatrixXd KinematicsDynamics::calcMassCenter(int joint_id)
{
  Eigen::MatrixXd mc(3,1);

  if (joint_id == -1)
    mc = Eigen::MatrixXd::Zero(3,1);
  else
  {
    mc = alice_link_data_[ joint_id ]->mass_ * ( alice_link_data_[ joint_id ]->orientation_ * alice_link_data_[ joint_id ]->center_of_mass_ + alice_link_data_[ joint_id ]->position_ );
    mc = mc + calcMassCenter( alice_link_data_[ joint_id ]->sibling_ ) + calcMassCenter( alice_link_data_[ joint_id ]->child_ );
  }

  return mc;
}

void KinematicsDynamics::calcJointsCenterOfMass(int joint_id)
{
  if(joint_id != -1)
  {
    LinkData *temp_data = alice_link_data_[ joint_id ];
    temp_data->joint_center_of_mass_
      = ( temp_data->orientation_ * temp_data->center_of_mass_ + temp_data->position_ );

    calcJointsCenterOfMass(temp_data->sibling_);
    calcJointsCenterOfMass(temp_data->child_);
  }
  else
    return;
}


Eigen::MatrixXd KinematicsDynamics::calcCenterOfMass(Eigen::MatrixXd mc)
{
  double mass ;
  Eigen::MatrixXd COM(3,1);

  mass = calcTotalMass(0);
  COM = mc/mass;

  return COM;
}

void KinematicsDynamics::calcForwardKinematics(int joint_id)
{
  if (joint_id == -1)
    return;

  if (joint_id == 0)
  {
    alice_link_data_[0]->position_ = Eigen::MatrixXd::Zero(3,1);
    alice_link_data_[0]->orientation_ =
        robotis_framework::calcRodrigues( robotis_framework::calcHatto( alice_link_data_[0]->joint_axis_ ), alice_link_data_[ 0 ]->joint_angle_ );
  }

  if ( joint_id != 0 )
  {
    int parent = alice_link_data_[joint_id]->parent_;

    alice_link_data_[joint_id]->position_ =
        alice_link_data_[parent]->orientation_ * alice_link_data_[joint_id]->relative_position_ + alice_link_data_[parent]->position_;
    alice_link_data_[ joint_id ]->orientation_ =
        alice_link_data_[ parent ]->orientation_ *
        robotis_framework::calcRodrigues(robotis_framework::calcHatto(alice_link_data_[joint_id]->joint_axis_), alice_link_data_[joint_id]->joint_angle_);

        //alice_link_data_[joint_id]->transformation_.block<3,1>(0,3) = alice_link_data_[joint_id]->position_;
        //alice_link_data_[joint_id]->transformation_.block<3,3>(0,0) = alice_link_data_[joint_id]->orientation_;
  }

  calcForwardKinematics(alice_link_data_[joint_id]->sibling_);
  calcForwardKinematics(alice_link_data_[joint_id]->child_);
}

bool KinematicsDynamics::calcInverseKinematicsForLeg(double *out, double x, double y, double z, double roll, double pitch, double yaw)
{
  Eigen::Matrix4d trans_ad, trans_da, trans_cd, trans_dc;
  Eigen::Matrix3d rot_ac;
  Eigen::Vector3d vec;

  bool invertible;
  double rac, arc_cos, arc_tan, alpha;
  double thigh_length = thigh_length_m_;
  double calf_length = calf_length_m_;
  double ankle_length = ankle_length_m_;

  trans_ad = robotis_framework::getTransformationXYZRPY(x, y, z, roll, pitch, yaw);

  vec.coeffRef(0) = trans_ad.coeff(0, 3) + trans_ad.coeff(0, 2) * ankle_length;
  vec.coeffRef(1) = trans_ad.coeff(1, 3) + trans_ad.coeff(1, 2) * ankle_length;
  vec.coeffRef(2) = trans_ad.coeff(2, 3) + trans_ad.coeff(2, 2) * ankle_length;

  // Get Knee
  rac = vec.norm();
  arc_cos = acos(
      (rac * rac - thigh_length * thigh_length - calf_length * calf_length) / (2.0 * thigh_length * calf_length));
  if (std::isnan(arc_cos) == 1)
    return false;
  *(out + 3) = arc_cos;

  // Get Ankle Roll
  trans_ad.computeInverseWithCheck(trans_da, invertible);
  if (invertible == false)
    return false;

  vec.coeffRef(0) = trans_da.coeff(0, 3);
  vec.coeffRef(1) = trans_da.coeff(1, 3);
  vec.coeffRef(2) = trans_da.coeff(2, 3) - ankle_length;

  arc_tan = atan2(vec(1), vec(2));
  if(arc_tan > M_PI_2)
    arc_tan = arc_tan - M_PI;
  else if(arc_tan < -M_PI_2)
    arc_tan = arc_tan + M_PI;

  *(out+5) = arc_tan;

  //Get Ankle Pitch
  alpha = asin( thigh_length*sin(M_PI - *(out+3)) / rac);
  *(out+4) = -atan2(vec(0), copysign(sqrt(vec(1)*vec(1) + vec(2)*vec(2)),vec(2))) - alpha;

  // Get Hip Pitch
  rot_ac = ( robotis_framework::convertRPYToRotation(roll, pitch, yaw)*robotis_framework::getRotationX(-(*(out+5))))
      * robotis_framework::getRotationY(-(*(out+3) + *(out+4)));

  arc_tan = atan2(rot_ac.coeff(0, 2), rot_ac.coeff(2, 2));
  *(out) = arc_tan;

  // Get Hip Roll
  arc_tan = atan2(-rot_ac.coeff(1, 2), rot_ac.coeff(0, 2) * sin(*(out)) + rot_ac.coeff(2, 2) * cos(*(out)));
  *(out + 1) = arc_tan;

  // Get Hip Yaw
  arc_tan = atan2(rot_ac.coeff(1, 0), rot_ac.coeff(1, 1));
  *(out+2) = arc_tan;


  return true;
}

bool KinematicsDynamics::calcInverseKinematicsForRightLeg(double *out, double x, double y, double z, double roll, double pitch, double yaw)
{
  if(calcInverseKinematicsForLeg(out, x, y, z, roll, pitch, yaw) == true) {

    *(out + 0) = out[0] * (alice_link_data_[ID_R_LEG_START + 2*0]->joint_axis_.coeff(1,0));
    *(out + 1) = out[1] * (alice_link_data_[ID_R_LEG_START + 2*1]->joint_axis_.coeff(0,0));
    *(out + 2) = out[2] * (alice_link_data_[ID_R_LEG_START + 2*2]->joint_axis_.coeff(2,0));
    *(out + 3) = out[3] * (alice_link_data_[ID_R_LEG_START + 2*3]->joint_axis_.coeff(1,0));
    *(out + 4) = out[4] * (alice_link_data_[ID_R_LEG_START + 2*4]->joint_axis_.coeff(1,0));
    *(out + 5) = out[5] * (alice_link_data_[ID_R_LEG_START + 2*5]->joint_axis_.coeff(0,0));
    return true;
  }
  else
    return false;
}

bool KinematicsDynamics::calcInverseKinematicsForLeftLeg(double *out, double x, double y, double z, double roll, double pitch, double yaw)
{
  if(calcInverseKinematicsForLeg(out, x, y, z, roll, pitch, yaw) == true) {

    out[0] = out[0] * (alice_link_data_[ID_L_LEG_START + 2*0]->joint_axis_.coeff(1,0));
    out[1] = out[1] * (alice_link_data_[ID_L_LEG_START + 2*1]->joint_axis_.coeff(0,0));
    out[2] = out[2] * (alice_link_data_[ID_L_LEG_START + 2*2]->joint_axis_.coeff(2,0));
    out[3] = out[3] * (alice_link_data_[ID_L_LEG_START + 2*3]->joint_axis_.coeff(1,0));
    out[4] = out[4] * (alice_link_data_[ID_L_LEG_START + 2*4]->joint_axis_.coeff(1,0));
    out[5] = out[5] * (alice_link_data_[ID_L_LEG_START + 2*5]->joint_axis_.coeff(0,0));
    return true;
  }
  else
    return false;
}

bool KinematicsDynamics::KinematicsGraig()
{
  joint_radian.resize(7,1);
  joint_radian.fill(0);

  // kinematics variables //
  P_inverse_.fill(0);
  P_.fill(0);

  // DH convention variables
  dh_alpha[0] = 0;
  dh_alpha[1] = M_PI/2;
  dh_alpha[2] = -M_PI/2;
  dh_alpha[3] = -M_PI/2;
  dh_alpha[4] = 0;
  dh_alpha[5] = M_PI/2;
  dh_alpha[6] = 0;

  dh_link[0] = 0;
  dh_link[1] = 0;
  dh_link[2] = 0;
  dh_link[3] = 0;
  dh_link[4] = 0.240;
  dh_link[5] = 0;
  dh_link[6] = 0.127;

  total_length_ = 0.607;
  sensor_length_ = 0.127;

  dh_link_d[0] = 0;
  dh_link_d[1] = 0;
  dh_link_d[2] = 0;
  dh_link_d[3] = 0.240;
  dh_link_d[4] = 0;
  dh_link_d[5] = 0;
  dh_link_d[6] = 0;

  real_theta[0] = 0;
  real_theta[1] = 0;
  real_theta[2] = 0;
  real_theta[3] = 0;
  real_theta[4] = 0;
  real_theta[5] = 0;
  real_theta[6] = 0;

  for(int i=0; i<8;i++)
  {
    H[i].resize(4,4);
    H[i].fill(0);
  }
  center_to_sensor_transform_right.resize(4,4);
  center_to_sensor_transform_right.fill(0);
  center_to_sensor_transform_left.resize(4,4);
  center_to_sensor_transform_left.fill(0);
  center_to_foot_transform_left_leg.resize(4,4);
  center_to_foot_transform_left_leg.fill(0);
  center_to_foot_transform_right_leg.resize(4,4);
  center_to_foot_transform_right_leg.fill(0);
  H_ground_to_center.resize(4,4);
  H_ground_to_center.fill(0);

  H[7] << 0 , 0, -1, 0,
      0 , 1,  0, 0,
      1 , 0,  0, 0,
      0 , 0,  0, 1;

  H_ground_to_center << 1 , 0, 0, 0,
      0 , 1, 0, 0,
      0 , 0, 1, total_length_,
      0 , 0, 0, 1;


}
void KinematicsDynamics::FowardKinematics(double joint[7], std::string left_right)
{
  double sum_theta[7] = {0,0,0,0,0,0,0};
  double offset_theta[7] = {0, 0, (M_PI)/2, -(M_PI)/2, (M_PI)/2, 0, 0};

  for(int i=1; i<7; i++)
  {
    sum_theta[i] = joint[i]+ offset_theta[i];
  }
  for(int i=1; i<7; i++)
  {
    H[i](0,0) = floor(100000.*(cos(sum_theta[i])+0.000005))/100000.;
    H[i](0,1) = floor(100000.*(-cos(dh_alpha[i])*sin(sum_theta[i])+0.000005))/100000.;
    H[i](0,2) = floor(100000.*(sin(dh_alpha[i])*sin(sum_theta[i])+0.000005))/100000.;
    H[i](0,3) = floor(100000.*(dh_link[i]*cos(sum_theta[i])+0.000005))/100000.;

    H[i](1,0) = floor(100000.*(sin(sum_theta[i])+0.000005))/100000.;
    H[i](1,1) = floor(100000.*(cos(dh_alpha[i])*cos(sum_theta[i])+0.000005))/100000.;
    H[i](1,2) = floor(100000.*(-sin(dh_alpha[i])*cos(sum_theta[i])+0.000005))/100000.;
    H[i](1,3) = floor(100000.*(dh_link[i]*sin(sum_theta[i])+0.000005))/100000.;

    H[i](2,0) =0;
    H[i](2,1) = floor(100000.*(sin(dh_alpha[i])+0.000005))/100000.;
    H[i](2,2) = floor(100000.*(cos(dh_alpha[i])+0.000005))/100000.;
    H[i](2,3) = -dh_link_d[i];

    H[i](3,0) =0;
    H[i](3,1) =0;
    H[i](3,2) =0;
    H[i](3,3) =1;

    H[i](0,0) = cos(sum_theta[i]);
    H[i](0,1) = -cos(dh_alpha[i])*sin(sum_theta[i]);
    H[i](0,2) = sin(dh_alpha[i])*sin(sum_theta[i]);
    H[i](0,3) = dh_link[i]*cos(sum_theta[i]);

    H[i](1,0) = sin(sum_theta[i]);
    H[i](1,1) = cos(dh_alpha[i])*cos(sum_theta[i]);
    H[i](1,2) = -sin(dh_alpha[i])*cos(sum_theta[i]);
    H[i](1,3) = dh_link[i]*sin(sum_theta[i]);

    H[i](2,0) =0;
    H[i](2,1) = sin(dh_alpha[i]);
    H[i](2,2) = cos(dh_alpha[i]);
    H[i](2,3) = -dh_link_d[i];

    H[i](3,0) =0;
    H[i](3,1) =0;
    H[i](3,2) =0;
    H[i](3,3) =1;
  }

  H[0](0,0) = 0;
  H[0](0,1) = -1;
  H[0](0,2) = 0;
  H[0](0,3) = 0;

  H[0](1,0) = 0;
  H[0](1,1) = 0;
  H[0](1,2) = 1;
  H[0](1,3) = -0.09;

  H[0](2,0) = -1;
  H[0](2,1) = 0;
  H[0](2,2) = 0;
  H[0](2,3) = 0;

  H[0](3,0) = 0;
  H[0](3,1) = 0;
  H[0](3,2) = 0;
  H[0](3,3) = 1;
  //// foot frame 을 pelvis frame 과 일치 시킨다.
  if(!left_right.compare("left")) // left
  {
    H[0](1,3) = 0.09;
    center_to_foot_transform_left_leg = H[0]*H[1]*H[2]*H[3]*H[4]*H[5]*H[6]*H[7];
  }
  else // right
  {
    H[0](1,3) = -0.09;
    center_to_foot_transform_right_leg = H[0]*H[1]*H[2]*H[3]*H[4]*H[5]*H[6]*H[7];
  }
}



