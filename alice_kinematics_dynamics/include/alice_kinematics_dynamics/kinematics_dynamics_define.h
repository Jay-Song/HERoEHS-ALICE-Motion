/*
 * kinematics_dynamics_define.h
 *
 *  Created on: Jun 3, 2018
 *      Author: jaysong
 */

#ifndef ALICE_KINEMATICS_DYNAMICS_KINEMATICS_DYNAMICS_DEFINE_H_
#define ALICE_KINEMATICS_DYNAMICS_KINEMATICS_DYNAMICS_DEFINE_H_

namespace alice
{

#define MAX_JOINT_ID    (22)
#define ALL_JOINT_ID    (35) //base, passive xyzrpy, pelvis, 4 branch end

#define MAX_ARM_ID      (3)
#define MAX_LEG_ID      (6)
#define MAX_ITER        (5)

#define ID_HEAD_END     (8)
#define ID_COB          (44)
#define ID_TORSO_P      (10)

#define ID_R_ARM_START  (2)
#define ID_L_ARM_START  (1)
#define ID_R_ARM_END    (24)
#define ID_L_ARM_END    (23)

#define ID_R_LEG_START  (12)
#define ID_L_LEG_START  (11)
#define ID_R_LEG_END    (26)
#define ID_L_LEG_END    (25)


#define ID_BASE         (0)
#define ID_PELVIS       (28)

#define ID_PELVIS_POS_X (29)
#define ID_PELVIS_POS_Y (30)
#define ID_PELVIS_POS_Z (31)
#define ID_PELVIS_ROT_X (32)
#define ID_PELVIS_ROT_Y (33)
#define ID_PELVIS_ROT_Z (34)


#define GRAVITY_ACCELERATION (9.8)

}



#endif /* ALICE_KINEMATICS_DYNAMICS_KINEMATICS_DYNAMICS_DEFINE_H_ */
