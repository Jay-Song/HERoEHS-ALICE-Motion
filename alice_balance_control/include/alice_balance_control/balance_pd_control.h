/*
 * balance_pd_control.h
 *
 *  Created on: May 29, 2018
 *      Author: robotemperor
 */

#ifndef ALICE_HEROEHS_ALICE_MOTION_ALICE_BALANCE_CONTROL_INCLUDE_ALICE_BALANCE_CONTROL_BALANCE_PD_CONTROL_H_
#define ALICE_HEROEHS_ALICE_MOTION_ALICE_BALANCE_CONTROL_INCLUDE_ALICE_BALANCE_CONTROL_BALANCE_PD_CONTROL_H_

namespace alice
{

class BalancePDController
{
public:
  BalancePDController();
  ~BalancePDController();

  double control_cycle_sec_;

  double desired_;

  double p_gain_;
  double d_gain_;

  double getFeedBack(double present_sensor_output);

private:
  double curr_err_;
  double prev_err_;
};

}



#endif /* ALICE_HEROEHS_ALICE_MOTION_ALICE_BALANCE_CONTROL_INCLUDE_ALICE_BALANCE_CONTROL_BALANCE_PD_CONTROL_H_ */
