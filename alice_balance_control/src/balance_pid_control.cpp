/*
 * balance_pid_control.cpp
 *
 *  Created on: May 29, 2018
 *      Author: robotemperor
 */




#include "alice_balance_control/balance_pid_control.h"


using namespace alice;


BalancePIDController::BalancePIDController()
{
  control_cycle_sec_ = 0.008;

  desired_ = 0;

  p_gain_ = 0;
  i_gain_ = 0;
  d_gain_ = 0;

  curr_err_ = 0;
  prev_err_ = 0;

  sum_err_ = 0;
}

BalancePIDController::~BalancePIDController()
{  }


double BalancePIDController::getFeedBack(double present_sensor_output)
{
  prev_err_ = curr_err_;
  curr_err_ = desired_ - present_sensor_output;

  sum_err_ += curr_err_;

  return (p_gain_*curr_err_ + i_gain_*sum_err_*control_cycle_sec_ + d_gain_*(curr_err_ - prev_err_)/control_cycle_sec_ );
}
