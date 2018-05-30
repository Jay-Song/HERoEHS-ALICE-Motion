/*
 * control_function.h
 *
 *  Created on: May 29, 2018
 *      Author: robotemperor
 */

#ifndef ALICE_HEROEHS_ALICE_MOTION_ALICE_BALANCE_CONTROL_INCLUDE_ALICE_BALANCE_CONTROL_CONTROL_FUNCTION_H_
#define ALICE_HEROEHS_ALICE_MOTION_ALICE_BALANCE_CONTROL_INCLUDE_ALICE_BALANCE_CONTROL_CONTROL_FUNCTION_H_



#include <stdio.h>
#include <math.h>

namespace control_function
{

class PID_function
{
public:
	PID_function(double dt, double max, double min, double kp, double kd, double ki);
	~PID_function();
	double PID_calculate(double ref_value, double current_value);
	double kp_;
	double kd_;
	double ki_;
	double max_;
	double min_;

private:
	double dt_;
	double pre_error_;
	double integral_;
};

class Filter
{
public:
	Filter();
	~Filter();
	double lowPassFilter(double value, double pre_value, double weight_factor, double sampling_time);
	double averageFilter(double value, double number, double min, double max);
	int signFunction(double value);

private:
	double number_count;
	double average_value;
};
}

#endif /* ALICE_HEROEHS_ALICE_MOTION_ALICE_BALANCE_CONTROL_INCLUDE_ALICE_BALANCE_CONTROL_CONTROL_FUNCTION_H_ */
