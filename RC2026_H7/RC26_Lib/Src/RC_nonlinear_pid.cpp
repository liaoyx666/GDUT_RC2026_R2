#include "RC_nonlinear_pid.h"

namespace pid
{
	NonlinearPid::NonlinearPid(float kp_, float accel_, float delta_, float max_out_)
	{
		Init(kp_, accel_, delta_, max_out_);
	}
	
	NonlinearPid::NonlinearPid()
	{
		kp = 0;
		two_accel = 0;
		delta = 0;
		max_out = 0;

		x0 = 0;
		y0 = 0;
	}

	void NonlinearPid::Init(float kp_, float accel_, float delta_, float max_out_)
	{
		kp = fabsf(kp_);
		two_accel = fabsf(2.f * accel_);
		delta = fabsf(delta_);
		max_out = fabsf(max_out_);
		
		if (kp_ != 0 && accel_ != 0)
		{
			x0 = powf(accel_ / (kp_ * sqrtf(2.f * accel_)), 2.f);
		}
		else
		{
			x0 = 0;
		}
		
		y0 = sqrtf(2.f * accel_ * x0);
		
		x0 = delta_ - x0;
		y0 = delta_ * kp_ - y0;
	}
	
	float NonlinearPid::NPid_Calculate(float target_, float real_, bool normalization, float unit)
	{
		unit = fabsf(unit);
		
		float period = 2.f * unit;
		
		float error = target_ - real_;
		
		if (normalization == true)// 归一化
		{
			if (error > unit) error = error - period;
			else if (error < -unit) error = error + period;
		}
		
		float abs_error = fabsf(error);
		
		float sgn_error = (error > 0.f ? 1.f : -1.f);
		
		float output;
		
		if (abs_error <= delta)
		{
			output = error * kp;
		}
		else
		{
			output = sgn_error * (sqrtf(two_accel * fabsf(abs_error - x0)) + y0);
		}
		
		return fminf(fmaxf(output, -max_out), max_out);
	}
}