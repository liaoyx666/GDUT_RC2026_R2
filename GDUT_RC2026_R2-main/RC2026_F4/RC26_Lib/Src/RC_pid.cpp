#include "RC_pid.h"

namespace pid
{

	void Pid::Pid_Mode_Init(bool incremental_, bool differential_prior_, float differential_lowpass_alpha_)
	{
		incremental = incremental_;
		
		differential_prior = differential_prior_;// 微分先行	
		
		differential_lowpass_alpha_ = fabsf(differential_lowpass_alpha_);
		if (differential_lowpass_alpha_ >= 1) differential_lowpass_alpha = 0;
		else differential_lowpass_alpha = differential_lowpass_alpha_;// 微分滤波
		
	}

	void Pid::Pid_Param_Init(
		float kp_, float ki_, float kd_, float kf_, float delta_time_, float deadzone_, float output_limit_, 
		float integral_limit_, float integral_separation_, float differential_limit_, float feed_forward_limit_
	)
	{
		kp = kp_;
		ki = ki_;
		kd = kd_;
		kf = kf_;// 前馈控制
		
		integral_limit = fabsf(integral_limit_);// 积分限幅
		output_limit = fabsf(output_limit_);// 输出限幅
		differential_limit = fabsf(differential_limit_);// 微分限幅
		feed_forward_limit = fabsf(feed_forward_limit_);// 前馈限幅
		
		integral_separation = fabsf(integral_separation_);// 积分分离
		deadzone = fabsf(deadzone_);// 死区

		delta_time_ = fabsf(delta_time_);
		if (delta_time_ == 0) delta_time_ = 0.001;
		else delta_time = delta_time_;// 时间差
	}





	float Pid::Pid_Calculate(bool normalization, float unit)
	{
		if (unit < 0) unit = -unit;
		
		// 前馈控制
		feed_forward = target - last_target;
		
		if (normalization == true)
		{
			if (feed_forward > unit) feed_forward = feed_forward - 2 * unit;
			else if (feed_forward < -unit) feed_forward = feed_forward + 2 * unit;
		}
		
		feed_forward =  feed_forward / delta_time * kf;
		
		// 前馈限幅
		if (feed_forward_limit != 0)
		{
			if (feed_forward > feed_forward_limit) feed_forward = feed_forward_limit;
			else if (feed_forward < -feed_forward_limit) feed_forward = -feed_forward_limit;
		}
		
		/*------------------------------------------------------------------------------------*/
		// 误差
		error = target - real;
		
		
		if (normalization == true)
		{
			if (error > unit) error = error - 2 * unit;
			else if (error < -unit) error = error + 2 * unit;
		}

		if (fabsf(error) < deadzone) error = 0;// 死区
		
		/*------------------------------------------------------------------------------------*/
		// 比例
		if (incremental == true) proportion = (error - last_error) * kp;
		else proportion = error * kp;
		
		/*------------------------------------------------------------------------------------*/
		// 积分
		if (incremental == true)
		{
			integral = error * ki;
		}
		else
		{
			// 积分分离
			if (integral_separation != 0)
			{
				if (fabsf(error) < integral_separation)
				{
					integral += (error + last_error) * delta_time * 0.5 * ki;// 梯形积分
				}
			}
			else
			{
				integral += (error + last_error) * delta_time * 0.5 * ki;// 梯形积分
			}
			
			// 积分限幅
			if (integral_limit != 0)
			{
				if (integral > integral_limit) integral = integral_limit;
				else if (integral < -integral_limit) integral = -integral_limit;
			}
		}
		
		/*------------------------------------------------------------------------------------*/
		// 微分
		if (incremental == true)
		{
			differential = (error - 2 * last_error + previous_error) * kd;// 普通微分
		}
		else
		{
			if (differential_prior == true)
			{
				differential = real - last_real;
				
				if (normalization == true)
				{
					if (differential > unit) differential = differential - 2 * unit;
					else if (differential < -unit) differential = differential + 2 * unit;
				}
				
				differential = differential / delta_time * kd;// 微分先行
			}
			else
			{
				differential = (error - last_error) / delta_time * kd;// 普通微分
			}
			
			// 微分滤波
			differential = differential_lowpass_alpha * last_differential + (1 - differential_lowpass_alpha) * differential;
			
			// 微分限幅
			if (differential_limit != 0)
			{
				if (differential > differential_limit) differential = differential_limit;
				else if (differential < -differential_limit) differential = -differential_limit;
			}
		}
		
		/*------------------------------------------------------------------------------------*/
		// 输出
		if (incremental == true)
		{
			output = proportion + integral + differential + last_output + feed_forward;
		}
		else
		{
			output = proportion + integral + differential + feed_forward;
		}
		
		// 输出限幅
		if (output_limit != 0)
		{
			if (output > output_limit) output = output_limit;
			else if (output < -output_limit) output = -output_limit;
		}
		
		
		/*------------------------------------------------------------------------------------*/
		// 更新
		previous_error = last_error;
		last_error = error;
		
		last_output = output;
		
		last_real = real;
		
		last_target = target;
		
		last_differential = differential;
		last_proportion = proportion;
		
		
		return output;
	}
	
	
	
	
	float Normalize(float data, float unit)
	{
		if (unit == 0) return 0;
 
		if (unit < 0) unit = -unit;
		
		if (isnan(data) || isnan(unit) || isinf(data) || isinf(unit)) return 0;
		
		while(data > unit)
			data = data - 2 * unit;
		while(data < -unit)
			data = data + 2 * unit;
		return data;
	}
	
	
	
	
	
	
	
	
	
	
	
}

