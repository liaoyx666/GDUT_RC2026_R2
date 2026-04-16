#pragma once
#include <math.h>
#include <arm_math.h>

#include "RC_serial.h"
#include "RC_filter.h"

#ifndef TWO_PI
#define TWO_PI 			6.2831853071795864769f
#endif


#ifndef HALF_PI
#define HALF_PI 		1.570796326794896619f
#endif

#ifndef TWO_THIRD_PI
#define TWO_THIRD_PI 	4.71238898038468985769f
#endif

#ifdef __cplusplus
namespace pid
{
	class Pid
	{
	public:
		Pid() {};
		~Pid() = default;
			
		void Pid_Mode_Init(bool incremental_ = true, bool differential_prior_ = true, float differential_lowpass_alpha_ = 0, bool use_td_ = false);
		
		void Pid_Param_Init(
			float kp_, float ki_, float kd_, float kf_ = 0, float delta_time_ = 0.001, float deadzone_ = 0, float output_limit_ = 0, 
			float integral_limit_ = 0, float integral_separation_ = 0, float differential_limit_ = 0, float feed_forward_limit_ = 0,
			float r_ = 50, float v2_max_ = 0.f
		);
		
		float Pid_Calculate(float real, float target, bool normalization = false, float unit = PI);
		float Mit_Calculate(
			float real_pos, float real_spd, 
			float target_pos, float target_spd = 0.f, float target_tor = 0.f, 
			bool normalization = false, float unit = PI
		);
		
		void Set_Delta_Time(float delta_time_) {delta_time = delta_time_;}
		
		void Set_Kp(float kp_) {kp = kp_;}
		void Set_Ki(float ki_) {ki = ki_;}
		void Set_Kd(float kd_) {kd = kd_;}
		void Set_Kf(float kf_) {kf = kf_;}
		
		void Set_Differential_lowpass_alpha(float differential_lowpass_alpha_);
		void Set_integral_limit(float integral_limit_) {integral_limit = fabsf(integral_limit_);}
		void Set_output_limit(float output_limit_) {output_limit = fabsf(output_limit_);}
		void Set_R(float r_) {td.Set_R(r_);}
		void Set_V2_Max(float v2_max_) {td.Set_V2_Max(v2_max_);}
		void Set_Td(float r_, float v2_max_) {td.TD_Init(r_, delta_time, v2_max_);}
		
	protected:
		
	private:
		float kp = 0, ki = 0, kd = 0, kf = 0;
		float integral_separation = 0;
		float integral_limit = 0;
		float output_limit = 0;
		float differential_limit = 0;
		float feed_forward_limit = 0;
		float deadzone = 0;
		float delta_time = 0.001f;

		bool differential_prior = false;// 默认普通微分
		bool incremental = true;// 默认增量式Pid

		float integral = 0;
		float last_output = 0;
		float last_error = 0;
		float last_real = 0;
		float last_target = 0;
		float last_differential = 0;
		float previous_error = 0;
		
		float differential_lowpass_alpha = 0;

		bool use_td = false;
		filter::TD td;
	};

	void Limit(float *input, float limit);
	float Normalize(float data, float unit);
}
#endif
