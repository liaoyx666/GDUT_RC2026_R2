#pragma once
#include <math.h>
#include <arm_math.h>

#define TWO_PI 6.2831853071795864769f
#define HALF_PI 1.570796326794896619f
#define THREE_THIRD_PI 4.71238898038468985769f

#ifdef __cplusplus


namespace pid
{
		
	class Pid
	{
	public:
		Pid(){};
		virtual	~Pid(){};
			
		void Pid_Mode_Init(bool incremental_ = true, bool differential_prior_ = true, float differential_lowpass_alpha_ = 0);
			
		void Pid_Param_Init(
			float kp_, float ki_, float kd_, float kf_ = 0, float delta_time_ = 0.001, float deadzone_ = 0, float output_limit_ = 0, 
			float integral_limit_ = 0, float integral_separation_ = 0, float differential_limit_ = 0, float feed_forward_limit_ = 0
		);
							
		float Pid_Calculate(bool normalization = false, float unit = PI);
		
		void Update_Real(float real_){real = real_;}
		void Update_Target(float target_){target = target_;}
		float Get_Output(){return output;}
		
		
		void Set_Kp(float kp_){kp = kp_;}
		void Set_Ki(float ki_){ki = ki_;}
		void Set_Kd(float kd_){kd = kd_;}
		void Set_Kf(float kf_){kf = kf_;}
		
		
		void Set_Differential_lowpass_alpha(float differential_lowpass_alpha_)
		{
			differential_lowpass_alpha_ = fabsf(differential_lowpass_alpha_);
			if (differential_lowpass_alpha_ >= 1) differential_lowpass_alpha = 0;
			else differential_lowpass_alpha = differential_lowpass_alpha_;// 微分滤波
		}
		
		void Set_integral_limit(float integral_limit_){integral_limit = fabsf(integral_limit_);}
		void Set_output_limit(float output_limit_){output_limit = fabsf(output_limit_);}

		
		
		
		
		
		
		
		
		
		float kp = 0, ki = 0, kd = 0, kf = 0;
		float integral_separation = 0;
		float integral_limit = 0, output_limit = 0, differential_limit = 0, feed_forward_limit = 0;
		float deadzone = 0;
		float delta_time = 0.001;

		bool differential_prior = false;// 默认普通微分
		bool incremental = true;// 默认增量式Pid

		float integral = 0, differential = 0, proportion = 0, feed_forward = 0;
		
		float target = 0, real = 0, output = 0, error = 0;
		float last_output = 0, last_error = 0, last_real = 0, last_target = 0;
		float last_differential = 0, last_proportion = 0;
		float previous_error = 0;
		
		float differential_lowpass_alpha = 0;



		
	protected:
		
	private:
		
	};

}

#endif
