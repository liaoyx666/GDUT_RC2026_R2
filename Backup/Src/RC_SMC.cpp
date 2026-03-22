#include "RC_SMC.h"

namespace smc
{
	SMC::SMC()
	{
		output_limit = 0;
		lambda = 0;
		k = 0;
		delta = 0;
		dt = 0.001f;  // 默认值
		//target_pos = 0;
		real_pos = 0;
		real_spd = 0;
		error = 0;
		output = 0;
	}


	void SMC::SMC_Init(
		float output_limit_,
	    float lambda_,
	    float k_,
	    float delta_,
	    float dt_
	)
	{
		output_limit = fabsf(output_limit_);
		
		lambda = (lambda_ > 0) ? lambda_ : 1.0f;  // 确保为正
		k = (k_ > 0) ? k_ : 1.0f;                // 确保为正
		delta = (delta_ > 0) ? delta_ : 0.01f;   // 确保为正
		
		if (dt_ <= 0.f) dt = 0.001f;
		else dt = dt_;
	}
	
	void SMC::Update_Target_Spd(float target_spd_)
	{
		target_spd = target_spd_;
	}
	
	
	void SMC::Update_Real_Pos(float real_pos_)
	{
		real_pos = real_pos_;
	}
	
	
	void SMC::Update_Real_Spd(float real_spd_)
	{	
		real_spd = real_spd_;
	}
	
	
	float SMC::SMC_calculate()
	{
		// 计算误差
        error = target_spd - real_spd;
		error_sum += error;

		
		if (error_sum > 1600) error_sum = 1600;
		else if (error_sum < -1600) error_sum = -1600;
  
        float s = -(error) - lambda * error_sum;


        float sat_val = sat(s, delta);


        float eta = 0.8f * lambda;
        output = -k * sat_val - eta * s;


        if (output > output_limit) output = output_limit;
		else if (output < -output_limit) output = -output_limit;
		
		return output;
	}
	
	
	float SMC::Get_Output()
	{
		return output;
	}
	
	
	float sat(float s, float delta)
	{
//		// 确保边界层厚度为正数（容错处理）
//		if (delta <= 0.0f)
//		{
//			delta = 0.001f;  // 默认极小值，避免除零
//		}

//		if (fabs(s) <= delta)
//		{
//			// 边界层内：线性特性，连续过渡
//			return s / delta;
//		}
//		else
//		{
//			// 边界层外：符号函数特性，保证鲁棒性
//			return (s > 0.0f) ? 1.0f : -1.0f;
//		}
		return s / (fabsf(s) + delta);
	}
}