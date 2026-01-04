#pragma once
#include <math.h>
// Header: 滑模控制
// File Name: 
// Author: 
// Date: 2025/10/25
#ifdef __cplusplus
namespace smc
{
	class SMC
    {
    public:
		SMC();
		virtual ~SMC() {}
		
		void SMC_Init(
			float output_limit_,
			float lambda_,
			float k_,
			float delta_,
			float dt_
		);
		
		void Update_Target_Spd(float target_spd_);
		void Update_Real_Pos(float real_pos_);
		void Update_Real_Spd(float real_spd_);
		float SMC_calculate();
		float Get_Output();	
		
		
			
    protected:
		
    private:
		float target_spd = 0;
		float real_pos = 0;
		float real_spd = 0;
		float error = 0;
		float output = 0;
		float error_sum = 0;

		float output_limit = 0;	// 输出限幅
		float lambda = 0;   	// 滑模面系数（影响收敛速度）
        float k = 0;        	// 趋近律增益（增强鲁棒性）
        float delta = 0;    	// 边界层厚度（抑制抖振）
        float dt = 0.001;   	// 控制周期
    };
	
	float sat(float s, float delta);
}
#endif
