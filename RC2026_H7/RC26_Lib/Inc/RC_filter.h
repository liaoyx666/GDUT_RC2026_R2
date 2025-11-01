#pragma once
#include "RC_adrc.h"
// Header: 滤波器
// File Name: 
// Author:
// Date:

#ifdef __cplusplus
namespace filter
{
	// 跟踪微分器TD
	class TD
    {
    public:
		TD(float r_, float h_, float v2_max_ = 0.f);
		TD();
		float TD_Calculate(float v);
		void TD_Init(float r_, float h_, float v2_max_ = 0.f);
	
    protected:
		
    private:
		float r = 0;
		float h = 0;
	
		float v1 = 0;
		float v2 = 0;
	
		float v1_last = 0;
		float v2_last = 0;
		
		float v2_max = 0;
	
    };
	
	
	class TD3rd
    {
    public:
		TD3rd(float r_, float h_, float v2_max_ = 0.f, float v3_max_ = 0.f);
		TD3rd();
		virtual ~TD3rd() {}
			
		void TD3rd_Init(float r_, float h_, float v2_max_ = 0.f, float v3_max_ = 0.f);
		
			
		float TD_Calculate(float v);
    protected:
		
    private:
		float v1, v2, v3;  // 位置、速度、加速度状态
		float v1_last, v2_last, v3_last;  // 上一时刻状态
		float h;           // 采样时间（必须与控制周期一致）
		float r;           // 快速跟踪因子（调大则响应快，可能震荡）
		float v2_max;      // 最大速度限制（S型曲线的速度上限）
		float v3_max;      // 最大加速度限制（S型曲线的加速度上限）
    };
	
	
	
	
	
	
	
	
	
}
#endif
