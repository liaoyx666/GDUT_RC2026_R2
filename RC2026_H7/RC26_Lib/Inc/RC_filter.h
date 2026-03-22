#pragma once
// Header: 滤波器
// File Name: 
// Author:
// Date:
#include "math.h"
#include "arm_math.h"

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
namespace filter
{
	// 跟踪微分器TD
	class TD
    {
    public:
		TD(float r_, float h_, float v2_max_ = 0.f);
		TD();
	
		float TD_Calculate(float v, bool normalization = false, float unit = PI, float* v2_return = NULL);
		void TD_Init(float r_, float h_, float v2_max_ = 0.f);
		void Set_R(float r_) {float r = r_;}
		void Set_V2_Max(float v2_max_) {float v2_max = v2_max_;}
	
    protected:
		
    private:
		float r = 0;
		float h = 0;
	
		float v1_last = 0;
		float v2_last = 0;
		
		float v2_max = 0;
    };
	
	class SecondOrderLPF
	{
	public:
		SecondOrderLPF(float fc, float fs, float zeta = 0.707f);

		void reset();

		float filter(float x);

	private:
		/**
		 * @brief 内部方法：计算离散化后的滤波器系数（双线性变换）
		 */
		void calculateCoefficients();

		// 滤波器核心参数
		float fc_;          // 截止频率 (Hz)
		float fs_;          // 采样频率 (Hz)
		float zeta_;        // 阻尼比
		float Ts_;          // 采样周期 (s) = 1/fs_
		float omega_n_;     // 自然频率 (rad/s) = 2*PI*fc_

		// 离散化系数
		float a0_;
		float a1_;
		float a2_;
		float b1_;
		float b2_;

		// 滤波器状态（保存前2次的输入/输出，避免重复计算）
		float x_prev1_;     // x(k-1)
		float x_prev2_;     // x(k-2)
		float y_prev1_;     // y(k-1)
		float y_prev2_;     // y(k-2)
	};
	
	
	
	float fal(float e_, float alpha_, float delta_);
	float fst(float x1_, float x2_, float r_, float h_);
	float sgn(float x_);
	float fhan(float x1, float x2, float r, float h);
	
	
	
//	class TD3rd
//    {
//    public:
//		TD3rd(float r_, float h_, float v2_max_ = 0.f, float v3_max_ = 0.f);
//		TD3rd();
//		virtual ~TD3rd() {}
//			
//		void TD3rd_Init(float r_, float h_, float v2_max_ = 0.f, float v3_max_ = 0.f);
//		
//		float TD_Calculate(float v, bool normalization = false, float unit = PI);
//    protected:
//		
//    private:
//		float v1, v2, v3;  // 位置、速度、加速度状态
//		float v1_last, v2_last, v3_last;  // 上一时刻状态
//		float h;           // 采样时间（必须与控制周期一致）
//		float r;           // 快速跟踪因子（调大则响应快，可能震荡）
//		float v2_max;      // 最大速度限制（S型曲线的速度上限）
//		float v3_max;      // 最大加速度限制（S型曲线的加速度上限）
//    };
	
}
#endif
