#include "RC_adrc.h"





namespace adrc
{
	ADRC::ADRC()
	{
	
	}
	
	void ADRC::ADRC_Param_Init(
		float output_limit_,// 输出限幅
		float r_,			// 快速跟踪因子
		float h_,			// 滤波因子，系统调用步长
		float b_,			// 系统系数
		float delta_,		// fal函数的线性区间宽度
		float beta_01_,		// 扩张状态观测器反馈增益1
		float beta_02_,		// 扩张状态观测器反馈增益2
		float beta_03_,		// 扩张状态观测器反馈增益3
		float alpha_1_,		// 非线性因子1
		float alpha_2_,		// 非线性因子2
		float beta_1_,		// 跟踪输入信号增益kp
		float beta_2_		// 跟踪微分信号增益kd
	)
	{
		output_limit = output_limit_;	

		r = r_;
		h = h_;

		b = b_;
		delta = delta_;
		beta_01 = beta_01_;
		beta_02 = beta_02_;
		beta_03 = beta_03_;
						  
		alpha_1 = alpha_1_;
		alpha_2 = alpha_2_;
		beta_1 = beta_1_;
		beta_2 = beta_2_;
	}
	
	

	float ADRC::ADRC_Calculate(bool normalization, float unit)
	{
	
		// 跟踪微分器TD
		v1 = v1_last + h * v2_last;
		v2 = v2_last + h * fst(v1_last - v, v2_last, r, h);
		
		v1_last = v1;
		v2_last = v2;
		

		// 扩张观测器ESO
		float e = z1 - y;
		
		z1 = z1 + h * (z2 - beta_01 * e);
		z2 = z2 + h * (z3 - beta_02 * fal(e, alpha_1, delta) + b * u);
		z3 = z3 - h * beta_03 * fal(e, alpha_2, delta);
		
		
		// 非线性组合NLSEF
		e1 = v1 - z1;
		e2 = v2 - z2;

		e2 = e2_last * 0.3f + e2 * 0.7f;

		e2_last = e2;


		u = beta_1 * fal(e1, alpha_1, delta) + beta_2 * fal(e2, alpha_2, delta);
		
		
		
		// 扰动补偿
		u0 = u - z3 / b;
		
		
		// 输出限幅
		if (u0 > output_limit) u0 = output_limit;
		else if (u0 < -output_limit) u0 = -output_limit;
		
		
		u0 = u0_last * 0.3f + u0 * 0.7f;
		
		u0_last = u0;
		
		return u0;
	}
	
	
	/**
    * @brief 最速控制综合函数
    * @note 
    * @param x:
    * @retval 
    */
	float fst(float x1_, float x2_, float r_, float h_)
	{
		float d = r_ * h_;
		float d0 = h_ * d;
		float y = x1_ + h_ * x2_;
		
		float a0 = sqrtf(d * d + 8.f * r_ * fabsf(y));
		float a;
		
		if (fabsf(y) > d0)
		{
			a = x2_ + (a0 - d) / 2.f * sgn(y);
		}
		else
		{
			a = x2_ + y / h_;
		}
		
		if (fabsf(a) > d)
		{
			return -r_ * sgn(a);
		}
		else
		{
			return -r_ * a / d;
		}
	}
	
	
	
	
	float fhan(float x1, float x2, float r, float h)
	{
		float d = r * h * h;
		float a0 = h * x2;
		float y = x1 + a0;
		float a1 = sqrtf(d * (d + 8.f * fabsf(y)));
		float a2 = a0 + sgn(y) * (a1 - d) / 2.f;
		return -r * (a2 / d) * (fabsf(a2) <= d) + -r * sgn(a2) * (fabsf(a2) > d);
	}
	
	
	
	
	
	
	
	
	
	
	
	/**
    * @brief  饱和函数
    * @note 
    * @param delta_:线性段区间长度
    * @retval 
    */
	float fal(float e_, float alpha_, float delta_)
	{
		if (fabsf(e_) <= delta_)
		{
			return e_ / powf(delta_, alpha_ - 1.f);
		}
		else
		{
			return powf(fabsf(e_), alpha_) * sgn(e_);
		}
	}
	
	
	
	/**
    * @brief  符号函数
    * @note 
    * @param x_:
    * @retval 
    */
	float sgn(float x_)
	{
		if (x_ > 0.f)
		{
			return 1.f;
		}
		else if (x_ == 0.f)
		{
			return 0.f;
		}
		else
		{
			return -1.f;
		}
	}
}