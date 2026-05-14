#include "RC_filter.h"

namespace filter
{
	/**************************************************************************/
	TD::TD()
	{
	}
	
	TD::TD(float r_, float h_, float v2_max_) : r(r_), h(h_), v2_max(v2_max_)
	{
	}
	
	void TD::TD_Init(float r_, float h_, float v2_max_)
	{
		r = r_;
		h = h_;
		v2_max = v2_max_;
	}
	
	float TD::TD_Calculate(float v, bool normalization, float unit, float* v2_return)
	{
		float period = 2.f * unit;
		
		float v1 = 0;
		float v2 = 0;
		
		// 归一化到-unit ~ unit
		if (normalization == true)
		{
            v = fmodf(v, period);
            if (v > unit)
			{
                v -= period;
            }
			else if (v < -unit)
			{
				v += period;
			}
        }
		
		v1 = v1_last + h * v2_last;
		
		float error = v1_last - v;
		
		// 归一化到-unit ~ unit
		if (normalization == true)
		{
            error = fmodf(error, period);
            if (error > unit)
			{
                error -= period;
            }
			else if (error < -unit)
			{
				error += period;
			}
			
			v1 = fmodf(v1, period);
			
            if (v1 > unit)
			{
                v1 -= period;
            }
			else if (v1 < -unit)
			{
				v1 += period;
			}
        }
		
		v2 = v2_last + h * fst(error, v2_last, r, h);

		// 限制速度
		if (v2_max != 0.f && fabsf(v2) > v2_max)
		{
			v2 = v2_max * sgn(v2);
		}
		
		v1_last = v1;
		v2_last = v2;
		
		// 输出v2
		if (v2_return != NULL)
		{
			*v2_return = v2;
		}
		
		return v1;
	}
	/**************************************************************************/
	
	// 构造函数实现
	SecondOrderLPF::SecondOrderLPF(float fc, float fs, float zeta) : fc_(fc), fs_(fs), zeta_(zeta)
	{
		//采样频率fs必须大于0

		//截止频率fc必须满足 0 < fc < fs/2

		//阻尼比zeta必须≥0

		// 初始化采样周期
		Ts_ = 1.0f / fs_;

		// 双线性变换预扭曲：补偿频率畸变
		float omega_n_continuous = 2 * PI * fc_;          // 连续域自然频率
		omega_n_ = (2.0f / Ts_) * tan(omega_n_continuous * Ts_ / 2.0f); // 预扭曲后的自然频率

		// 计算离散化系数
		calculateCoefficients();

		// 初始化滤波器状态（避免野值）
		reset();
	}

	// 重置滤波器状态
	void SecondOrderLPF::reset()
	{
		x_prev1_ = 0.0f;
		x_prev2_ = 0.0f;
		y_prev1_ = 0.0f;
		y_prev2_ = 0.0f;
	}

	// 单次滤波计算（核心方法）
	float SecondOrderLPF::filter(float x)
	{
		// 二阶低通递推公式：y(k) = a0*x(k) + a1*x(k-1) + a2*x(k-2) - b1*y(k-1) - b2*y(k-2)
		float y = a0_ * x + a1_ * x_prev1_ + a2_ * x_prev2_ - b1_ * y_prev1_ - b2_ * y_prev2_;

		// 更新状态（保存当前输入/输出到历史，供下一次计算使用）
		// 顺序必须正确：先更新k-2，再更新k-1
		x_prev2_ = x_prev1_;
		x_prev1_ = x;
		y_prev2_ = y_prev1_;
		y_prev1_ = y;

		return y;
	}

	// 计算离散化系数（双线性变换）
	void SecondOrderLPF::calculateCoefficients()
	{
		float Ts = Ts_;
		float omega_n = omega_n_;
		float zeta = zeta_;

		// 双线性变换核心系数计算
		float A = 4.0f + 4.0f * zeta * omega_n * Ts + pow(omega_n * Ts, 2);
		a0_ = pow(omega_n * Ts, 2) / A;
		a1_ = 2.0f * pow(omega_n * Ts, 2) / A;
		a2_ = pow(omega_n * Ts, 2) / A;
		b1_ = (2.0f * pow(omega_n * Ts, 2) - 8.0f) / A;
		b2_ = (4.0f - 4.0f * zeta * omega_n * Ts + pow(omega_n * Ts, 2)) / A;
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