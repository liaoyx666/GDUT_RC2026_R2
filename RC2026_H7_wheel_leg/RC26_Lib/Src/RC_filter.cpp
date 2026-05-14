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
	
	float TD::TD_Calculate(float v, bool normalization, float unit)
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
		
		v2 = v2_last + h * adrc::fst(error, v2_last, r, h);

		// 限制速度
		if (v2_max != 0.f && fabsf(v2) > v2_max)
		{
			v2 = v2_max * adrc::sgn(v2);
		}
		
		v1_last = v1;
		v2_last = v2;
		
		return v1;
	}
	/**************************************************************************/
	
	
}