#include "RC_filter.h"

namespace filter
{
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
	
	
	float TD::TD_Calculate(float v)
	{
		v1 = v1_last + h * v2_last;
		v2 = v2_last + h * adrc::fst(v1_last - v, v2_last, r, h);
		
		// 限制速度
		if (v2_max != 0.f && fabsf(v2) > v2_max)
		{
			v2 = v2_max * adrc::sgn(v2);
		}
		
		v1_last = v1;
		v2_last = v2;
		
		return v1;
	}
}