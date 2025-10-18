#include "RC_filter.h"

namespace filter
{
	TD::TD()
	{
	
	}
	
	
	TD::TD(float r_, float h_) : r(r_), h(h_)
	{
		
	}
	
	void TD::TD_Init(float r_, float h_)
	{
		r = r_;
		h = h_;
	}
	
	
	float TD::TD_Calculate(float v)
	{
		v1 = v1_last + h * v2_last;
		v2 = v2_last + h * adrc::fhan(v1_last - v, v2_last, r, h);
		
		v1_last = v1;
		v2_last = v2;
		
		return v1;
	}
}