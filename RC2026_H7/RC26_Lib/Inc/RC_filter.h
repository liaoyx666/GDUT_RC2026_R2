#pragma once
#include "RC_adrc.h"
// Header: 滤波器
// File Name: 
// Author:
// Date:

#ifdef __cplusplus
namespace filter
{
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
}
#endif
