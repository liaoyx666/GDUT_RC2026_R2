#pragma once
#include <math.h>
#include "RC_pid.h"

#ifdef __cplusplus
namespace pid
{
	class NonlinearPid
    {
    public:
		NonlinearPid(float kp_, float accel_, float delta_, float max_out_, float deadzone_ = 0);
		NonlinearPid();
		virtual ~NonlinearPid() {}
		
		float NPid_Calculate(float target_, float real_, bool normalization = false, float unit = PI);
		
		void Set_Accel(float accel_)
		{
			Init(kp, accel_, delta, max_out);
		}
		
		void Set_Kp(float kp_)
		{
			Init(kp_, two_accel / 2.f, delta, max_out);
		}
		
		void Set_Delta(float delta_)
		{
			Init(kp, two_accel / 2.f, delta_, max_out);
		}
		
		void Set_Max_Out(float max_out_)
		{
			Init(kp, two_accel / 2.f, delta, max_out_);
		}
		
		void Init(float kp_, float accel_, float delta_, float max_out_, float deadzone_ = 0);
    protected:
		
    private:
		float deadzone = 0;
	
		float kp;
		float two_accel;
		float delta;
		float max_out;

		float x0;
		float y0;
	
    };
}
#endif
