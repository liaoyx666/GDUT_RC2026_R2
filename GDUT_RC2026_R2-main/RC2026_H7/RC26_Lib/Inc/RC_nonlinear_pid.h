#pragma once
#include <math.h>
#include "RC_pid.h"
#include "RC_timer.h"

#ifdef __cplusplus
namespace pid
{
	class NonlinearPid
    {
    public:
		NonlinearPid(float kp_, float kd, float accel_, float delta_, float max_out_, float deadzone_);
		NonlinearPid();
		~NonlinearPid() = default;
		
		float NPid_Calculate(float target_, float real_, bool normalization = false, float unit = PI);
		
		void Set_Accel(float accel_)
		{
			Init(kp, kd ,accel_, delta, max_out, deadzone);
		}
		
		void Set_Kp(float kp_)
		{
			Init(kp_, kd, two_accel / 2.f, delta, max_out, deadzone);
		}
		
		void Set_Kd(float kd_)
		{
			kd = kd_;
		}
		
		void Set_Delta(float delta_)
		{
			Init(kp, kd, two_accel / 2.f, delta_, max_out, deadzone);
		}
		
		void Set_Max_Out(float max_out_)
		{
			max_out = max_out_;
		}
		
		void Set_Deadzone(float dz)
		{
			deadzone = dz;
		}
		
		void Init(float kp_, float kd, float accel_, float delta_, float max_out_, float deadzone_);
    protected:
		
    private:
		float deadzone = 0;
	
		float kp;
		float kd;
		float two_accel;
		float delta;
		float max_out;

		float x0;
		float y0;
		float last_error;
		uint32_t last_time;
	
    };
}
#endif
