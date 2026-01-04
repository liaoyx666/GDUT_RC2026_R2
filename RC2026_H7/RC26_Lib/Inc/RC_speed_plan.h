#pragma once
#include <math.h>
#include "arm_math.h"

#ifdef  __cplusplus
namespace speedplan
{
	class TrapSpeedPlan
    {
    public:
		TrapSpeedPlan(float max_accel_, float max_decel_, float max_vel_, float min_vel_ = 0, float deadzone_ = 0);
		virtual ~TrapSpeedPlan() {}

		bool Generate(float distance_, float start_vel_ = 0, float vel_ = 0);
		float Get_Vel(float current_distance_);
		
    protected:
		
    private:
		float max_accel = 0;
		float max_decel = 0;
		float max_vel = 0;
		float min_vel = 0;
		float deadzone = 0;
	
		float start_vel = 0;
		float vel = 0;
	
		float distance = 0;
	
		float accel_s = 0;
		float decel_s = 0;
    };
}
#endif
