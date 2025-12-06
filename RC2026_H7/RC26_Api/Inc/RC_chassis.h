#pragma once
#include "RC_motor.h"
#include "RC_task.h"
#include "RC_timer.h"
#include "RC_vector2d.h"

#include <math.h>

#ifdef __cplusplus
namespace chassis
{
	class Chassis : public task::ManagedTask
    {
    public:
		Chassis(
			float max_linear_vel_, float linear_accel_, float linear_decel_,
			float max_angular_vel_, float angular_accel_, float angular_decel_
		);
		virtual ~Chassis() {}
		
		void Set_Robot_Vel(vector2d::Vector2D v_, float vw_);
		void Set_World_Vel(vector2d::Vector2D v_, float vw_, float yaw_);
		
    protected:
		virtual void Kinematics_calc() = 0;
		
		vector2d::Vector2D target_v;
		float target_vw = 0;
	
		vector2d::Vector2D v;
		float vw = 0;
	
		vector2d::Vector2D last_v;
		float last_vw = 0;
	
    private:
		void Task_Process() override;
		
		bool is_init = false;
		bool is_enable = false;
	
		float max_linear_vel = 0;
		float linear_accel = 0;
		float linear_decel = 0;
	
		float max_angular_vel = 0;
		float angular_accel = 0;
		float angular_decel = 0;
	
		uint32_t last_time = 0;

    };

	float Limit_Accel(float delta_spd, float max_acc, float dt);
}
#endif
