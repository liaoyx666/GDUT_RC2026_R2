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
		
		// 设置机器人坐标速度
		void Set_Robot_Vel(vector2d::Vector2D v_, float vw_);
		
		// 设置世界坐标送速度
		void Set_World_Vel(vector2d::Vector2D v_, float vw_, float yaw_);
			
		vector2d::Vector2D Get_Vel() const {return v;}
		
		// 使能
		void Chassis_Enable() {is_enable = true;}
		
		// 失能
		void Chassis_Disable() {is_enable = false;}
		
    protected:
		// 成功初始化
		void Set_Is_Init_True() {is_init = true;}
		
		// 成功初始化
		void Set_Is_Init_False() {is_init = false;}
		
		// 再次初始化
		virtual void Chassis_Re_Init() = 0;
		
		// 底盘初始化
		virtual void Chassis_Init() = 0;
	
		// 运动学解算并设置电机
		virtual void Kinematics_calc(vector2d::Vector2D v_, float vw_) = 0;
		
    private:
		// 底盘任务
		void Task_Process() override;
		
		// 目标速度
		vector2d::Vector2D target_v;
		float target_vw = 0;
		
		// 真实速度
		vector2d::Vector2D v;
		float vw = 0;
	
		// 上一次速度
		vector2d::Vector2D last_v;
		float last_vw = 0;
	
		// 线速度
		float max_linear_vel = 0;
		float linear_accel = 0;
		float linear_decel = 0;
		
		// 角速度
		float max_angular_vel = 0;
		float angular_accel = 0;
		float angular_decel = 0;
	
		// 时间戳
		uint32_t last_time = 0;
		
		bool is_init = false;
		bool is_enable = true;
    };
	
	// 加速度限制
	float Limit_Accel(float delta_spd, float max_acc, float dt);
}
#endif
