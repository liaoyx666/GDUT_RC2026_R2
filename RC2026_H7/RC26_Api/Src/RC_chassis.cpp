#include "RC_chassis.h"

namespace chassis
{
	Chassis::Chassis(
		float max_linear_vel_, float linear_accel_, float linear_decel_,
		float max_angular_vel_, float angular_accel_, float angular_decel_
	) : ManagedTask("ChassisTask", 15, 512, task::TASK_PERIOD, 1)
	{
		max_linear_vel = fabsf(max_linear_vel_);
		linear_accel = fabsf(linear_accel_);
		linear_decel = fabsf(linear_decel_);
		
		max_angular_vel = fabsf(max_angular_vel_);
		angular_accel = fabsf(angular_accel_);
		angular_decel = fabsf(angular_decel_);
		
		is_init = false;
		is_enable = true;
		
		last_time = 0;
	}
	
	void Chassis::Task_Process()
	{
		/*************************************************/
		if (is_init == false || is_enable == false)
		{
			target_v = vector2d::Vector2D(0, 0);
			target_vw = 0;
		}
		
		/************************速度限幅*************************/
		float v_length = target_v.length();
		float vw_length = fabsf(target_vw);
		
		if (v_length > max_linear_vel)
		{
			target_v = target_v / v_length * max_linear_vel;
		}
		
		if (vw_length > max_angular_vel)
		{
			if (target_vw > 0)
			{
				target_vw = max_angular_vel;
			}
			else
			{
				target_vw = -max_angular_vel;
			}
		}
		
		/**************************获取dt***********************/
		float dt = (float)timer::Timer::Get_DeltaTime(last_time) / 1000000.f;// us->s
		last_time = timer::Timer::Get_TimeStamp();
		if (dt <= 0 || dt > 0.1f) dt = 0.01f;

		/*************************角加速度限制************************/
		// 判断是否需要减速
		if (target_vw * last_vw < 0)
		{
			vw = last_vw + Limit_Accel(target_vw - last_vw, angular_decel, dt);
		}
		else
		{
			vw = last_vw + Limit_Accel(target_vw - last_vw, angular_accel, dt);
		}	
		/************************真实前一次速度方向*************************/
		last_v = last_v.rotate(-(vw + last_vw) / 2.f * dt);
		
		/************************线加速度限制*************************/
		vector2d::Vector2D normal_v = target_v.perpendicular();// 获取垂直法向量（逆时针90度）
		
		float v_x;// 切向目标速度
		// v_y = 0// 法向目标速度
		
		vector2d::Vector2D x_unit;// 切单位向量
		vector2d::Vector2D y_unit;// 法单位向量
		
		float last_v_x;// 上次切向速度
		float last_v_y;// 上次法向速度
		
		if (vector2d::Vector2D::isZero(target_v.lengthSquared()))
		{
			// 切向目标速度为0
			v_x = 0;
			
			if (vector2d::Vector2D::isZero(last_v.lengthSquared()))
			{
				x_unit = vector2d::Vector2D(1, 0);
				y_unit = vector2d::Vector2D(0, 1);
				last_v_x = 0;
				last_v_y = 0;
			}
			else
			{
				x_unit = last_v.normalize();
				y_unit = x_unit.perpendicular();
				
				last_v_x = last_v.length();
				last_v_y = 0;
			}
		}
		else
		{
			// 计算切向目标速度
			v_x = target_v.length();
			
			x_unit = target_v.normalize();
			y_unit = x_unit.perpendicular();
			
			last_v_x = last_v.projectLength(target_v);
			last_v_y = last_v.projectLength(normal_v);
		}
		
		// 切向减速度控制
		last_v_y = last_v_y + Limit_Accel(-last_v_y, linear_decel, dt);
		
		// 法向加，减速度控制
		if (last_v_x < 0)
		{
			last_v_x = last_v_x + Limit_Accel(v_x - last_v_x, linear_decel, dt);
		}
		else
		{
			last_v_x = last_v_x + Limit_Accel(v_x - last_v_x, linear_accel, dt);
		}
		
		// 速度合成
		v = (x_unit * last_v_x) + (y_unit * last_v_y);

		/*************************************************/
		// 更新上次速度
		last_v = v;
		last_vw = vw;

		if (is_init == false && vector2d::Vector2D::isZero(v.lengthSquared()) && fabsf(vw) < 1e-6)
		{
			// 初始化底盘（底盘需静止）
			Chassis_Init();
		}
		else
		{
			// 运动学解算并设置电机
			Kinematics_calc(v, vw);
		}
		/*************************************************/
	}
	
	void Chassis::Set_Robot_Vel(vector2d::Vector2D v_, float vw_)
	{
		if (is_init == false || is_enable == false)
		{
			target_v = vector2d::Vector2D(0, 0);
			target_vw = 0;
		}
		else
		{
			target_v = v_;
			target_vw = vw_;
		}
	}
	
	void Chassis::Set_World_Vel(vector2d::Vector2D v_, float vw_, float yaw_)
	{
		Set_Robot_Vel(v_.rotate(-yaw_), vw_);
	}
	
	// 加速度限制
    float Limit_Accel(float delta, float max_acc, float dt)
    {
        float max_delta = max_acc * dt;  // 最大速度增量
      
        // 限制速度增量
        if (delta > max_delta)  delta = max_delta;
        else if (delta < -max_delta) delta = -max_delta;
        
        return delta;
    }
}
