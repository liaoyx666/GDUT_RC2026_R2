#include "RC_chassis.h"

namespace chassis
{
    // 构造函数（三轮）
    OmniChassis::OmniChassis(motor::M3508 &wheel_left, motor::M3508 &wheel_mid, motor::M3508 &wheel_right, float max_xy_spd_, float max_yaw_spd_) : ManagedTask("ChassisTask", 15, 128, task::TASK_PERIOD, 1)
    {
		motor[0] = &wheel_left;// 60
        motor[1] = &wheel_mid;// 180
        motor[2] = &wheel_right;// 300
		
        // 检查电机指针有效性
        if (motor[0] == nullptr || motor[1] == nullptr || motor[2] == nullptr) Error_Handler();
	
        // 初始化目标值
        target_spd_yaw = 0.0f;
		
		max_xy_spd = fabsf(max_xy_spd_);
		max_yaw_spd = fabsf(max_yaw_spd_);
		
		Chassis_Init();
    }

    // 底盘初始化
    void OmniChassis::Chassis_Init()
    {
       
    }


	void OmniChassis::Set_Chassis_World_Spd(float target_x, float target_y, float target_yaw, float yaw)
	{
		vector2d::Vector2D spd = vector2d::Vector2D(target_x, target_y);
		
		vector2d::Vector2D world_spd = spd.rotate(yaw);
		
		Set_Chassis_Spd(world_spd.data()[0], world_spd.data()[1], target_yaw);
	}
	
	
    // 设置速度目标
    void OmniChassis::Set_Chassis_Spd(float x, float y, float yaw)
    {
		float spd = sqrtf(powf(x, 2) + powf(y, 2));
		
		if (spd > max_xy_spd && spd != 0)
		{
			target_spd = vector2d::Vector2D(x * max_xy_spd / spd, y * max_xy_spd / spd);
		}
		else
		{
			target_spd = vector2d::Vector2D(x, y);
		}
		
		if (yaw > max_yaw_spd) target_spd_yaw = max_yaw_spd;
		else if (yaw < -max_yaw_spd) target_spd_yaw = -max_yaw_spd;
		else target_spd_yaw = yaw;
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


    // 运动控制（三轮全向轮解算）
    void OmniChassis::Chassis_Calc()
    {
		/*----------------------------------------计算dt-------------------------------------------------*/
		
		float dt = (float)timer::Timer::Get_DeltaTime(last_time) / 1000000.f;// us->s
		last_time = timer::Timer::Get_TimeStamp();
		
		
		
		
		
		
		
		
		/*-----------------------------------------yaw加速度限制------------------------------------------------*/
		float target_delta_spd;// 目标变化量
		float current_delta_spd;// 实际变化量

		target_delta_spd = target_spd_yaw - last_spd_yaw;
		
		current_delta_spd = Limit_Accel(target_delta_spd, MAX_ACC_YAW, dt);
		
		current_spd_yaw = last_spd_yaw + current_delta_spd;
		
		
		
		
		
		/*------------------------------------------前馈----------------------------------------------*/
		target_spd = target_spd.rotate(current_spd_yaw * 0.1f);



		/*-------------------------------------------xy加速度限制----------------------------------------------*/

		float delta_yaw = (last_spd_yaw + current_spd_yaw) * 0.5f * dt;// 计算上一周期到现在，底盘转动角度
		
		vector2d::Vector2D real_last_spd = last_spd.rotate(delta_yaw);// 计算上一周期相对当前机器人坐标系的速度向量
		
		vector2d::Vector2D delta_spd = target_spd - real_last_spd;// 计算目标速度差向量
		
		target_delta_spd = delta_spd.length();// 计算目标速度变化量
		
		if (target_delta_spd < 1e-6f)
		{
			current_spd = real_last_spd;
		}
		else
		{
			current_delta_spd = Limit_Accel(target_delta_spd, MAX_ACC_SPEED, dt);// 限制速度变化量
			
			current_spd = real_last_spd + delta_spd * (current_delta_spd / target_delta_spd);// 加速度限制
		}
		
		/*--------------------------------------------解算---------------------------------------------*/
		
        // 三轮全向轮运动学解算
        float motor_rpm[3] = 
        {
            (current_spd.data()[0] * COS60  - current_spd.data()[1] * SIN60  + L * current_spd_yaw) * k,// 60
            (current_spd.data()[0] * COS180 - current_spd.data()[1] * SIN180 + L * current_spd_yaw) * k,// 180
            (current_spd.data()[0] * COS300 - current_spd.data()[1] * SIN300 + L * current_spd_yaw) * k // 300
        };


        // 设置电机转速
        for (int i = 0; i < 3; i++)
        {
			motor[i]->Set_Rpm(motor_rpm[i]);
        }
		
		
		// 更新上一周期速度
		last_spd_yaw = current_spd_yaw;
		last_spd = current_spd;
    }
}
    
