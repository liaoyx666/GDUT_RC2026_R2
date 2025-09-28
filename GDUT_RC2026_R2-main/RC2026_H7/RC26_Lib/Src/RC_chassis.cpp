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
        target_pos_x = target_pos_y = target_pos_yaw = 0.0f;
        target_spd_x = target_spd_y = target_spd_yaw = 0.0f;
		
		
		max_xy_spd = fabsf(max_xy_spd_);
		max_yaw_spd = fabsf(max_yaw_spd_);
		
		Chassis_Init();
    }

    // 底盘初始化
    void OmniChassis::Chassis_Init()
    {
       
    }

    // 设置位置目标
    void OmniChassis::Set_Chassis_Pos(float x, float y, float yaw)
    {
        target_pos_x = x;
        target_pos_y = y;
        target_pos_yaw = yaw;
    }

    // 设置速度目标
    void OmniChassis::Set_Chassis_Spd(float x, float y, float yaw)
    {
		float spd = sqrtf(powf(x, 2) + powf(y, 2));
		if (spd > max_xy_spd && spd != 0)
		{
			target_spd_x = x * max_xy_spd / spd;
			target_spd_y = y * max_xy_spd / spd;
		}
		else
		{
			target_spd_x = x;
			target_spd_y = y;
		}
		
		if (yaw > max_yaw_spd) target_spd_yaw = max_yaw_spd;
		else if (yaw < -max_yaw_spd) target_spd_yaw = -max_yaw_spd;
		else target_spd_yaw = yaw;
    }

    // 带加速度限制的速度计算
    float OmniChassis::Limit_Accel(float delta, float max_acc, float dt)
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
		uint32_t delta_time = timer::Timer::Get_DeltaTime(last_time);
		float dt = (float)delta_time / 1000000.f;// us->s
		last_time = timer::Timer::Get_TimeStamp();
		
        // 加速度限制
		float target_delta_spd = sqrtf(powf(target_spd_x - last_spd_x, 2) + powf(target_spd_y - last_spd_y, 2));
		float current_delta_spd = Limit_Accel(target_delta_spd, MAX_ACC_SPEED, dt);
		
		float current_spd_x = 0;
		float current_spd_y = 0;
		
		if (target_delta_spd != 0)
		{
			current_spd_x = last_spd_x + current_delta_spd * (target_spd_x - last_spd_x) / target_delta_spd;
			current_spd_y = last_spd_y + current_delta_spd * (target_spd_y - last_spd_y) / target_delta_spd;
		}
		else
		{
			current_spd_x = last_spd_x;
			current_spd_y = last_spd_y;
		}
		
		target_delta_spd = target_spd_yaw - last_spd_yaw;
		current_delta_spd = Limit_Accel(target_delta_spd, MAX_ACC_YAW, dt);
		
		float current_spd_yaw = last_spd_yaw + current_delta_spd;


        // 更新上一周期速度
        last_spd_x = current_spd_x;
        last_spd_y = current_spd_y;
        last_spd_yaw = current_spd_yaw;
        
        // 三轮全向轮运动学解算
        float motor_rpm[3] = 
        {
            // 60
            (current_spd_x * COS60 + current_spd_y * SIN60 + L * current_spd_yaw) * k,
            // 180
            (current_spd_x * COS180 + current_spd_y * SIN180 + L * current_spd_yaw) * k,
            // 300
            (current_spd_x * COS300 + current_spd_y * SIN300 + L * current_spd_yaw) * k
        };

		
        
        for (int i = 0; i < 3; i++)
        {
			// 设置电机转速
			motor[i]->Set_Rpm(motor_rpm[i]);
        }
    }
}
    
