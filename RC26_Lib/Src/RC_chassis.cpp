#include "RC_chassis.h"

namespace chassis
{
			
    // 构造函数
				RC_Chassis::RC_Chassis(
        m3508::M3508* m3508_1, 
        m3508::M3508* m3508_2, 
        m3508::M3508* m3508_3, 
        m3508::M3508* m3508_4
    )
    {
        if (m3508_1 == nullptr || m3508_2 == nullptr || m3508_3 == nullptr || m3508_4 == nullptr)
        Error_Handler();

        motor[0] = m3508_1;
        motor[1] = m3508_2;
        motor[2] = m3508_3;
        motor[3] = m3508_4;

        target_x = target_y = target_yaw = 0.0f;
        target_x_spd = target_y_spd = target_yaw_spd = 0.0f;
    }

    // 底盘初始化
    void RC_Chassis::chassis_init()
    {
        // 位置环PID初始化（如需要位置控制）
        // pid_x.Pid_Mode_Init(...);
        // pid_x.Pid_Param_Init(...);
    }

    // 设置位置目标
    void RC_Chassis::Set_Target_Pos(float x, float y, float yaw)
    {
        target_x = x;
        target_y = y;
        target_yaw = yaw;
        
    }

    // 设置速度目标
    void RC_Chassis::Set_Target_Spd(float x, float y, float yaw)
    {
        target_x_spd = x;
        target_y_spd = y;
        target_yaw_spd = yaw;
    }

    // 带加速度限制的速度计算
    float RC_Chassis::limit_acceleration(float target_spd, float last_spd, float max_acc)
    {
        // 1. 计算当前周期允许的最大速度增量（Δv = a × dt）
        float max_delta = max_acc * dt;
        
        // 2. 计算速度差值（目标 - 上一速度）
        float delta = target_spd - last_spd;
        
        // 3. 限制速度增量：若差值超过最大增量，取最大增量；否则取实际差值
        if (delta > max_delta)
            delta = max_delta;  // 加速时不超过最大增量
        else if (delta < -max_delta)
            delta = -max_delta; // 减速时不超过最大增量（负增量）
        
        // 4. 返回当前周期的实际速度（上一速度 + 限制后增量）
        return last_spd + delta;
    }

    // 运动控制
    void RC_Chassis::Update()
    {
        // 加速度限制
        // 计算X/Y/Yaw轴的实际执行速度（避免速度突变）
        float current_x_spd = limit_acceleration(target_x_spd, last_x_spd, MAX_ACC_X);
        float current_y_spd = limit_acceleration(target_y_spd, last_y_spd, MAX_ACC_Y);
        float current_yaw_spd = limit_acceleration(target_yaw_spd, last_yaw_spd, MAX_ACC_YAW);

        // 更新上一周期速度
        last_x_spd = current_x_spd;
        last_y_spd = current_y_spd;
        last_yaw_spd = current_yaw_spd;
				
        // 运动学解算
        float motor_rpm[4] = 
        {
            (COS45 * (current_x_spd  - current_y_spd) + L * current_yaw_spd) * k,  
            (COS45 * (-current_x_spd - current_y_spd) + L * current_yaw_spd) * k, 
            (COS45 * (-current_x_spd + current_y_spd) + L * current_yaw_spd) * k, 
            (COS45 * (current_x_spd  + current_y_spd) + L * current_yaw_spd) * k   
        };

        //限制电机最大转速
        for (int i = 0; i < 4; i++)
        {
            if (motor_rpm[i] > MAX_RPM)
                motor_rpm[i] = MAX_RPM;
            else if (motor_rpm[i] < -MAX_RPM)
                motor_rpm[i] = -MAX_RPM;
        }

        // 设置电机转速
        motor[0]->Set_Rpm(motor_rpm[0]);
        motor[1]->Set_Rpm(motor_rpm[1]);
        motor[2]->Set_Rpm(motor_rpm[2]);
        motor[3]->Set_Rpm(motor_rpm[3]);
    }
}
