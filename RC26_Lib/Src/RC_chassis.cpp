#include "RC_chassis.h"

namespace chassis
{
    // 构造函数（三轮）
    RC_Chassis::RC_Chassis(
        m3508::M3508* m3508_1, 
        m3508::M3508* m3508_2, 
        m3508::M3508* m3508_3
    )
    {
        // 检查电机指针有效性
        if (m3508_1 == nullptr || m3508_2 == nullptr || m3508_3 == nullptr)
            Error_Handler();

        motor[0] = m3508_1;  // 电机1：0°方向
        motor[1] = m3508_2;  // 电机2：120°方向
        motor[2] = m3508_3;  // 电机3：240°方向

        // 初始化目标值
        target_x = target_y = target_yaw = 0.0f;
        target_x_spd = target_y_spd = target_yaw_spd = 0.0f;
    }

    // 底盘初始化
    void RC_Chassis::chassis_init()
    {
        // 位置环PID初始化（如需位置控制可在此添加）
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
        float max_delta = max_acc * dt;  // 最大速度增量
        float delta = target_spd - last_spd;

        // 限制速度增量
        if (delta > max_delta)
            delta = max_delta;
        else if (delta < -max_delta)
            delta = -max_delta;
        
        return last_spd + delta;
    }

    // 运动控制（三轮全向轮解算）
    void RC_Chassis::Update()
    {
        // 加速度限制
        float current_x_spd = limit_acceleration(target_x_spd, last_x_spd, MAX_ACC_X);
        float current_y_spd = limit_acceleration(target_y_spd, last_y_spd, MAX_ACC_Y);
        float current_yaw_spd = limit_acceleration(target_yaw_spd, last_yaw_spd, MAX_ACC_YAW);

        // 更新上一周期速度
        last_x_spd = current_x_spd;
        last_y_spd = current_y_spd;
        last_yaw_spd = current_yaw_spd;
        
        // 三轮全向轮运动学解算
        // 电机分布：0°、120°、240°（顺时针排列）
        float motor_rpm[3] = 
        {
            // 电机1（0°）：X方向速度 + 旋转速度
            (current_x_spd * COS0 + current_y_spd * SIN0 + L * current_yaw_spd) * k,
            // 电机2（120°）：分解速度 + 旋转速度
            (current_x_spd * COS120 + current_y_spd * SIN120 + L * current_yaw_spd) * k,
            // 电机3（240°）：分解速度 + 旋转速度
            (current_x_spd * COS240 + current_y_spd * SIN240 + L * current_yaw_spd) * k
        };

        // 限制电机最大转速
        for (int i = 0; i < 3; i++)
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
    }
}
    
