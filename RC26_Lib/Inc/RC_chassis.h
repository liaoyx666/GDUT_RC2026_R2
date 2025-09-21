#pragma once
#define RC_CHASSIS_H
#include "RC_pid.h"
#include "RC_motor.h"
#include "RC_m3508.h"
#include "RC_task.h"

#ifdef __cplusplus

namespace chassis
{
    const float COS45 = sqrt(2.0f) / 2.0f;  // ≈0.7071（保证计算精度）

    class RC_Chassis
    {
    public:
        RC_Chassis(m3508::M3508* m3508_1, m3508::M3508* m3508_2, m3508::M3508* m3508_3, m3508::M3508* m3508_4);
        
        void chassis_init();
        void Set_Target_Pos(float x, float y, float yaw);
        void Set_Target_Spd(float x, float y, float yaw);
        void Update();
				
        float target_x, target_y, target_yaw;
        float target_x_spd, target_y_spd, target_yaw_spd;

    private:
        // 电机与机械参数
        m3508::M3508* motor[4];
        const float L = 0.641f / 2.0f;    // 轮子到底盘中心距离
        const float R = 0.152f / 2.0f;    // 轮半径
        const int ratio = 19;             // M3508减速比
        const float k = (60.0f * ratio) / (2.0f * PI * R);  
        const float MAX_RPM = 6000.0f;    // M3508最大转速

        // 加速度控制相关（核心新增）
        const float MAX_ACC_X = 2.0f;     // X方向最大线加速度（m/s²）
        const float MAX_ACC_Y = 2.0f;     // Y方向最大线加速度（m/s²）
        const float MAX_ACC_YAW = 1.0f;   // Yaw轴最大角加速度（rad/s²）
        const float dt = 0.001f;          // 控制周期（1ms）
        float last_x_spd = 0.0f;          // 上一周期X方向实际速度
        float last_y_spd = 0.0f;          // 上一周期Y方向实际速度
        float last_yaw_spd = 0.0f;        // 上一周期Yaw轴实际角速度

        float limit_acceleration(float target_spd, float last_spd, float max_acc);
    };
}

// 底盘控制任务类
class RC_Chassis_Task : public task::ManagedTask
{
public:
    RC_Chassis_Task(chassis::RC_Chassis& chassis_ref) 
        : ManagedTask("ChassisTask", 15, 256, task::TASK_PERIOD, 1), chassis(chassis_ref)
    {
        chassis.chassis_init();
    }

    void Task_Process() override
    {
        chassis.Update();        
    }

private:
    chassis::RC_Chassis& chassis;
};

#endif
