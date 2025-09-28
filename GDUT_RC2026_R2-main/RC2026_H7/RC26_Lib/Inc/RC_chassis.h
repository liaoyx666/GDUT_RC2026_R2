#pragma once
#include "RC_m3508.h"
#include "RC_task.h"
#include "RC_timer.h"

#include <math.h>

#ifdef __cplusplus

namespace chassis
{
    // 三轮全向轮三角函数常量（120°均匀分布）
    const float COS60  = 0.5f;          
    const float SIN60  = sqrtf(3.0f) / 2.0f;          
    const float COS180 = -1.0f;           
    const float SIN180 = 0.0f; 
    const float COS300 = 0.5f;
    const float SIN300 = -sqrtf(3.0f) / 2.0f;

	
	
	
    class OmniChassis : public task::ManagedTask
    {
    public:
        OmniChassis(motor::M3508 &wheel_left, motor::M3508 &wheel_mid, motor::M3508 &wheel_right, float max_xy_spd_, float max_yaw_spd_);
        
	
        void Chassis_Init();
	
	
        void Set_Chassis_Pos(float x, float y, float yaw);
        void Set_Chassis_Spd(float x, float y, float yaw);
        void Chassis_Calc();
		
        float target_pos_x = 0, target_pos_y = 0, target_pos_yaw = 0;
        float target_spd_x = 0, target_spd_y = 0, target_spd_yaw = 0;

		void Task_Process() override {Chassis_Calc();}
	
    private:
		float max_xy_spd = 0;
		float max_yaw_spd = 0;

		uint32_t last_time = 0;
		
        // 电机与机械参数（三轮）
        motor::M3508* motor[3];
        float L = 0.641f / 2.0f;    // 轮子到底盘中心距离
        float R = 0.152f / 2.0f;    // 轮半径
        float ratio = 19;             // M3508减速比
        float k = (60.0f * ratio) / (2.0f * PI * R);  // 速度转转速系数

        // 加速度控制相关
        float MAX_ACC_SPEED = 1.0f;     // 最大线加速度（m/s²）
        float MAX_ACC_YAW = 3.0f;   // Yaw轴最大角加速度（rad/s²）

        float last_spd_x = 0;          // 上一周期X方向实际速度
        float last_spd_y = 0;          // 上一周期Y方向实际速度
        float last_spd_yaw = 0;        // 上一周期Yaw轴实际角速度

        float Limit_Accel(float delta_spd, float max_acc, float dt);
    };
	
	
}




#endif
