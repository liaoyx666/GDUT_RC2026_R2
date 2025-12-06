#pragma once
#include "RC_m3508.h"
#include "RC_task.h"
#include "RC_timer.h"
#include "RC_vector2d.h"
#include "RC_chassis.h"

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

	const float L = 0.641f / 2.0f;    // 轮子到底盘中心距离
	const float R = 0.152f / 2.0f;    // 轮半径
	const float ratio = 19;             // M3508减速比
	const float k = (60.0f * ratio) / (2.0f * PI * R);  // 速度转转速系数

	// 加速度控制相关
	const float MAX_ACC_SPEED = 2.5f;     // 最大线加速度（m/s²）
	const float MAX_ACC_YAW = 4.5f;   // Yaw轴最大角加速度（rad/s²）

    class OmniChassis : public task::ManagedTask
    {
    public:
        OmniChassis(motor::M3508 &wheel_left, motor::M3508 &wheel_mid, motor::M3508 &wheel_right, 
			float max_xy_spd_, float max_yaw_spd_);

        void Chassis_Init();

		void Set_Chassis_World_Spd(float target_x, float target_y, float target_yaw, float yaw);
        void Set_Chassis_Spd(float x, float y, float yaw);
        void Chassis_Calc();

		vector2d::Vector2D target_spd;
		vector2d::Vector2D last_spd;
		                            
		float target_spd_yaw = 0;   
		float last_spd_yaw = 0;// 上一周期Yaw轴实际角速度

		void Task_Process() override {Chassis_Calc();}

    private:
		float max_xy_spd = 0;
		float max_yaw_spd = 0;

		uint32_t last_time = 0;

        // 电机与机械参数（三轮）
        motor::M3508* motor[3];

		vector2d::Vector2D current_spd;
		float current_spd_yaw = 0;

    };
	

}
#endif
