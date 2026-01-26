#pragma once
#include "RC_motor.h"
#include "RC_vector2d.h"
#include "RC_timer.h"
#include "RC_chassis.h"

#include <math.h>


#ifdef __cplusplus

/************************** 麦轮核心参数 **************************/
#define MECANUM4_CHASSIS_TRACK_WIDTH 0.44020f  // 左右轮距
#define MECANUM4_CHASSIS_WHEELBASE 0.380f    // 前后轮距
#define MECANUM4_CHASSIS_WHEEL_RADIUS 0.076f // 麦轮半径


namespace chassis
{


	class Mecanum4Chassis : public Chassis
    {
    public:
		Mecanum4Chassis(
			motor::Motor& drive_motor_1_, motor::Motor& drive_motor_2_, motor::Motor& drive_motor_3_, motor::Motor& drive_motor_4_,// 麦轮
            motor::Motor& auxiliary_motor_1_, motor::Motor& auxiliary_motor_2_, // 辅助轮
            motor::Motor& lift_lower_motor_,// 升降导轨电机
			float max_linear_vel_, float linear_accel_, float linear_decel_,
			float max_angular_vel_, float angular_accel_, float angular_decel_
		);
		
		virtual ~Mecanum4Chassis() {}
		
    protected:
			
    private:
		void Kinematics_calc(vector2d::Vector2D v_, float vw_) override;

		void up_stair();
        void down_stair();

        void lift_body();
        void lower_body();

		
		// 电机指针
		motor::Motor* drive_motor[4];// 麦轮电机
        motor::Motor* auxiliary_motor[2];// 辅助轮电机
		motor::Motor* lift_lower_motor;// 辅助轮电机

		float vel[4] = {0};// 电机速度
		
		int8_t drive_motor_sign[4] = {1, 1, 1, 1};

		const float vel_to_rpm = (1.f / MECANUM4_CHASSIS_WHEEL_RADIUS) * (60.0f / (2.0f * PI));
    };
}
#endif
