#pragma once
#include "RC_motor.h"
#include "RC_vector2d.h"
#include "RC_timer.h"
#include "RC_chassis.h"
#include "RC_photogate.h"

#include <math.h>

// 轮子到中心距离
#define SWERVE4_CHASSIS_L1 0.325f
#define SWERVE4_CHASSIS_L2 0.325f
#define SWERVE4_CHASSIS_L3 0.325f
#define SWERVE4_CHASSIS_L4 0.325f

// 轮子中心连线与x轴夹角
#define SWERVE4_CHASSIS_THETA1 0.785398f
#define SWERVE4_CHASSIS_THETA2 2.356194f
#define SWERVE4_CHASSIS_THETA3 -2.356194f
#define SWERVE4_CHASSIS_THETA4 -0.785398f

#define SWERVE4_CHASSIS_WHEEL_RADIUS 0.055f// 轮子半径

#ifdef __cplusplus
namespace chassis
{
	class Swerve4Chassis : public Chassis
    {
    public:
		Swerve4Chassis(
			motor::Motor& steer_motor_1_, motor::Motor& steer_motor_2_, motor::Motor& steer_motor_3_, motor::Motor& steer_motor_4_,
			motor::Motor& drive_motor_1_, motor::Motor& drive_motor_2_, motor::Motor& drive_motor_3_, motor::Motor& drive_motor_4_,
			float max_linear_vel_, float linear_accel_, float linear_decel_,
			float max_angular_vel_, float angular_accel_, float angular_decel_,
			uint16_t gpio_pin_1_, uint16_t gpio_pin_2_, uint16_t gpio_pin_3_, uint16_t gpio_pin_4_ 
		);
		
		virtual ~Swerve4Chassis() {}
		
    protected:
		photogate::PhoGateRepos photogate_reposotion[4];
		
		// 再次初始化
		void Chassis_Re_Init() override;
		
		// 底盘初始化
		void Chassis_Init() override;
	
    private:
		void Kinematics_calc(vector2d::Vector2D v_, float vw_) override;
		
		// 电机指针
		motor::Motor* steer_motor[4];// 舵向电机
		motor::Motor* drive_motor[4];// 航向电机
	
		bool is_reposition[4] = {false};
		
		vector2d::Vector2D tangent_vector[4];// 单位切向量
		
		vector2d::Vector2D vel_vector[4];// 电机速度向量
		
		float vel[4] = {0};// 电机速度
		float angle[4] = {0};// 电机角度
		
		int8_t drive_motor_sign[4] = {0};

		const float vel_to_rpm = (1.f / SWERVE4_CHASSIS_WHEEL_RADIUS) * (60.0f / (2.0f * PI));

    };
}
#endif
