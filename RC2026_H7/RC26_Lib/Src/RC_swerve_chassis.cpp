#include "RC_swerve_chassis.h"

namespace chassis
{
	Swerve4Chassis::Swerve4Chassis(
		motor::Motor& steer_motor_1_, motor::Motor& steer_motor_2_, motor::Motor& steer_motor_3_, motor::Motor& steer_motor_4_,
		motor::Motor& drive_motor_1_, motor::Motor& drive_motor_2_, motor::Motor& drive_motor_3_, motor::Motor& drive_motor_4_,
		float max_linear_vel_, float linear_accel_, float linear_decel_,
		float max_angular_vel_, float angular_accel_, float angular_decel_,
		uint16_t gpio_pin_1_, uint16_t gpio_pin_2_, uint16_t gpio_pin_3_, uint16_t gpio_pin_4_ 
	) : 
	Chassis(
		max_linear_vel_, linear_accel_, linear_decel_,
		max_angular_vel_, angular_accel_, angular_decel_
	), 
	photogate_reposotion{
		photogate::PhoGateRepos(&steer_motor_1_, &is_reposition[0], 0, gpio_pin_1_), 
		photogate::PhoGateRepos(&steer_motor_2_, &is_reposition[1], 0, gpio_pin_2_),
		photogate::PhoGateRepos(&steer_motor_3_, &is_reposition[2], 0, gpio_pin_3_),
		photogate::PhoGateRepos(&steer_motor_4_, &is_reposition[3], 0, gpio_pin_4_),
	}
	{
		tangent_vector[0] = vector2d::Vector2D(SWERVE4_CHASSIS_THETA1 + HALF_PI);
		tangent_vector[1] = vector2d::Vector2D(SWERVE4_CHASSIS_THETA2 + HALF_PI);
		tangent_vector[2] = vector2d::Vector2D(SWERVE4_CHASSIS_THETA3 + HALF_PI);
		tangent_vector[3] = vector2d::Vector2D(SWERVE4_CHASSIS_THETA4 + HALF_PI);
		
		steer_motor[0] = &steer_motor_1_;
		steer_motor[1] = &steer_motor_2_;
		steer_motor[2] = &steer_motor_3_;
		steer_motor[3] = &steer_motor_4_;

		for (uint8_t i = 0; i < 4; i++)
		{
			steer_motor[i]->pid_pos.Pid_Mode_Init(false, false, 0.01, true);
			steer_motor[i]->pid_pos.Pid_Param_Init(200, 0, 0, 0, 0.001, 0, 14000, 10000, 10000, 10000, 10000, 20000.f, 15000);
			is_reposition[i] = true;
			
			steer_motor[i]->Reset_Out_Angle(0);
			angle[i] = 0;
		}
		
		drive_motor[0] = &drive_motor_1_;
		drive_motor[1] = &drive_motor_2_;
		drive_motor[2] = &drive_motor_3_;
		drive_motor[3] = &drive_motor_4_;
	}
	
	void Swerve4Chassis::Kinematics_calc(vector2d::Vector2D v_, float vw_)
	{
		/*************************底盘解算************************/
		vel_vector[0] = tangent_vector[0] * vw_ * SWERVE4_CHASSIS_L1 + v_;
		vel_vector[1] = tangent_vector[1] * vw_ * SWERVE4_CHASSIS_L2 + v_;
		vel_vector[2] = tangent_vector[2] * vw_ * SWERVE4_CHASSIS_L3 + v_;
		vel_vector[3] = tangent_vector[3] * vw_ * SWERVE4_CHASSIS_L4 + v_;
		
		for (uint8_t i = 0; i < 4; i++)
		{
			vel[i] = vel_vector[i].length();
			
			if (vel[i] < 1e-3f)
			{
				vel[i] = 0;
			}
			else
			{
				// -pi ~ pi -> 0 ~ 2pi
				angle[i] = fmodf(vel_vector[i].angle() + TWO_PI, TWO_PI);
			}
		}
		
		/***********************设置电机**************************/
		for (uint8_t i = 0; i < 4; i++)
		{
			// 获取真实电机航向角
			float steer_angle = steer_motor[i]->Get_Out_Angle();

			// 计算转动角度
			float delta_angle = fmodf(steer_angle - angle[i], TWO_PI);
			
			if (delta_angle > PI)
			{
				delta_angle -= TWO_PI;
			}
			else if (delta_angle < -PI)
			{
				delta_angle += TWO_PI;
			}
			
			// 计算最小转动角度
			if (fabsf(delta_angle) > HALF_PI)
			{
				angle[i] = fmodf(angle[i] + PI, TWO_PI);
				drive_motor_sign[i] = -1; 
			}
			else
			{
				drive_motor_sign[i] = 1; 
			}
			
			// 设置舵向电机角度
			steer_motor[i]->Set_Out_Angle(angle[i]);
			
			// 设置航向电机速度
			drive_motor[i]->Set_Out_Rpm((float)drive_motor_sign[i] * vel[i] * vel_to_rpm);
		}
		
		/*************************************************/
	}
	
	
	// 再次初始化
	void Swerve4Chassis::Chassis_Re_Init()
	{
		Set_Is_Init_False();
		
		is_reposition[0] = false;
		is_reposition[1] = false;
		is_reposition[2] = false;
		is_reposition[3] = false;
	}

#define STEER_MOTOR_INIT_RPM 6

	// 底盘初始化
	void Swerve4Chassis::Chassis_Init()
	{
		for (uint8_t i = 0; i < 4; i++)
		{
			if (is_reposition[i] == false)
			{
				steer_motor[i]->Set_Out_Rpm(STEER_MOTOR_INIT_RPM);
			}
			else
			{
				steer_motor[i]->Set_Out_Angle(0);
			}
		}
		
		if (is_reposition[0] == true && is_reposition[1] == true && is_reposition[2] == true && is_reposition[3] == true)
		{
			Set_Is_Init_True();
		}
	}
}