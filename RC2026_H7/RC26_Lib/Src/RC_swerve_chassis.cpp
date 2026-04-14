#include "RC_swerve_chassis.h"

namespace chassis
{
	Swerve4Chassis::Swerve4Chassis(
		motor::DjiMotor& steer_motor_1_, motor::DjiMotor& steer_motor_2_, motor::DjiMotor& steer_motor_3_, motor::DjiMotor& steer_motor_4_,
		motor::Motor& drive_motor_1_, motor::Motor& drive_motor_2_, motor::Motor& drive_motor_3_, motor::Motor& drive_motor_4_,
		float max_linear_vel_, float linear_accel_, float linear_decel_,
		float max_angular_vel_, float angular_accel_, float angular_decel_,
		uint16_t gpio_pin_1_, uint16_t gpio_pin_2_, uint16_t gpio_pin_3_, uint16_t gpio_pin_4_,
		data::RobotPose& pose_
	) : 
	Chassis(
		max_linear_vel_, linear_accel_, linear_decel_,
		max_angular_vel_, angular_accel_, angular_decel_,
		pose_
	),
	photogate_reposotion{
		photogate::PhoGateRepos(steer_motor_1_, is_reposition[0], TWO_THIRD_PI, gpio_pin_1_, 25), 
		photogate::PhoGateRepos(steer_motor_2_, is_reposition[1], HALF_PI, gpio_pin_2_, 25),
		photogate::PhoGateRepos(steer_motor_3_, is_reposition[2], TWO_THIRD_PI, gpio_pin_3_, 25),
		photogate::PhoGateRepos(steer_motor_4_, is_reposition[3], TWO_THIRD_PI, gpio_pin_4_, 25),
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
			steer_motor[i]->pid_pos.Pid_Mode_Init(false, false, 0.0, true);
			steer_motor[i]->pid_pos.Pid_Param_Init(170, 0, 3, 0, 0.002, 0, 11500, 10000, 10000, 10000, 10000, 10000, 11000);
			steer_motor[i]->Reset_Out_Angle(0);
		}
		
		drive_motor[0] = &drive_motor_1_;
		drive_motor[1] = &drive_motor_2_;
		drive_motor[2] = &drive_motor_3_;
		drive_motor[3] = &drive_motor_4_;
	}
	
	void Swerve4Chassis::Kinematics_calc(vector2d::Vector2D v_, float vw_)
	{
		/*************************底盘解算************************/
		if (v_.length() <= 0.005f && fabsf(vw_) <= 0.006f)
		{
			/*零速锁定底盘*/
			angle[0] = SWERVE4_CHASSIS_THETA1;
			angle[1] = SWERVE4_CHASSIS_THETA2;
			angle[2] = SWERVE4_CHASSIS_THETA3;
			angle[3] = SWERVE4_CHASSIS_THETA4;
			
			vel[0] = 0;
			vel[1] = 0;
			vel[2] = 0;
			vel[3] = 0;
		}
		else
		{
			vel_vector[0] = (tangent_vector[0] * (vw_ * SWERVE4_CHASSIS_L1)) + v_;
			vel_vector[1] = (tangent_vector[1] * (vw_ * SWERVE4_CHASSIS_L2)) + v_;
			vel_vector[2] = (tangent_vector[2] * (vw_ * SWERVE4_CHASSIS_L3)) + v_;
			vel_vector[3] = (tangent_vector[3] * (vw_ * SWERVE4_CHASSIS_L4)) + v_;
			
			for (uint8_t i = 0; i < 4; i++)
			{
				vel[i] = vel_vector[i].length();
				
				if (vel[i] < 0.003f)
				{
					vel[i] = 0;
				}
				else
				{
					angle[i] = vel_vector[i].angle();
			
					// -pi ~ pi -> 0 ~ 2pi
					if (angle[i] < 0.f)
					{
						angle[i] += TWO_PI;
					}
				}
			}
		}
		
		/***********************设置电机**************************/
		for (uint8_t i = 0; i < 4; i++)
		{
			// 获取真实电机航向角
			float steer_angle = steer_motor[i]->Get_Out_Angle();

			// 计算转动角度
			float delta_angle = steer_angle - angle[i];
			
			if (delta_angle > PI)
				delta_angle -= TWO_PI;
			else if (delta_angle < -PI)
				delta_angle += TWO_PI;
			
			// 计算最小转动角度，平衡舵向转动扭矩
			if (fabsf(delta_angle - HALF_PI) < 0.017453f) /*0.5度*/
			{
				if (i % 2 == 0)
				{
					if (angle[i] < PI)
						angle[i] +=PI;
					else
						angle[i] -=PI;
					
					drive_motor_sign[i] = -1;
				}
			}
			else if (fabsf(delta_angle + HALF_PI) < 0.017453f) /*0.5度*/
			{
				if (i % 2 == 1)
				{
					if (angle[i] < PI)
						angle[i] += PI;
					else
						angle[i] -= PI;
						
					drive_motor_sign[i] = -1;
				}
			}
			else if (fabsf(delta_angle) > HALF_PI)
			{
				if (angle[i] < PI)
					angle[i] += PI;
				else
					angle[i] -= PI;
				
				drive_motor_sign[i] = -1;
			}
			else
			{
				drive_motor_sign[i] = 1;
			}
			
			// 设置舵向电机角度
			steer_motor[i]->Set_Out_Angle(angle[i]);
			
			/*低速状态下转到位才能运动*/
			if (vel[i] > 0.15f || fabsf(delta_angle) < 0.087266f)
			{
				// 设置航向电机速度
				drive_motor[i]->Set_Out_Rpm(vel[i] * vel_to_rpm * ((float)drive_motor_sign[i]));
			}
			else
			{
				drive_motor[i]->Set_Out_Rpm(0);
			}
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

	#define STEER_MOTOR_INIT_RPM 35

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
				steer_motor[i]->Set_Out_Rpm(0);
			}
		}
		
		if (is_reposition[0] == true && is_reposition[1] == true && is_reposition[2] == true && is_reposition[3] == true)
		{
			Set_Is_Init_True();
		}
	}
}