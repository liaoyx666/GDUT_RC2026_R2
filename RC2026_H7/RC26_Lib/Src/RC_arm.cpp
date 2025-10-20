#include "RC_arm.h"


#define L1_LENGTH       0.45657f    // 大臂长度（米）
#define L2_LENGTH       0.4135f    // 小臂长度（米）
#define L3_LENGTH       0.13350f    // 末端执行器长度（米）
#define BASE_HEIGHT     0.1f 	    // 底座高度（米）

#define L1_Particle_LENGTH       0.22821f    // 大臂关节到质心长度（米）
#define L2_Particle_LENGTH       0.20264f    // 小臂关节到质心长度（米）
#define L3_Particle_LENGTH       0.07760f    // 末端关节到质心长度（米）

#define GRAVITY_ACEEL      9.788    //重力加速度

#define L1_gravity         1.027f * GRAVITY_ACEEL      //大臂重力（kg）
#define L2_gravity         0.416f * GRAVITY_ACEEL      //小臂重力（kg）
#define L3_gravity         0.2f * GRAVITY_ACEEL      //末端重力（kg）


namespace arm
{
	ArmDynamics::ArmDynamics()
	{
	
	}

	
	
	void ArmDynamics::gravity_compensation()
	{
		joint_angle_now.theta1 = -motor_angle.theta1;
		joint_angle_now.theta2 = -motor_angle.theta1 + motor_angle.theta2 + PI - 0.146084f;
		joint_angle_now.theta3 = -motor_angle.theta3 + motor_angle.theta2 - motor_angle.theta1 - 0.146084f;


		joint_gravity_compensation.joint1 = L1_gravity * L1_Particle_LENGTH * cosf(joint_angle_now.theta1) 
										  + L2_gravity * (L2_Particle_LENGTH * cosf(joint_angle_now.theta2) + L1_LENGTH * cosf(joint_angle_now.theta1))
										  + L3_gravity * (L3_Particle_LENGTH * cosf(joint_angle_now.theta3) + L2_LENGTH * cosf(joint_angle_now.theta2) + L1_LENGTH * cos(joint_angle_now.theta1));

		joint_gravity_compensation.joint2 = L2_gravity * L2_Particle_LENGTH * cosf(joint_angle_now.theta2)
										  + L3_gravity * (L3_Particle_LENGTH * cosf(joint_angle_now.theta3) + L2_LENGTH * cosf(joint_angle_now.theta2));

		joint_gravity_compensation.joint3 = L3_gravity * L3_Particle_LENGTH * cosf(joint_angle_now.theta3);
	}
}