#pragma once
#include "RC_pid.h"
// Header: 机械臂
// File Name: 
// Author:
// Date:

#ifdef __cplusplus
namespace arm
{
	typedef struct
	{
		float theta0;
		float theta1;       //大臂关节与绝对坐标系x轴夹角
		float theta2;       //小臂关节与绝对坐标系x轴夹角
		float theta3;       //末端关节与绝对坐标系x轴夹角
	} JointAngles;
	
	typedef struct
	{
		float joint1;
		float joint2;
		float joint3;
	}Joint_gravity_compensation;        //关节重力补偿结构体

	
	
	class ArmDynamics
	{
	public:
		ArmDynamics();
		virtual ~ArmDynamics() {}
		
		JointAngles motor_angle{0, 0, 0, 0};
		JointAngles joint_angle_now{0, 0, 0, 0};//当前关节角度

		Joint_gravity_compensation joint_gravity_compensation{0, 0, 0};//关节重力补偿值（扭矩 N/m）

		void gravity_compensation();
		
	private:

	};
}
#endif
