#pragma once
#include "RC_pid.h"
#include "arm_matrix.h"
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
	
	
	
	
	
	#define L1_Particle_LENGTH       0.22821f    // 大臂关节到质心长度（米）
	#define L2_Particle_LENGTH       0.20264f    // 小臂关节到质心长度（米）
	#define L3_Particle_LENGTH       0.07760f    // 末端关节到质心长度（米）

	#define GRAVITY_ACEEL      9.788    //重力加速度

	#define L1_gravity         1.027f * GRAVITY_ACEEL      //大臂重力（kg）
	#define L2_gravity         0.416f * GRAVITY_ACEEL      //小臂重力（kg）
	#define L3_gravity         0.2f * GRAVITY_ACEEL      //末端重力（kg）



	
	
	// ------------------ 机械臂参数 ------------------
	#define L1_LENGTH       0.45657f    // 大臂长度（米）
	#define L2_LENGTH       0.4135f    // 小臂长度（米）
	#define L3_LENGTH       0.13350f    // 末端执行器长度（米）
	#define BASE_HEIGHT     0.1f 	    // 底座高度（米）

	// ------------------ 关节角度限制 ------------------
	#define THETA0_MIN      -PI
	#define THETA0_MAX       PI
	#define THETA1_MIN       -PI
	#define THETA1_MAX       0
	#define THETA2_MIN       0
	#define THETA2_MAX       1.5*PI
	#define THETA3_MIN       -PI
	#define THETA3_MAX       PI

	// ------------------ 关节角度偏移（全零位置） ------------------
	#define THETA0_OFFSET    0.0f
	#define THETA1_OFFSET    -PI
	#define THETA2_OFFSET     PI
	#define THETA3_OFFSET    0.0f

	// ------------------ 结构体 ------------------
	struct EndEffectorPos {
		float x;
		float y;
		float z;
		float angle;
	};

	enum ActuatorType {
		ACTUATOR_1,
		ACTUATOR_2
	};

	class ArmKinematics
	{
	public:
		// 正逆运动学接口
		 void forward(const JointAngles& angles, ActuatorType actuator, EndEffectorPos& end_pos);
		bool inverse(const EndEffectorPos& target_pos,
					JointAngles& result_angles);

	private:
		// 工具函数
		float normalizeAngle(float angle);
		float constrainValue(float value, float min, float max);
		float calc_reward(const JointAngles& q,
						  const EndEffectorPos& end_pos,
						  const EndEffectorPos& target, 
						  const JointAngles& current_angles);

		static ArmMatrix<4, 4> buildDHTable(float theta, float alpha, float a, float d, float offset);

		// 成员变量，存储正运动学临时矩阵
		static ArmMatrix<4,4> T01;
		static ArmMatrix<4,4> T12;
		static ArmMatrix<4,4> T23;
		static ArmMatrix<4,4> T34;
		static ArmMatrix<4,4> T04;
	};
}
#endif
