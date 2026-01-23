#pragma once
#include <arm_math.h>
#include "RC_pid.h"
#include "RC_motor.h"
#include "RC_vector2d.h"
#include "RC_timer.h"
#include "RC_task.h"
#include "RC_JY901S.h"
#include "RC_chassis.h"
#include "LegH_KPoly.h"

#define  Joint_angle_min   -1.0f
#define  Joint_angle_max   -0.2f
#define  ground_off_FN     0.0f

#define FN_Max    0.0f
#define FN_Min    0.0f
#define Max_H   0.5f
#define Min_H   0.0f
#define STEP_H  0.05f
#define STOP_SPEED 0.0f  //大于0
#define MAX_SPEED  0.0f  //大于0

#define  L1         0.282f
#define  L2         0.2856f
#define  L3         0.29281f
#define  L4         0.13067f
#define  L23        0.05702f
#define  g          9.8f
#define  Wheel_R    0.060f
#define  Wheel_m    0.287f
#define  Body_m     0.0f

#ifdef __cplusplus
using namespace Poly_List;

namespace chassis
{
	typedef struct{
		float wheel_x;
		float wheel_y;
		float SmallLeg_ToGound_angle;
	}WheelToJoint_pos;
	
	typedef struct{
		float FN_x;
		float FN_y;
	}WheelToGound_FN;
	
	typedef struct{
		float x;
		float xd;
		float q;
		float qd;
	}State_data;
	
	typedef struct{
		float joint_angle;
		float joint_angle_d;//关节角速度
		float joint_Torque;
	}Joint_data;
	
	typedef struct{
		float Wheel_angle;
		float Wheel_angle_d;
		float Wheel_Torque;
		float Wheel_x;
		float Wheel_xd;
	}Wheel_data;
				
	class wheelleg: public Chassis
	{
		public:
			
				pid::Pid LeftVMC_controlleg_pid;
				pid::Pid RightVMC_controlleg_pid;
		
				pid::Pid LeftLeg_posControl_pid;
				pid::Pid RightLeg_posControl_pid;
		
				pid::Pid Yaw_Pid;
				pid::Pid Roll_Pid;
		
				wheelleg(motor::Motor& Left_jointMotor, motor::Motor& Right_jointMotor,
								motor::Motor& Left_wheelMotor, motor::Motor& Right_wheelMotor,imu::JY901S& IMU_,
								float max_linear_vel_, float linear_accel_, float linear_decel_,
								float max_angular_vel_, float angular_accel_, float angular_decel_);
				virtual ~wheelleg() {}
					
				LQR_K Chassis_K_Calculate(float Leg_H);//拟合的K
								
				void Update_Data();
				
				//正解解算由关节角度得到  轮子相对于关节的位置x,y(主要是Y)，以及小腿相对于地面的角度
				WheelToJoint_pos Leg_Kinematics(float joint_angle);
		
				//逆解解算得到控制对应轮子位置（x,y）的对应关节角度
				float Leg_Inverse(float target_y);
		
				//对地作用力获取
				WheelToGound_FN Get_wheel_F_ToGound(float Joint_Torque,float joint_angle,float small_legToGround_angle);
		
				//LQR控制轮子(x,xd,q,qd)
				float LQR_Calculate(LQR_K K,State_data state){
						return K.K3*(state.x-State_target.x) + K.K4*(state.xd-State_target.xd) + K.K1*(state.q-State_target.q) + K.K2*(state.qd-State_target.qd);
				}
				float LQR_Calculate_offGound(LQR_K K,State_data state){
						return K.K1*(state.q-State_target.q) + K.K2*(state.qd-State_target.qd);
				}
				
				//VMC控制关节
				float VMC_controlleg(pid::Pid pid,float target_Fx,float target_Fy,float joint_angle_d,float joint_angle,float small_legToGround_angle);
		
				//PID控制关节
				float Pid_controlleg(pid::Pid pid,float target_legH);
		
				//离地监测
				bool Check_Ground_off(WheelToGound_FN FN);
				
				//跳跃/上台阶
				void Robot_Jumping();
				
				float Limit_Data(float Raw_Data,float max,float min);
		protected:
				// 再次初始化
				void Chassis_Re_Init() override;
		
				// 底盘初始化
				void Chassis_Init() override;
		
		private:
				Poly_List::LegH_KPoly Poly_List;
				void Kinematics_calc(vector2d::Vector2D v_, float vw_) override;
		
				motor::Motor* Joint_motor[2];//关节电机
				motor::Motor* Wheel_motor[2];//轮部电机
				imu::JY901S* IMU;//IMU
		
				WheelToJoint_pos Left_wheel_pos;//轮子相对腿的位置坐标
				WheelToJoint_pos Right_wheel_pos;
		
				WheelToGound_FN  Left_wheelToGound_FN;//轮子对地作用力
				WheelToGound_FN  Right_wheelToGound_FN;
		
				Joint_data Left_Joint_data;//关节电机数据
				Joint_data Right_Joint_data;
			
				float Robot_Yaw;//底盘YAW
				float Robot_Roll;//底盘ROLL角
				State_data chassis_state;//底盘状态
				State_data State_target;//目标状态
				
				Wheel_data Left_WheelData;//足部数据
				Wheel_data Right_WheelData;
				
				LQR_K Left_Leg_K;	//K系数
				LQR_K Right_Leg_K;
				
				float Left_wheelout;
				float Right_wheelout;
				float Left_jointout;
				float Right_jointout;
				
				float target_Lleg;
				float target_Rleg;
				
				int move_stop_flag = 0;
	};

}
#endif
