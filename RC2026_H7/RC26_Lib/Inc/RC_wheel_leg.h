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
#include "RC_serial.h"
#define  Joint_angle_min   -0.7f
#define  Joint_angle_max   -0.0f

#define  ground_off_FN     3.0f

#define LeftJoint_angle_offset  1.04719755116f
#define RightJoint_angle_offset -1.04719755116f


#define FN_Max    100.0f
#define FN_Min    0.0f
#define Max_H   0.35f
#define Min_H   0.0f

#define STEP_H  0.05f
#define STOP_SPEED 0.0f  //大于0
#define MAX_SPEED  0.0f  //大于0

#define LeftJoint_Dir      1
#define RightJoint_Dir     -1
#define LeftWheel_Dir      -1
#define RightWheel_Dir     1

#define Range     30
#define KF        0.02
#define  L1         0.282f
#define  L2         0.2856f
#define  L3         0.29281f
#define  L4         0.13067f
#define  L23        0.05702f
#define  g          9.8f
#define  Wheel_R    0.060f
#define  Wheel_m    0.173520730578001f
#define  Body_m     10.113f

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
		
				pid::Pid Yaw_Pid;
				pid::Pid Roll_Pid;
		
				wheelleg(motor::Motor& Left_jointMotor, motor::Motor& Right_jointMotor,
								motor::Motor& Left_wheelMotor, motor::Motor& Right_wheelMotor,imu::imu& IMU_,timer::Timer& timer_us_,
								float max_linear_vel_, float linear_accel_, float linear_decel_,
								float max_angular_vel_, float angular_accel_, float angular_decel_);
				virtual ~wheelleg() {}
					
				LQR_K Chassis_K_Calculate(float Leg_H);//拟合的K
								
				void Update_Data();
				
				//正解解算由关节角度得到  轮子相对于关节的位置x,y(主要是Y)，以及小腿相对于地面的角度
				WheelToJoint_pos Leg_Kinematics(float joint_angle);
				
				float Pitch_offset(float x,float y);
					
				float Gravity_Compension(float alpha_,float belta_,float x);
					
				//逆解解算得到控制对应轮子位置（x,y）的对应关节角度
				float Leg_Inverse(float target_y);
		
				//对地作用力获取
				float Get_wheel_F_ToGound(pid::Pid VMC_pid,float target_LegH);

				float FN_ToGound(float Torque);
				
				//LQR控制轮子(x,xd,q,qd)
				float LQR_Calculate(LQR_K K,State_data state){
						return  K.K3*(state.x-State_target.x)-K.K4*(state.xd-State_target.xd) + K.K1*(state.q-State_target.q) + K.K2*(state.qd-State_target.qd);
				}
				float LQR_Calculate_offGound(LQR_K K,State_data state){
						return K.K1*(state.q-State_target.q) + K.K2*(state.qd-State_target.qd);
				}
				
				//VMC控制关节
				float VMC_controlleg(float Expect_FN,float joint_angle);
			
				//刚性控制关节
				float Rigid_controlleg(float target_legH);
				
				//根据步长，以及目标和当前误差，线性逼近
				float Calc_Jointout(float current,float target,float step);
				
				//离地监测
				bool Check_Ground_off(float FN);
				
				//跳跃/上台阶
				void Robot_Jumping();
				
				void Set_targetLeg(float Lleg,float Rleg);
				
				float Limit_Data(float Raw_Data,float max,float min);
				
				void Printf_Data();
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
				imu::imu* IMU;//IMU
				timer::Timer* Timer_us;
		
				WheelToJoint_pos Left_wheel_pos;//轮子相对腿的位置坐标
				WheelToJoint_pos Right_wheel_pos;
		
				float  Left_wheelToGound_FN;//轮子对地作用力
				float  Right_wheelToGound_FN;
				float  Left_Expect_FN;
		    float  Right_Expect_FN;
		
				Joint_data Left_Joint_data;//关节电机数据
				Joint_data Right_Joint_data;
			
				float Robot_Yaw;//底盘YAW
				float Robot_Roll;//底盘ROLL角
				State_data chassis_state;//底盘状态
				State_data State_target;//目标状态
				
				Wheel_data Left_WheelData;//足部数据
				Wheel_data Right_WheelData;
				
				float L5;
				
				float Vertical_angle = 0.0f;
				float Vertical_angled = 0.0f;
				float Last_vertical_angled = 0.0f;
				float Vertical_angledd = 0.0f;
				
				float Leg_H;
				float Leg_Hd;
				float Leg_Hdd;
				float Last_LegH;
				float Last_LegHd;
				float zm_d;
				float zm_dd;
				float last_zm_d;
				
				LQR_K Left_Leg_K;	//K系数
				LQR_K Right_Leg_K;
				
				float Left_wheelout;
				float Right_wheelout;
				float Left_jointout;
				float Right_jointout;
				
				float Left_Torque_out;
				float Right_Torque_out;
				
				float target_Lleg = 0.3f;
				float target_Rleg = 0.3f;
				
				bool Off_flag = 0;
				
				int move_stop_flag = 0;
				
				float last_count;
	};

}
#endif
