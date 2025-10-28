#include "RC_WheelFoot_Chassis.h"

namespace RC_WheelFoot_Chassis{
	//joint_angle为弧度制
	void WheelFoot_control::WheelFoot_ChassisControl(L_target target_L,State_data state,Joint_angle joint_angle,State_data target_state){
		L_target now_L = this->Get_Now_L(joint_angle);
		K_data K = this->K_calc(now_L);
		this->Joint_Control(target_L,now_L);
		this->Wheel_Control(state,K,target_state);
	}
	L_target WheelFoot_control::Get_Now_L(Joint_angle joint_angle){
		L_target now_L;
		//左腿y,单位m,坐标为负值，运算由matlab生成
		now_L.Left_L = 235.0*sin(joint_angle.Left_angle + acos((0.00877193*(51831.6*cos(joint_angle.Left_angle - 0.785398) - 12214.0))/sqrt(68501.0 - 51831.6*cos(joint_angle.Left_angle - 0.785398))) + acos((0.00210084*(51831.6*cos(joint_angle.Left_angle - 0.785398) - 113288.0))/sqrt(68501.0 - 51831.6*cos(joint_angle.Left_angle - 0.785398)))) + 238.0*sin(joint_angle.Left_angle);
		now_L.Left_L  = -now_L.Left_L; //左腿y取正
		//右腿y
		now_L.Right_L = 235.0*sin(joint_angle.Right_angle + acos((0.00877193*(51831.6*cos(joint_angle.Right_angle  - 0.785398) - 12214.0))/sqrt(68501.0 - 51831.6*cos(joint_angle.Right_angle  - 0.785398))) + acos((0.00210084*(51831.6*cos(joint_angle.Right_angle  - 0.785398) - 113288.0))/sqrt(68501.0 - 51831.6*cos(joint_angle.Right_angle  - 0.785398)))) + 238.0*sin(joint_angle.Right_angle);
		now_L.Right_L = -now_L.Right_L; //右腿y取正
		return now_L;
	}

	K_data WheelFoot_control::K_calc(L_target now_L){
		K_data K_out;
		//运算由matlab生成
		//左轮K
		K_out.K1[0]=-2.449490; 
		K_out.K2[0]=-13.468329*(now_L.Left_L*now_L.Left_L)+21.016063*now_L.Left_L+213.504559; 
		K_out.K3[0]= 0.435572*(now_L.Left_L*now_L.Left_L)-0.664364*now_L.Left_L-7.862820; 
		K_out.K4[0]=-18.354839*(now_L.Left_L*now_L.Left_L)+32.810958*now_L.Left_L+2.510728; 
		//右轮K
		K_out.K1[1]=-2.449490; 
		K_out.K2[1]=-13.468329*(now_L.Right_L*now_L.Right_L)+21.016063*now_L.Right_L+213.504559; 
		K_out.K3[1]=0.435572*(now_L.Right_L*now_L.Right_L)-0.664364*now_L.Right_L-7.862820; 
		K_out.K4[1]=-18.354839*(now_L.Right_L*now_L.Right_L)+32.810958*now_L.Right_L+2.510728; 
		return K_out;
	}
	void WheelFoot_control::Joint_Control(L_target L,L_target now_L){
		//直接PID控制关节电机
		//左腿控制
		//右腿控制
		//关节输出，输出为扭矩
	}
	void WheelFoot_control::Wheel_Control(State_data state,K_data K,State_data target_state){
		//状态卡尔曼滤波
		
		//左轮控制
		uint16_t Left_out = K.K1[0]*(state.X_Pose[0]-target_state.X_Pose[0])
							+K.K2[0]*(state.Pitch[0]-target_state.Pitch[0])
							+K.K3[0]*(state.X_speed[0]-target_state.X_speed[0])
							+K.K4[0]*(state.Gyro[0]-target_state.Gyro[0]);
		//右轮控制
		uint16_t Right_out = K.K1[1]*(state.X_Pose[1]-target_state.X_Pose[1])
							+K.K2[1]*(state.Pitch[1]-target_state.Pitch[1])
							+K.K3[1]*(state.X_speed[1]-target_state.X_speed[1])
							+K.K4[1]*(state.Gyro[1]-target_state.Gyro[1]);
		//电机输出(注意输出是F)
		
	}

}
