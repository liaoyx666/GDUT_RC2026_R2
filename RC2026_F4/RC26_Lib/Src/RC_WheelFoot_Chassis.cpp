#include "RC_WheelFoot_Chassis.h"

namespace RC_WheelFoot_Chassis{
	//K的获取需要车身的转动惯量，转轴到车身质心的距离，轮子半径，腿部质量和 通过matlab求取
	State_U WheelFoot_LQR::accel_calc(void){
		
		//计算车体加速度
		float base_accel = -(Robot_K.K1 * Robot_State.X_Pose + 
		                     Robot_K.K2 * (Robot_State.X_speed - Robot_target.target_speed) + 
		                     Robot_K.K3 * (Robot_State.Pitch - Robot_target.target_Pitch) + 
		                     Robot_K.K4 * Robot_State.Gyro);
		
		//计算转向差速加速度
		float turn_diff = this->Turn_Ratio * Robot_target.target_turn;
		
		//左右轮加速度 = 基础加速度 ± 转向差速（实现差速转向）
		Robot_Accel.Left_accel = base_accel - turn_diff;
		Robot_Accel.Right_accel = base_accel + turn_diff;
		
		return this->Robot_Accel;
	}
	
	Output_Velocity WheelFoot_LQR::Get_Robot_Out(void){
		this->Robot_Output.Velocity_Left = this->Ratio *(this->Robot_State.X_speed + this->Robot_Accel.Left_accel*this->dt);
		this->Robot_Output.Velocity_Right = this->Ratio *(this->Robot_State.X_speed + this->Robot_Accel.Right_accel*this->dt) ;
		return this->Robot_Output;
	}
	
	void WheelFoot_LQR::Update_State(State_data my_State){
		this->Robot_State.X_Pose = my_State.X_Pose;
		this->Robot_State.X_speed = my_State.X_speed;
		this->Robot_State.Pitch = my_State.Pitch;
		this->Robot_State.Gyro  = my_State.Gyro;
	}
	
	void WheelFoot_LQR::input_data(State_Target my_target){
		this->Robot_target.target_Pitch = my_target.target_Pitch;
		this->Robot_target.target_speed = my_target.target_speed;
		this->Robot_target.target_turn = my_target.target_turn;
	}
	void WheelFoot_LQR::Set_K(State_K my_K){
		this->Robot_K.K1 = my_K.K1;
		this->Robot_K.K2 = my_K.K2;
		this->Robot_K.K3 = my_K.K3;
		this->Robot_K.K4 = my_K.K4;
	}
}