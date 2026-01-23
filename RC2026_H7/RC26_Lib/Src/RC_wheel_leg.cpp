#include "RC_wheel_leg.h"

namespace chassis
{		

		wheelleg::wheelleg(motor::Motor& Left_jointMotor, motor::Motor& Right_jointMotor,
											motor::Motor& Left_wheelMotor, motor::Motor& Right_wheelMotor,imu::JY901S& IMU_,
											float max_linear_vel_, float linear_accel_, float linear_decel_,
											float max_angular_vel_, float angular_accel_, float angular_decel_):
												
		Chassis(
			max_linear_vel_, linear_accel_, linear_decel_,
			max_angular_vel_, angular_accel_, angular_decel_
		)
		{
				Joint_motor[0] = &Left_jointMotor;
				Joint_motor[1] = &Right_jointMotor;
				Wheel_motor[0] = &Left_wheelMotor;
				Wheel_motor[1] = &Right_wheelMotor;
				IMU = &IMU_;
			
				State_target.x = 0.0f;
				State_target.xd = 0.0f;
				State_target.q = 0.0f;
				State_target.qd = 0.0f;
				
				move_stop_flag = 1;
		}
		//正解
		WheelToJoint_pos wheelleg::Leg_Kinematics(float joint_angle){
				if(joint_angle < Joint_angle_min){
					joint_angle = Joint_angle_min;
				}
				else if(joint_angle > Joint_angle_max){
					joint_angle = Joint_angle_max;
				}
				float L5 = sqrt(L2*L2 + L4*L4 -2*L2*L4*cos(PI/4.0f - joint_angle));
				WheelToJoint_pos Leg_data;
				
				Leg_data.SmallLeg_ToGound_angle = PI +joint_angle - acos((L5*L5 + L2*L2 -L4*L4)/(2*L5*L2)) 
							- acos((L5*L5 +L23*L23 -L3*L3)/(2*L5*L23));
							
				Leg_data.wheel_x = L2*cos(joint_angle) - L1*cos(Leg_data.SmallLeg_ToGound_angle);
				Leg_data.wheel_y = L2*sin(joint_angle) - L1*sin(Leg_data.SmallLeg_ToGound_angle);				
				return Leg_data;
		}
		
		//逆解
		float wheelleg::Leg_Inverse(float target_y){
				float X = -0.000001;
				float phi = atan(-target_y/X);
				float R = sqrt(X*X +target_y*target_y);
				float middle_val1 = 2*L2*R;
				float middle_val2 = L2*L2 -L1*L1 + R*R;
				float acos_arg = middle_val2 / middle_val1;
				
				float angle_1 = phi - acos(acos_arg);
				float angle_2 = phi + acos(acos_arg);
				float joint_angle;
			
				if(angle_1 < Joint_angle_max && angle_1 > Joint_angle_min){
					joint_angle = angle_1;
				}
				else{
					joint_angle = Joint_angle_min;
				}
				if(angle_2 < Joint_angle_max && angle_2 > Joint_angle_min){
					joint_angle = angle_2;
				}
				else{
					joint_angle = Joint_angle_min;
				}
				return joint_angle;
		}
		
		//求对地作用力
		WheelToGound_FN wheelleg::Get_wheel_F_ToGound(float Joint_Torque,float joint_angle,float small_legToGround_angle){
			WheelToGound_FN wheel_F;
			wheel_F.FN_x = Joint_Torque/L2 *cos(small_legToGround_angle)*cos(PI/2 - small_legToGround_angle+joint_angle);
			wheel_F.FN_y = Joint_Torque/L2 *sin(small_legToGround_angle)*cos(PI/2 - small_legToGround_angle+joint_angle);
			return wheel_F;
		}
		//VMC控制关节角度
		float wheelleg::VMC_controlleg(pid::Pid pid,float target_Fx,float target_Fy,float joint_angle_d,float joint_angle,float small_legToGround_angle){
				float joint_Torque_out;
				
				pid.Update_Target(Limit_Data(target_Fy,FN_Max,FN_Min));
			
//			float angled_to_wheel_VX = joint_angle_d*L2 *cos(small_legToGround_angle)*cos(PI/2 - small_legToGround_angle+joint_angle);
				float angled_to_wheel_VY = joint_angle_d*L2 *sin(small_legToGround_angle)*cos(PI/2 - small_legToGround_angle+joint_angle);
				joint_Torque_out = angled_to_wheel_VY * pid.Pid_Calculate();
				return joint_Torque_out;
		}
		//Pid控制关节角度
		float wheelleg::Pid_controlleg(pid::Pid pid,float target_legH){
				pid.Update_Target(-Limit_Data(target_legH,Max_H,Min_H));
				return pid.Pid_Calculate();
		}

		//离地监测
		bool wheelleg::Check_Ground_off(WheelToGound_FN FN){
				if(FN.FN_y < ground_off_FN){
						return true;
				}
				else{
						return false;
				}
		}
		//限幅函数
		float wheelleg::Limit_Data(float Raw_Data,float max,float min){
			if(Raw_Data >= min && Raw_Data <= max){
					return Raw_Data;
			}
			else if( Raw_Data < min){
					return min;
			}
			else{
					return max;
			}
		}
				

		LQR_K wheelleg::Chassis_K_Calculate(float Leg_H){
				//拟合0.0~0.5m腿长对应K拟合,并边界线性插值
				float normalized = (Leg_H - Min_H)/STEP_H;
				int idx_low =static_cast<int>(floor(normalized));
				int idx_high = idx_low +1;
				if(idx_high >= 10){
					return Poly_List.fitFunctions[9](Leg_H);
				}
				if(idx_low < 0){
					return Poly_List.fitFunctions[0](Leg_H);
				}
				float t = normalized - idx_low;
				t = fmax(0.0f, fmin(1.0f, t));
				LQR_K K_low = Poly_List.fitFunctions[idx_low](Leg_H);
				LQR_K K_high = Poly_List.fitFunctions[idx_high](Leg_H);
				LQR_K K;
				K.K1 = (1-t)*K_low.K1 + t*K_high.K1;
				K.K2 = (1-t)*K_low.K2 + t*K_high.K2;
				K.K3 = (1-t)*K_low.K3 + t*K_high.K3;
				K.K4 = (1-t)*K_low.K4 + t*K_high.K4;
				return K;
		}
		
		void wheelleg::Update_Data(){
				Left_Joint_data.joint_angle = Joint_motor[0]->Get_Angle();
				Left_Joint_data.joint_angle_d = Joint_motor[0]->Get_Rpm();
				Left_Joint_data.joint_Torque= Joint_motor[0]->Get_Torque();
				
				Right_Joint_data.joint_angle = Joint_motor[1]->Get_Angle();
				Right_Joint_data.joint_angle_d = Joint_motor[1]->Get_Rpm();
				Right_Joint_data.joint_Torque= Joint_motor[1]->Get_Torque();
									
				Left_WheelData.Wheel_angle = Wheel_motor[0]->Get_Angle();
				Left_WheelData.Wheel_angle_d = Wheel_motor[0]->Get_Rpm();
				Left_WheelData.Wheel_Torque = Wheel_motor[0]->Get_Torque();
				Left_WheelData.Wheel_x = Wheel_motor[0]->Get_Pos();
				Left_WheelData.Wheel_xd = Wheel_motor[0]->Get_Rpm()*Wheel_R;
				
				Right_WheelData.Wheel_angle = Wheel_motor[1]->Get_Angle();
				Right_WheelData.Wheel_angle_d = Wheel_motor[1]->Get_Rpm();
				Right_WheelData.Wheel_Torque = Wheel_motor[1]->Get_Torque();
				Right_WheelData.Wheel_x = Wheel_motor[1]->Get_Pos();
				Right_WheelData.Wheel_xd = Wheel_motor[1]->Get_Rpm()*Wheel_R;
									
				
				Left_wheel_pos = Leg_Kinematics(Left_Joint_data.joint_angle);
				Right_wheel_pos = Leg_Kinematics(Right_Joint_data.joint_angle);
				
				Left_wheelToGound_FN = Get_wheel_F_ToGound(Left_Joint_data.joint_Torque,Left_Joint_data.joint_angle,Left_wheel_pos.SmallLeg_ToGound_angle);
				Right_wheelToGound_FN = Get_wheel_F_ToGound(Right_Joint_data.joint_Torque,Right_Joint_data.joint_angle,Right_wheel_pos.SmallLeg_ToGound_angle);
				

				Robot_Yaw = IMU->Get_Yaw();
				Robot_Roll = IMU->Get_Roll();
				
				
				chassis_state.x = (Left_WheelData.Wheel_x + Right_WheelData.Wheel_x)/2.0f;
				chassis_state.xd = (Left_WheelData.Wheel_xd + Right_WheelData.Wheel_xd)/2.0f;
				chassis_state.q = IMU->Get_Pitch();
				chassis_state.qd = IMU->Get_Gyro()[1];
									
				Left_Leg_K = Chassis_K_Calculate(-Left_wheel_pos.wheel_y);
				Right_Leg_K = Chassis_K_Calculate(-Right_wheel_pos.wheel_y);
				
				Left_jointout = VMC_controlleg(LeftVMC_controlleg_pid,0.00f,target_Lleg,Left_Joint_data.joint_angle_d,Left_Joint_data.joint_angle,Left_wheel_pos.SmallLeg_ToGound_angle);
				Right_jointout = VMC_controlleg(RightVMC_controlleg_pid,0.00f,target_Rleg,Right_Joint_data.joint_angle_d,Right_Joint_data.joint_angle,Right_wheel_pos.SmallLeg_ToGound_angle);
		}
		
		void wheelleg::Kinematics_calc(vector2d::Vector2D v_, float vw_){
				Update_Data();//数据更新
				//常态模式,纯差速控制,腿部锁定在一个位置
				//上台阶模式,腿部锁定至台阶高度位置,贴台阶直接上去
				//上坡模式
			
				
				//运动时位移零点重置
				if(v_.y() != 0){
					State_target.x = chassis_state.x;
				}
//				if((last_v.x() != 0 && v_.x() == 0) || (last_v.y() != 0 && v_.y() == 0)){
//						move_stop_flag = 1;
//				}
				if((move_stop_flag == 1) && (abs(chassis_state.xd) < STOP_SPEED)){
						move_stop_flag = 0;
						State_target.x = chassis_state.x;//重置位移零点
				}
				//被外部推动
				if(abs(chassis_state.xd) > MAX_SPEED){
						State_target.x = chassis_state.x;//重置位移零点
				}
				
//				Left_wheelout = v_.y() + v_.x()*vw_;
//				Right_wheelout =  v_.y() - v_.x()*vw_;
				
				if(Check_Ground_off(Left_wheelToGound_FN) && Check_Ground_off(Right_wheelToGound_FN)){
						Left_wheelout = LQR_Calculate_offGound(Left_Leg_K,chassis_state);
						Right_wheelout = LQR_Calculate_offGound(Right_Leg_K,chassis_state);
				}
				else{
						Left_wheelout = LQR_Calculate(Left_Leg_K,chassis_state);
						Right_wheelout = LQR_Calculate(Right_Leg_K,chassis_state);
				}
				
				
//				Wheel_motor[0]->Set_Torque(Left_wheelout + Yaw_Pid.Pid_Calculate(Robot_Yaw));
//				Wheel_motor[1]->Set_Torque(Right_wheelout - Yaw_Pid.Pid_Calculate(Robot_Yaw));
//				Joint_motor[0]->Set_Torque(Left_jointout);
//				Joint_motor[1]->Set_Torque(Right_jointout);
		}
			// 再次初始化
		void wheelleg::Chassis_Re_Init(){
				Set_Is_Init_False();
			
		}
		
		// 底盘初始化
		void wheelleg::Chassis_Init(){
			
			
			
			
			
			Set_Is_Init_True();
			
		}	

//bool wheelleg::Ground_off(){
//		float zdd_wheel;
//		float zdd_body;
//		float dt;
//		zdd_wheel = zdd_body - Leg_H*dt*dt*cos(state(0,0))
//					+2*Leg_H*dt*sin(state(1,0)) + Leg_H*state(1,0)*dt*sin(state(0,0)) 
//					+ Leg_H*state(1,0)*state(1,0)*cos(state(0,0)); 
//		FN = F_Y + Wheel_m*g +Wheel_m*zdd_wheel;
//		if(FN > ground_off_FN){
//				return true;
//		}
//		else {
//			return false;
//		}
//		return false;
//}

}
