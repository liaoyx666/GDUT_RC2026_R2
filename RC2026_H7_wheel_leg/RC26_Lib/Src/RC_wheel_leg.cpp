#include "RC_wheel_leg.h"

namespace chassis
{		

		wheelleg::wheelleg(motor::Motor& Left_jointMotor, motor::Motor& Right_jointMotor,
											motor::Motor& Left_wheelMotor, motor::Motor& Right_wheelMotor,imu::imu& IMU_,timer::Timer& timer_us_,
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
				Timer_us = &timer_us_;
				State_target.x = 0.0f;
				State_target.xd = 0.0f;
				State_target.q = 0.0f;
				State_target.qd = 0.0f;
				
				move_stop_flag = 1;
			  last_count = Timer_us->Get_TimeStamp();
		}
		//正解
		WheelToJoint_pos wheelleg::Leg_Kinematics(float joint_angle){
				L5 = sqrt(L2*L2 + L4*L4 -2*L2*L4*cos(PI/4.0f - joint_angle));
				L5 = Limit_Data(L5,0.355325,0.241285);
				WheelToJoint_pos Leg_data;
				
				Leg_data.SmallLeg_ToGound_angle = PI +joint_angle - acos((L5*L5 + L2*L2 -L4*L4)/(2*L5*L2)) 
							- acos((L5*L5 +L23*L23 -L3*L3)/(2*L5*L23));
							
				Leg_data.wheel_x = L2*cos(joint_angle) - L1*cos(Leg_data.SmallLeg_ToGound_angle);
				Leg_data.wheel_y = L2*sin(joint_angle) - L1*sin(Leg_data.SmallLeg_ToGound_angle);				
				return Leg_data;
		}
		
		//逆解,由于一个自由度，x不可控，逆解用多项式拟合，测试误差比正常逆解设置x为0要小
		float wheelleg::Leg_Inverse(float target_y){
				float joint_angle = 1.851058*target_y*target_y*target_y - 0.485887*target_y*target_y + 0.923364*target_y - 0.196307;
				return joint_angle;
		}
		//重力补偿，输入alpha，belta都为正，x是由正解得到的,输出扭矩是正号
		float wheelleg::Gravity_Compension(float alpha_,float belta_,float x){
				float alpha = abs(alpha_);
				float belta = abs(belta_);
				float K  =(-L2*sin(alpha) + L23*sin(belta) - sqrt(2)*L4/2.0f)/(L2*cos(alpha)+L23*cos(belta) - sqrt(2)*L4/2.0f);
				float x0 = x;
				float y0 = K*x0 +sqrt(2)*L4/2*(1-K);
				float K1 = (y0+L2*sin(alpha)) /(x0-L2*cos(alpha));
				float costheta1 = abs(1+(K1-K1*x0+y0)*(K+sqrt(2)*L4/2*(1-K)))/(sqrt(1+(K1-K1*x0+y0)*(K1-K1*x0+y0))*sqrt(1+(K+sqrt(2)*L4/2*(1-K))*(K+sqrt(2)*L4/2*(1-K))));
			  float costheta2 = abs(L2*cos(alpha)+K1-K1*x0+y0)/(sqrt(1+L2*cos(alpha)*L2*cos(alpha))*sqrt(1+(K1-K1*x0+y0)*(K1-K1*x0+y0)));
				float theta1 = acos(costheta1);
				float theta2 = acos(costheta2);
				float d = abs(-K1*x0 + y0)/sqrt(K1*K1 + 1);
				float F_joint = ((Body_m/2.0f)*g*sin(theta2))/sin(theta1);
				return F_joint*d;
		}
		
		//求对地作用力
	  float wheelleg::Get_wheel_F_ToGound(pid::Pid VMC_pid,float target_LegH){
			VMC_pid.Pid_Calculate(target_LegH);
			return VMC_pid.Get_Output();
		}
		float wheelleg::FN_ToGound(float Torque){
			float angle0 = acos((L3 * L3 + L23 * L23 - L5 * L5) / (2.0f * L3 * L23));
			float angle3 = acos((L5 * L5 + L23 * L23 - L3 * L3) / (2.0 * L5 * L23));
			float angle4 = acos((L2 * L2 + L5 * L5 - L4 * L4) / (2.0 * L2 * L5));
			float angle6 = PI - angle3 -angle4;
			float L6 = sqrt(L1 * L1 + L2 * L2 - 2.0 * L1 * L2 * cos(angle6));
			float angle1 = acos((L6 * L6 + L2 * L2 - L1 * L1) / (2.0 * L6 * L2));
			float angle2 = acos((L1 * L1 + L6 * L6 - L2 * L2) / (2.0 * L1 * L6));
			float af1 = angle0+angle2;
			float angle20 = PI -angle0-angle2;
			float L0 = sin(angle0) * (L1 + L23) / sin(angle20);
			float l2 = sqrt(L0 * L0 + L1 * L1 - 2.0 * L0 * L1 * cos(angle2));
			float af2 =  acos((L0 * L0 + l2 * l2 - L1 * L1) / (2.0 * L0 * l2));
			float a1 = af1 - af2;
			return (((Torque / L2) / sin(a1)) / sin(af1)) * sin(PI - af1 - af2);
		}
		
		//求不同高度下目标pitch,目标pitch应该在-15度~15度左右
		float wheelleg::Pitch_offset(float x,float y){
			if(y==0){
				y = 0.0000000001f;
			}
			float Tan_theta = x/y;
			return Limit_Data(atan(Tan_theta),0.26,-0.26);
		}
		//VMC控制关节角度
		float wheelleg::VMC_controlleg(float Expect_FN,float joint_angle){
				float joint_Torque_out;
				joint_Torque_out = (14.888256*joint_angle*joint_angle*joint_angle+32.513368*joint_angle*joint_angle+22.759128*joint_angle+5.558670)*Expect_FN;
				return joint_Torque_out;
		}
		
		float wheelleg::Calc_Jointout(float current,float target,float step){
				float delta = target - current;
				if(fabs(delta)<=step){
						return target;
				}
				if(delta > 0){
						return current+step;
				}
				else{
					  return current-step; 
				}
		}
		//刚性控制关节角度，MIT协议，直接用逆解发送目标位置
		float wheelleg::Rigid_controlleg(float target_legH){
			  return Leg_Inverse(-target_legH);
		}

		//离地监测
		bool wheelleg::Check_Ground_off(float FN){
				if(FN < ground_off_FN){
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
				Left_Joint_data.joint_angle = -(Joint_motor[0]->Get_Pos()+LeftJoint_angle_offset);
				
			  Left_Joint_data.joint_angle_d = -Joint_motor[0]->Get_Rpm();
				Left_Joint_data.joint_Torque= -Joint_motor[0]->Get_Torque();
				
				Right_Joint_data.joint_angle = Joint_motor[1]->Get_Pos()+RightJoint_angle_offset;
			
				Right_Joint_data.joint_angle_d = Joint_motor[1]->Get_Rpm();
				Right_Joint_data.joint_Torque= Joint_motor[1]->Get_Torque();
									
				Left_WheelData.Wheel_angle = Wheel_motor[0]->Get_Angle();
				Left_WheelData.Wheel_angle_d = Wheel_motor[0]->Get_Rpm();
				Left_WheelData.Wheel_Torque = Wheel_motor[0]->Get_Torque();
				Left_WheelData.Wheel_x =Left_WheelData.Wheel_xd*Timer_us->Get_DeltaTime(last_count)*0.000001f+Left_WheelData.Wheel_x;
				Left_WheelData.Wheel_xd = -(Wheel_motor[0]->Get_Rpm()*2*PI*Wheel_R*0.0634328358f)/60.0f;
				
				Right_WheelData.Wheel_angle = Wheel_motor[1]->Get_Angle();
				Right_WheelData.Wheel_angle_d = Wheel_motor[1]->Get_Rpm();
				Right_WheelData.Wheel_Torque = Wheel_motor[1]->Get_Torque();
				Right_WheelData.Wheel_x = Right_WheelData.Wheel_xd*Timer_us->Get_DeltaTime(last_count)*0.000001f+Right_WheelData.Wheel_x;
				Right_WheelData.Wheel_xd = (Wheel_motor[1]->Get_Rpm()*2*PI*Wheel_R*0.0634328358f)/60.0f;
									
				Left_wheel_pos = Leg_Kinematics(Left_Joint_data.joint_angle);
				Right_wheel_pos = Leg_Kinematics(Right_Joint_data.joint_angle);
				
				
				LeftVMC_controlleg_pid.Update_Real(-Left_wheel_pos.wheel_y);
				RightVMC_controlleg_pid.Update_Real(-Right_wheel_pos.wheel_y);
				
				Yaw_Pid.Update_Real(Robot_Yaw);
				Roll_Pid.Update_Real(Robot_Roll);
				
				
				chassis_state.x = Limit_Data((Right_WheelData.Wheel_x+Left_WheelData.Wheel_x)/2.0f,1.0f,-1.0f);
				chassis_state.xd = (Left_WheelData.Wheel_xd + Right_WheelData.Wheel_xd)/2.0f;
				chassis_state.q = (IMU->Get_Pitch()*PI)/180.0f;
				chassis_state.qd = (IMU->Get_Gyro()[1]*PI)/180.0f;
								
				Vertical_angle = chassis_state.q;
				Vertical_angled = chassis_state.qd;
				Vertical_angledd = (Vertical_angled - Last_vertical_angled)*Timer_us->Get_DeltaTime(last_count)*0.000001f;
				Last_vertical_angled = Vertical_angled;
				
				Leg_H = (-Left_wheel_pos.wheel_y-Right_wheel_pos.wheel_y)/2.0f;
				Leg_Hd = (Leg_H - Last_LegH)*Timer_us->Get_DeltaTime(last_count)*0.000001f;
				Leg_Hdd = (Leg_Hd-Last_LegHd)*Timer_us->Get_DeltaTime(last_count)*0.000001f;
				Last_LegH = Leg_H;
				Last_LegHd = Leg_Hd;
				
				zm_d = sqrt((IMU->Get_Accel()[0])*(IMU->Get_Accel()[0])+ (IMU->Get_Accel()[2])*(IMU->Get_Accel()[2]))-g;
				zm_dd = (zm_d - last_zm_d)*Timer_us->Get_DeltaTime(last_count)*0.000001f;
				last_zm_d = zm_d;
				
				last_count = Timer_us->Get_TimeStamp();
				
				Left_Expect_FN = FN_ToGound(Left_Joint_data.joint_Torque);
				Right_Expect_FN = FN_ToGound(Right_Joint_data.joint_Torque);
				
				float zw_dd = zm_dd-Leg_Hdd*cos(Vertical_angle) + 2*Leg_Hd*Vertical_angled*sin(Vertical_angle)+Leg_H*Vertical_angledd*sin(Vertical_angle)+Leg_H*Vertical_angled*Vertical_angled*cos(Vertical_angle);
				
				Left_wheelToGound_FN = Left_Expect_FN + Wheel_m*g+ Wheel_m*zw_dd;
				Right_wheelToGound_FN = Right_Expect_FN + Wheel_m*g+ Wheel_m*zw_dd;
				
				Off_flag = (Check_Ground_off(-Left_wheelToGound_FN) && Check_Ground_off(-Right_wheelToGound_FN));
				
				Robot_Yaw = (IMU->Get_Yaw()*PI)/180.0F;
				Robot_Roll = (IMU->Get_Roll()*PI)/180.0F;
				
				Left_Leg_K = Chassis_K_Calculate(-Left_wheel_pos.wheel_y);
				Right_Leg_K = Chassis_K_Calculate(-Right_wheel_pos.wheel_y);
				
				Left_jointout = Calc_Jointout(Joint_motor[0]->Get_target_pos(),-Rigid_controlleg(target_Lleg)-LeftJoint_angle_offset,0.001);
				Right_jointout = Calc_Jointout(-Joint_motor[1]->Get_target_pos(),-(Rigid_controlleg(target_Rleg)-RightJoint_angle_offset),0.001);
				
				Yaw_Pid.Pid_Calculate(0.0f);
		}
		void wheelleg::Printf_Data(){
				uart_printf("%f,%f,%f,%f\n",Left_wheel_pos.wheel_y,Right_wheel_pos.wheel_y,Left_Joint_data.joint_Torque,Right_Joint_data.joint_Torque);
		}
		void wheelleg::Set_targetLeg(float Lleg,float Rleg){
				target_Lleg = Lleg;
				target_Rleg = Rleg;
		}
		void wheelleg::Kinematics_calc(vector2d::Vector2D v_, float vw_){
				Update_Data();//数据更新
				
				//运动时位移零点重置
//				if(v_.y() != 0){
//					State_target.x = chassis_state.x;
//				}
//				if((last_v.x() != 0 && v_.x() == 0) || (last_v.y() != 0 && v_.y() == 0)){
//						move_stop_flag = 1;
//				}
//				if((move_stop_flag == 1) && (abs(chassis_state.xd) < STOP_SPEED)){
//						move_stop_flag = 0;
//						State_target.x = chassis_state.x;//重置位移零点
//				}
//				//被外部推动
//				if(abs(chassis_state.xd) > MAX_SPEED){
//						State_target.x = chassis_state.x;//重置位移零点
//				}
				
//				Left_wheelout = LeftWheel_Dir*(v_.y() + v_.x()*vw_);
//				Right_wheelout =  RightWheel_Dir*(v_.y() - v_.x()*vw_);
				
				if(Off_flag){
						Left_wheelout = LeftWheel_Dir*LQR_Calculate_offGound(Left_Leg_K,chassis_state);
						Right_wheelout = RightWheel_Dir*LQR_Calculate_offGound(Right_Leg_K,chassis_state);
				}
				else{
						Left_wheelout = LeftWheel_Dir*(LQR_Calculate(Left_Leg_K,chassis_state)-Yaw_Pid.Get_Output());
						Right_wheelout = RightWheel_Dir*(LQR_Calculate(Right_Leg_K,chassis_state)+Yaw_Pid.Get_Output());
				}
				
//				Wheel_motor[0]->Set_Torque(Left_wheelout + Yaw_Pid.Pid_Calculate(Robot_Yaw));
//				Wheel_motor[1]->Set_Torque(Right_wheelout - Yaw_Pid.Pid_Calculate(Robot_Yaw));

				Left_Torque_out = Gravity_Compension(Left_Joint_data.joint_angle,Left_wheel_pos.SmallLeg_ToGound_angle,Left_wheel_pos.wheel_x);
				Right_Torque_out = -Gravity_Compension(Right_Joint_data.joint_angle,Right_wheel_pos.SmallLeg_ToGound_angle,Right_wheel_pos.wheel_x);
				
//				Joint_motor[0]->Set_Pos(Limit_Data(Left_jointout,0.0,-0.75));
//				Joint_motor[1]->Set_Pos(-Limit_Data(Right_jointout,0.0,-0.75));
				
//				Joint_motor[0]->Set_Torque(Left_Torque_out);
//				Joint_motor[1]->Set_Torque(Right_Torque_out);

				
		}
			// 再次初始化
		void wheelleg::Chassis_Re_Init(){
				Set_Is_Init_False();
		}
		
		// 底盘初始化
		void wheelleg::Chassis_Init(){
			
			
			
			
			
			Set_Is_Init_True();
			
		}	


}
