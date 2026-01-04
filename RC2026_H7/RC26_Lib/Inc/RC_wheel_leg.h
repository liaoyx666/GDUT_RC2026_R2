#pragma once
#include <arm_math.h>
#include "RC_Matrix.h"
#include "RC_pid.h"
#ifdef __cplusplus
namespace chassis
{
#define Joint_angle_min -1
	#define Joint_angle_max -0.2
	#define ground_off_FN   0.0
	const float L1 = 0.282;
	const float L2 = 0.2856;
	const float L3 = 0.29281;
	const float L4 = 0.13067;
	const float L23 = 0.05702;
	const float g = 9.8;
	const float Wheel_m = 0.287;
	const float Body_m = 0.0;
	
	class wheelleg
	{
		public:
			wheelleg();
			~wheelleg(){}
			void pid_controlleg();
			void VMC_controlleg();
			void K_calc();
			void LQR_controlwheel();
			void State_update(MathLib::matrix<4,1> input_state);
			bool Ground_off();
			void set_target(float x,float xd,float angle,float angle_d,float Leg_h);
			void Leg_Kinematics(float joint_angle_){
				if(joint_angle_ < Joint_angle_min){
					joint_angle_ = Joint_angle_min;
				}
				else if(joint_angle_ > Joint_angle_max){
					joint_angle_ = Joint_angle_max;
				}
				
				float L5 = sqrt(L2*L2 + L4*L4 -2*L2*L4*cos(PI/4.0f - joint_angle_));
				Leg_angle = PI +joint_angle_ - acos((L5*L5 + L2*L2 -L4*L4)/(2*L5*L2)) 
							- acos((L5*L5 +L23*L23 -L3*L3)/(2*L5*L23));
				wheel_x = L2*cos(joint_angle_) - L1*cos(Leg_angle);
				wheel_y = L2*sin(joint_angle_) - L1*sin(Leg_angle);		
			}
			void Leg_Inverse(float Y){
				float X = -0.000001;
				float phi = atan(-Y/X);
				float R = sqrt(X*X +Y*Y);
				float middle_val1 = 2*L2*R;
				float middle_val2 = L2*L2 -L1*L1 + R*R;
				float acos_arg = middle_val2 / middle_val1;
				
				float angle_1 = phi - acos(acos_arg);
				float angle_2 = phi + acos(acos_arg);
				if(angle_1 < Joint_angle_max && angle_1 > Joint_angle_min){
					joint_angle = angle_1;
				}
				if(angle_2 < Joint_angle_max && angle_2 > Joint_angle_min){
					joint_angle = angle_2;
				}
			}
			
		private:
			MathLib::matrix<1,4> K;
			MathLib::matrix<4,1> state;
			MathLib::matrix<4,1> target_state;
			float target_leg;
			float Leg_H;
			float wheel_x,wheel_y;
			float Leg_angle;
			float joint_angle;
			float F_X,F_Y;
			float joint_T;
			float joint_W;
			float joint_to_wheel_VX;
			float joint_to_wheel_VY;
			pid::simple_pid joint_pid;
			float joint_out;
			MathLib::matrix<1,1> wheel_out;
	};
}
#endif
