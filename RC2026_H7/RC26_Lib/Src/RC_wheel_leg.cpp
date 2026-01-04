#include "RC_wheel_leg.h"

namespace chassis
{
	wheelleg::wheelleg(){
		target_state<<0.0f,
					  0.0f,
					  0.0f,
					  0.0f;
		state<<0.0f,
			   0.0f,
			   0.0f,
			   0.0f;
		joint_pid.set_param(0.0,0.0,0.0,0.3);
	}
	void wheelleg::pid_controlleg(){
		joint_pid.set_target(target_leg);
		joint_pid.pid_calc(Leg_H);
		joint_out = joint_pid.get_output();
	}
	void wheelleg::VMC_controlleg(){
		float F_X_ = 0.0f;
		float F_Y_;
		joint_pid.set_target(target_leg);
		joint_pid.pid_calc(Leg_H);
		F_Y_ = joint_pid.get_output();

		joint_to_wheel_VX = joint_W*L2 *cos(Leg_angle)*cos(PI/2 - Leg_angle+joint_angle);
		joint_to_wheel_VY = joint_W*L2 *cos(Leg_angle)*cos(PI/2 - Leg_angle+joint_angle);
		joint_out = joint_to_wheel_VX * F_X_ + joint_to_wheel_VY*F_Y_;
	}
	
	void wheelleg::LQR_controlwheel(){
		wheel_out = K*(state - target_state);
	}
	
	void wheelleg::K_calc(){
		float k1 =0.0f;
		float k2 =0.0f;
		float k3 =0.0f;
		float k4 =0.0f;
		K<<k1,
		   k2,
		   k3,
		   k4;
	}
	
	void wheelleg::State_update(MathLib::matrix<4,1> input_state){
		float joint_alpha;
		Leg_Kinematics(joint_alpha);
		Leg_H = -this->wheel_y;
		joint_T = 0.0;
		joint_W =0.0;
		joint_angle = 0.0;
		F_X = joint_T/L2 *cos(Leg_angle)*cos(PI/2 - Leg_angle+joint_angle);
		F_Y = joint_T/L2 *cos(Leg_angle)*cos(PI/2 - Leg_angle+joint_angle);
		state = input_state;
//		state<<0.0f,//q
//			   0.0f,//qd
//			   0.0f,//x
//			   0.0f;//xd
		
	}
	
	void wheelleg::set_target(float x,float xd,float angle,float angle_d,float Leg_h){
		target_state<<angle,
					  angle_d,
					  x,
					  xd;
		target_leg = Leg_h;
	}
	
	bool wheelleg::Ground_off(){
		float zdd_wheel;
		float zdd_body;
		float dt;
		zdd_wheel = zdd_body - Leg_H*dt*dt*cos(state(0,0))
					+2*Leg_H*dt*sin(state(1,0)) + Leg_H*state(1,0)*dt*sin(state(0,0)) 
					+ Leg_H*state(1,0)*state(1,0)*cos(state(0,0)); 
		float FN = F_Y + Wheel_m*g +Wheel_m*zdd_wheel;
		if(FN > ground_off_FN){
				return true;
		}
		else {
			return false;
		}
	}


}