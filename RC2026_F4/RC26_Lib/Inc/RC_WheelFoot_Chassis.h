#pragma once
#include "RC_pid.h"
#include "arm_math.h"

#ifdef __cplusplus
namespace RC_WheelFoot_Chassis{
	//0:左 1:右
	typedef struct {
		float X_Pose[2];
		float Pitch[2];
		float X_speed[2];
		float Gyro[2];
	}State_data;
	typedef struct {
		float Left_angle;
		float Right_angle;
	} Joint_angle;

	typedef struct {
		float Left_L;
		float Right_L;
	}L_target;
	typedef struct{
		float K1[2];
		float K2[2];
		float K3[2];
		float K4[2];
	}K_data;

	class WheelFoot_control{
		public:
			WheelFoot_control(){};
			void WheelFoot_ChassisControl(L_target L,State_data state,Joint_angle joint_angle,State_data target_state);
			void FN_Get(State_data state,Joint_angle joint_angle);//获取对地压力，用于离地检测
		private:
			K_data K_calc(L_target now_L);
			L_target Get_Now_L(Joint_angle joint_angle);
			void Joint_Control(L_target L,L_target now_L);
			void Wheel_Control(State_data state,K_data K,State_data target_state);
	};

}


#endif
