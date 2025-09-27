#include "RC_chassis.h"


namespace RC_chassis{

	//1，设置底盘参数，类型
	//2，姿态控制模式，运动模式
	//3，输入底盘Vx,Vy,W
	//4,解算至各轮上
	
	
    template <typename chassis_t>
    void omni3_Chassis<chassis_t>::omni3_chassis_calc(float wheel_speeds[3], float vx, float vy, float wz){
        // 计算三轮全向轮底盘的轮子速度
        wheel_speeds[0] = vx - (1.732/2)*vy - this->info.R* wz; // 左轮
        wheel_speeds[1] = vx + (1.732/2)*vy + this->info.R* wz; // 右轮
        wheel_speeds[2] = (1.732)*vy - this->info.R* wz; // 后轮
    }
    template <typename chassis_t>
    void omni4_Chassis<chassis_t>::omni4_chassis_calc(float wheel_speeds[4], float vx, float vy, float wz){
        // 计算四轮全向轮底盘的轮子速度
		wheel_speeds[0] = (vx * cos45 + vy * sin45) - wz * this->info.R;  // 前左
		wheel_speeds[1] = (-vx * cos45 + vy * sin45) + wz * this->info.R; // 前右
		wheel_speeds[2] = (-vx * cos45 - vy * sin45) - wz * this->info.R; // 后右
		wheel_speeds[3] = (vx * cos45 - vy * sin45) + wz * this->info.R;  // 后左
    }
	
	template class RC_chassis::omni4_Chassis<float>;
}
