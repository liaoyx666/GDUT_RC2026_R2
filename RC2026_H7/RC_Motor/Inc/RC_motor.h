#pragma once
#include <math.h>
#include "RC_pid.h"
#include "RC_can.h"
#include "RC_tim.h"

#ifdef __cplusplus
namespace motor
{
	typedef enum MotorMode
	{
		RPM_MODE,// 转速模式
		POS_MODE,// 位置模式
		ANGLE_MODE// 角度模式（0~2pi）
	} MotorMode;
	
	class Motor
    {
    public:
		Motor();
		virtual ~Motor() {}
			
		void Set_Pos_limit(float pos_limit_);
		void Set_Rpm(float target_rpm_);
		void Set_Angle(float target_angle_);
		void Set_Pos(float target_pos_);
		
		float rpm = 0, angle = 0, pos = 0, current = 0, temperature = 0, torque = 0;// 真实参数
		
    protected:
		float target_rpm = 0, target_angle = 0, target_pos = 0, target_current = 0, target_torque = 0;// 目标参数
		int32_t cycle = 0;// 圈数
		float last_angle = 0;
		float pos_limit = 6434;
		
		MotorMode motor_mode = RPM_MODE;

    private:
		
    };
}
#endif
