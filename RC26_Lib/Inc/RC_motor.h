#pragma once
#include <math.h>
#include "RC_pid.h"



#ifdef __cplusplus


namespace motor
{
	typedef enum MotorMode
	{
		RPM_MODE,// 转速模式
		POS_MODE,// 位置模式
		ANGLE_MODE// 角度模式（-pi~pi）
	} MotorMode;
	
	typedef enum MoveMode
	{
		SHORTEST,// 最短路径
		NORMAL// 正常路径
	} MoveMode;
	
	
	
	class Motor
    {
    public:
		Motor();
		virtual ~Motor() {}
		
		
		void Set_Pos_limit(float pos_limit_)
		{
			pos_limit_ = fabsf(pos_limit_);
			if (pos_limit_ > 6434) pos_limit = 6434;
			else pos_limit = pos_limit_;
		}
		
		
		void Set_Rpm(float target_rpm_)
		{
			motor_mode = RPM_MODE;
			target_rpm = target_rpm_;
		}
		
		
		void Set_Angle(float target_angle_, MoveMode move_mode_ = SHORTEST)
		{
			if (target_angle_ > PI) target_angle = PI;
			else if (target_angle_ < -PI) target_angle = -PI;
			else target_angle = target_angle_;
			
			motor_mode = ANGLE_MODE;
			target_angle = target_angle_;
			move_mode = move_mode_;
		}
		
		
		void Set_Pos(float target_pos_, MoveMode move_mode_ = NORMAL)
		{
			if (target_pos_ > pos_limit) target_pos = pos_limit;
			else if (target_pos_ < -pos_limit) target_pos = -pos_limit;
			else target_pos = target_pos_;
			
			motor_mode = POS_MODE;
			target_pos = target_pos_;
			move_mode = move_mode_;
		}
		float rpm = 0, angle = 0, pos = 0, current = 0;
    protected:
		
		float target_rpm = 0, target_angle = 0, target_pos = 0, target_current = 0;
		int32_t cycle = 0;// 圈数
		float last_angle = 0;
		float temperature = 0;
		
		float pos_limit = 6434;
		
		MotorMode motor_mode = RPM_MODE;
		MoveMode move_mode = NORMAL;

    private:
		
    };

}


#endif
