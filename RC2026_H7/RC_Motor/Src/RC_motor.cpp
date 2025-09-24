#include "RC_motor.h"

namespace motor
{
	Motor::Motor()
	{
		
	}
	
	void Motor::Set_Pos_limit(float pos_limit_)
	{
		pos_limit_ = fabsf(pos_limit_);
		if (pos_limit_ > 6000) pos_limit = 6000;
		else pos_limit = pos_limit_;
	}
	
	
	void Motor::Set_Rpm(float target_rpm_)
	{
		motor_mode = RPM_MODE;
		target_rpm = target_rpm_;
	}
	
	
	void Motor::Set_Angle(float target_angle_)
	{
		if (target_angle_ >= TWO_PI) target_angle = 0;
		else if (target_angle_ <= 0) target_angle = 0;
		else target_angle = target_angle_;
		
		motor_mode = ANGLE_MODE;
		target_angle = target_angle_;
	}
	
	
	void Motor::Set_Pos(float target_pos_)
	{
		if (target_pos_ > pos_limit) target_pos = pos_limit;
		else if (target_pos_ < -pos_limit) target_pos = -pos_limit;
		else target_pos = target_pos_;
		
		motor_mode = POS_MODE;
		target_pos = target_pos_;
	}
}