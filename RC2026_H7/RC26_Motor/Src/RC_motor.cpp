#include "RC_motor.h"

namespace motor
{
	/*----------------------------------Motor------------------------------------------*/
	/*
	 * gear_ratio_ : 减速比
	 * is_reset_pos_ : 是否上电初始化电机位置为0
	 */
	Motor::Motor(float gear_ratio_, bool is_reset_pos_) : gear_ratio(gear_ratio_), is_reset_pos(is_reset_pos_)
	{
		
	}
	
	
	/*----------------------------------JointM------------------------------------------*/
	JointM::JointM(float gear_ratio_, bool is_reset_pos_) : Motor(gear_ratio_, is_reset_pos_)
	{
		
	}
	
	/*----------------------------------工具函数------------------------------------------*/
	int float_to_uint(float x_float, float x_min, float x_max, int bits)
	{
		if (x_float > x_max) x_float = x_max;
		else if (x_float < x_min) x_float = x_min;
		
		float span = x_max - x_min;
		float offset = x_min;
		return (int) ((x_float - offset) * ((float)((1 << bits) - 1)) / span);
	}

	float uint_to_float(int x_int, float x_min, float x_max, int bits)
	{
		float span = x_max - x_min;
		float offset = x_min;
		return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
	}
}