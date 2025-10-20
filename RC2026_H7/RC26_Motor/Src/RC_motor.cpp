#include "RC_motor.h"

namespace motor
{
	Motor::Motor(float gear_ratio_) : gear_ratio(gear_ratio_)
	{
		
	}
	
	
	/**
    * @brief 设置位置范围
    * @note rad
    * @param pos_max_:位置最大值
	* @param pos_min_:位置最小值
    */
	void Motor::Set_Pos_limit(float pos_max_, float pos_min_)
	{
		if (pos_min_ > pos_max_) return;
		
		if (pos_max_ > 6000) pos_max_ = 6000;
		if (pos_min_ < -6000) pos_min_ = -6000;
		
		pos_max = pos_max_;
		pos_min = pos_min_;
	}
	
	
	/**
    * @brief 设置目标转速
    * @note rpm
    * @param target_rpm_:目标转速
    */
	void Motor::Set_Rpm(float target_rpm_)
	{
		// 设置模式
		motor_mode = RPM_MODE;
		target_rpm = target_rpm_;
	}
	
	
	/**
    * @brief 设置目标角度
    * @note 0 rad ~ 2pi rad
    * @param target_angle_:目标角度
    */
	void Motor::Set_Angle(float target_angle_)
	{
		if (target_angle_ >= TWO_PI) target_angle = 0;
		else if (target_angle_ <= 0) target_angle = 0;
		else target_angle = target_angle_;
		
		// 设置模式
		motor_mode = ANGLE_MODE;
		target_angle = target_angle_;
	}
	
	/**
    * @brief 设置目标位置
    * @note rad
    * @param target_pos_:目标位置
    */
	void Motor::Set_Pos(float target_pos_)
	{
		if (target_pos_ > pos_max) target_pos = pos_max;
		else if (target_pos_ < pos_min) target_pos = pos_min;
		else target_pos = target_pos_;
		
		// 设置模式
		motor_mode = POS_MODE;
		target_pos = target_pos_;
	}
	
	
	/**
    * @brief 设置目标电流
    * @note 不同电机单位可能不同
    * @param target_current_:目标电流
    */
	void Motor::Set_Current(float target_current_)
	{
		// 设置模式
		motor_mode = CURRENT_MODE;
		target_current = target_current_;
	}
	
	/**
    * @brief 设置目标扭矩
    * @note N * m
    * @param target_torque_:目标扭矩
    */
	void Motor::Set_Torque(float target_torque_)
	{
		// 设置模式
		motor_mode = TORQUE_MODE;
		target_torque = target_torque_;
	}
	
	
	/**
    * @brief 重置电机位置
    * @note rad
    * @param pos_:位置
    */
	void Motor::Reset_Pos(float pos_)
	{
		pos_offset = pos_ - pos;
	}
	
	
	
	
	
	/**
    * @brief 设置输出轴转速
    * @note rpm
    * @param target_out_rpm_:期望输出轴转速
    */
	void Motor::Set_Out_Rpm(float target_out_rpm_)
	{
		Set_Rpm(target_out_rpm_ * gear_ratio);
	}
	
	
	
	/**
    * @brief 设置输出轴位置
    * @note rad
    * @param target_out_rpm_:期望输出轴位置
    */
	void Motor::Set_Out_Pos(float target_out_pos_)
	{
		Set_Pos(target_out_pos_ * gear_ratio);
	}

	
	/**
    * @brief 设置刚度系数kp
    * @note 
    * @param target_k_pos_:刚度系数kp
    */
	void Motor::Set_K_Pos(float target_k_pos_)
	{
		
	}
	
	
	/**
    * @brief 设置阻尼系数kd
    * @note 
    * @param target_k_spd_:阻尼系数kd
    */
	void Motor::Set_K_Spd(float target_k_spd_)
	{

	}

	
	/**
    * @brief 重置输出轴位置
    * @note rad
    * @param out_pos_:输出轴位置
    */
	void Motor::Reset_Out_Pos(float out_pos_)
	{
		Reset_Pos(out_pos_ * gear_ratio);
	}
	
	
	
	void Motor::Set_Feedforward(float feedforward_)
	{
		feedforward = feedforward_;
	}
	
	
	
	
	int float_to_uint(float x_float, float x_min, float x_max, int bits)
	{
		/* Converts a float to an unsigned int, given range and number of bits */
		float span = x_max - x_min;
		float offset = x_min;
		return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
	}
	
	
	
	
	float uint_to_float(int x_int, float x_min, float x_max, int bits)
	{
		/* converts unsigned int to float, given range and number of bits */
		float span = x_max - x_min;
		float offset = x_min;
		return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
	}
	
	
	
	
	
}