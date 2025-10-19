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
		ANGLE_MODE,// 角度模式（0~2pi）
		CURRENT_MODE,//电流模式
		TORQUE_MODE,//力矩模式
		MIT_MODE,// mit模式
	} MotorMode;
	
	class Motor
    {
    public:
		Motor(float gear_ratio_ = 1.f);
		virtual ~Motor() {}
		
		// 设置参数
		void Set_Pos_limit(float pos_max_, float pos_min_);
		void Set_Out_Pos_limit(float out_pos_max_, float out_pos_min_);
		
		void Set_Rpm(float target_rpm_);
		void Set_Pos(float target_pos_);
		virtual void Set_Angle(float target_angle_);//有些电机没有角度控制
		virtual void Set_Current(float target_current_);//有些电机没有电流控制
		virtual void Set_Torque(float target_torque_);// 有些电机没有力矩控制
		
		
		void Set_Out_Rpm(float target_out_rpm_);// 设置输出轴转速
		void Set_Out_Pos(float target_out_pos_);// 设置输出轴位置
		
		virtual void Set_K_Pos(float target_k_pos_);// 设置刚度系数kp
		virtual void Set_K_Spd(float target_k_spd_);// 设置阻尼系数kd
		
		void Reset_Out_Pos(float out_pos_);// 重置输出轴位置
		void Reset_Pos(float pos_);// 重置转子位置
		
		
		// 获取参数
		float Get_Rpm() const {return rpm;}
		float Get_Pos() const {return pos;}
		float Get_Out_Pos() const {return out_pos;}
		float Get_Angle() const {return angle;}
		float Get_Current() const {return current;}
		float Get_Torque() const {return torque;}
		float Get_Temperature() const {return temperature;}
		
		
    protected:
		// 真实参数
		float rpm = 0;// r/minute
		float angle = 0;// rad 0 ~ 2pi
		float pos = 0;// rad
		float current = 0;
		float temperature = 0;
		float torque = 0;// N*m
		float k_spd = 0;// 阻尼系数
		float k_pos = 0;// 刚度系数
		
		// 目标参数
		float target_rpm = 0;
		float target_angle = 0;
		float target_pos = 0;
		float target_current = 0;
		float target_torque = 0;
		float target_k_spd = 0;
		float target_k_pos = 0;
	
		float out_pos = 0;// 输出轴位置
	
		float pos_offset = 0;// 位置偏移量 pos = 电机读取位置 + pos_offset
	
		int32_t cycle = 0;// 转子累计旋转圈数
	
		float last_angle = 0;

		float pos_max = 6000;
		float pos_min = -6000;
	
	
		float gear_ratio = 1.f;// 减速比

		MotorMode motor_mode = RPM_MODE;// 模式

    private:
		
    };
	
	
	
	
	
	
	
	
	
	int float_to_uint(float x_float, float x_min, float x_max, int bits);
	float uint_to_float(int x_int, float x_min, float x_max, int bits);
}
#endif
