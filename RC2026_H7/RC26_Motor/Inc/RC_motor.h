#pragma once
#include <math.h>

#include "RC_pid.h"
#include "RC_can.h"
#include "RC_tim.h"
// Header: 电机基类
// File Name: 
// Author:
// Date:

#ifdef __cplusplus
namespace motor
{
	enum MotorMode : uint8_t
	{
		RPM_MODE = 0,	// 转速模式
		POS_MODE,		// 位置模式
		ANGLE_MODE,		// 角度模式(0~2pi)
		OUT_ANGLE_MODE,	// 输出轴角度模式(0~2pi)
		CURRENT_MODE,	// 电流模式
		TORQUE_MODE,	// 力矩模式
		LOCAL_MIT_MODE	// 本地计算mit模式（有些电机自带mit算法，如果要使用电机自带算法，需使use_mit = true）
	};

	class Motor
    {
    public:
		Motor(float gear_ratio_ = 1.f, bool is_reset_pos_ = false);
		~Motor() = default;
		
		// 设置参数
		void Set_Pos_limit(float pos_max_, float pos_min_);
		void Set_Out_Pos_limit(float out_pos_max_, float out_pos_min_);
		void Set_Rpm(float target_rpm_);
		void Set_Pos(float target_pos_);

		void Set_Out_Rpm(float target_out_rpm_);// 设置输出轴转速
		void Set_Out_Pos(float target_out_pos_);// 设置输出轴位置
		void Set_Pos_Offset(float pos_offset_) {pos_offset = pos_offset_;}
		void Reset_Out_Pos(float out_pos_);// 重置输出轴位置
		void Reset_Pos(float pos_);// 重置转子位置
		
		// 获取参数
		float Get_Rpm() const {return rpm;}
		float Get_Pos() const {return pos;}
		float Get_Out_Rpm() const {return rpm / gear_ratio;}
		float Get_Out_Pos() const {return pos / gear_ratio;}
		float Get_Temperature() const {return temperature;}
		
		// 速度环pid，位置环pid
		pid::Pid pid_spd, pid_pos;
		
    protected:
		// 真实参数
		float rpm = 0;// (r/min)
		float pos = 0;// (rad)
		float temperature = 0;
		float torque = 0;// (N*m)
		
		// 目标参数
		float target_rpm = 0;
		float target_pos = 0;
		
		// 变量
		float pos_offset = 0;// 位置偏移量(pos = 电机读取位置 + pos_offset)
		
		// 电机参数
		float pos_max = 6000;
		float pos_min = -6000;
		float gear_ratio = 1;// 减速比
		MotorMode motor_mode = RPM_MODE;// 电机模式
		
		bool is_reset_pos = false;
    };
	
	/*关节电机*/
	class JointM : public Motor
    {
    public:
		JointM(float gear_ratio_ = 1.f, bool is_reset_pos_ = false);
		~JointM() = default;	
		
		void Set_Torque(float target_torque_);// 
		void Set_Out_Torque(float target_out_torque_);
		
		float Get_Torque() const {return torque;}
		float Get_Out_Torque() const {return torque * gear_ratio;}
		
		void Set_Mit_Pos(float pos_);
		void Set_Mit_Rpm(float rpm_);
		void Set_Mit_Tor(float tor_);

		void Set_Out_Mit_Pos(float out_pos_);
		void Set_Out_Mit_Rpm(float out_rpm_);
		void Set_Out_Mit_Tor(float out_tor_);
		
		virtual void Set_K_Pos(float target_k_pos_) = 0;// 设置刚度系数kp
		virtual void Set_K_Spd(float target_k_spd_) = 0;// 设置阻尼系数kd
    protected:
		
		float target_torque = 0;
		
		float k_spd = 0;// 阻尼系数kd
		float k_pos = 0;// 刚度系数kp
		
		float target_k_spd = 0;
		float target_k_pos = 0;
		float ff_torque = 0;// 前馈力矩
    };
	
	// 工具函数
	int float_to_uint(float x_float, float x_min, float x_max, int bits);
	float uint_to_float(int x_int, float x_min, float x_max, int bits);
	
	float rpm_to_radps(float rpm_);
	float radps_to_rpm(float radps_);
}
#endif
