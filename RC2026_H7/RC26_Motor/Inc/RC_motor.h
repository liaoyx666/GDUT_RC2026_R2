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
		/**
		* @brief 设置位置范围
		* @note rad
		* @param pos_max_:位置最大值
		* @param pos_min_:位置最小值
		*/
		void Set_Pos_limit(float pos_max_, float pos_min_)
		{
			if (pos_min_ > pos_max_) return;
			if (pos_max_ > 6000) pos_max_ = 6000;
			if (pos_min_ < -6000) pos_min_ = -6000;
			pos_max = pos_max_;
			pos_min = pos_min_;
		}
		
		constexpr void Set_Out_Pos_limit(float out_pos_max_, float out_pos_min_)
		{
			Set_Pos_limit(out_pos_max_ * gear_ratio, out_pos_min_ * gear_ratio);
		}
		
		/**
		* @brief 设置目标转速
		* @note rpm
		* @param target_rpm_:目标转速
		*/
		constexpr void Set_Rpm(float target_rpm_)
		{
			target_rpm = target_rpm_;
			
			// 设置模式
			motor_mode = RPM_MODE;
		}
		
		/**
		* @brief 设置目标位置
		* @note rad
		* @param target_pos_:目标位置
		*/
		void Set_Pos(float target_pos_)
		{
			if (target_pos_ > pos_max) target_pos = pos_max;
			else if (target_pos_ < pos_min) target_pos = pos_min;
			else target_pos = target_pos_;
			
			// 设置模式
			motor_mode = POS_MODE;	
		}
		
		/**
		* @brief 重置电机位置
		* @note rad
		* @param pos_:位置
		*/
		constexpr void Reset_Pos(float pos_) { pos_offset = pos_ - pos; }

		/**
		* @brief 设置输出轴转速
		* @note rpm
		* @param target_out_rpm_:期望输出轴转速
		*/
		constexpr void Set_Out_Rpm(float target_out_rpm_) { Set_Rpm(target_out_rpm_ * gear_ratio); }

		/**
		* @brief 设置输出轴位置
		* @note rad
		* @param target_out_rpm_:期望输出轴位置
		*/
		constexpr void Set_Out_Pos(float target_out_pos_) { Set_Pos(target_out_pos_ * gear_ratio); }
		
		/**
		* @brief 重置输出轴位置
		* @note rad
		* @param out_pos_:输出轴位置
		*/
		constexpr void Reset_Out_Pos(float out_pos_) { Reset_Pos(out_pos_ * gear_ratio); }
		
		constexpr void Set_Pos_Offset(float pos_offset_) {pos_offset = pos_offset_;}

		// 获取参数
		constexpr float Get_Rpm() const {return rpm;}
		constexpr float Get_Pos() const {return pos;}
		constexpr float Get_Out_Rpm() const {return rpm / gear_ratio;}
		constexpr float Get_Out_Pos() const {return pos / gear_ratio;}
		constexpr float Get_Temperature() const {return temperature;}
		
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
		
		/**
		* @brief 设置目标扭矩
		* @note N * m
		* @param target_torque_:目标扭矩
		*/
		constexpr void Set_Torque(float target_torque_)
		{
			target_torque = target_torque_;
			
			// 设置模式
			motor_mode = TORQUE_MODE;
		}
		
		/**
		* @brief 设置输出轴目标扭矩
		* @note N * m
		* @param target_out_torque_:输出轴目标扭矩
		*/
		constexpr void Set_Out_Torque(float target_out_torque_) { Set_Torque(target_out_torque_ / gear_ratio); }
		
		void Set_Mit_Pos(float pos_)
		{
			if (pos_ > pos_max) target_pos = pos_max;
			else if (pos_ < pos_min) target_pos = pos_min;
			else target_pos = pos_;
			
			// 设置模式
			motor_mode = LOCAL_MIT_MODE;
		}
		
		constexpr void Set_Mit_Rpm(float rpm_)
		{
			target_rpm = rpm_;
			
			// 设置模式
			motor_mode = LOCAL_MIT_MODE;
		}
		
		constexpr void Set_Mit_Tor(float tor_)
		{
			ff_torque = tor_;
			
			// 设置模式
			motor_mode = LOCAL_MIT_MODE;
		}
		
		constexpr void Set_Out_Mit_Pos(float out_pos_) { Set_Mit_Pos(out_pos_ * gear_ratio); }
		constexpr void Set_Out_Mit_Rpm(float out_rpm_) { Set_Mit_Rpm(out_rpm_ * gear_ratio); }
		constexpr void Set_Out_Mit_Tor(float out_tor_) { Set_Mit_Tor(out_tor_ / gear_ratio); }

		constexpr float Get_Torque() const {return torque;}
		constexpr float Get_Out_Torque() const {return torque * gear_ratio;}

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
	// 预计算转换系数，避免重复计算
	#ifndef RPM_TO_RADPS_RATIO
	#define RPM_TO_RADPS_RATIO 	((2.0f * PI) / 60.0f)
	#endif

	#ifndef RADPS_TO_RPM_RATIO
	#define RADPS_TO_RPM_RATIO 	(60.0f / (2.0f * PI))
	#endif

	/**
	 * @brief 将转速从RPM(转/分钟)转换为rad/s(弧度/秒)
	 * @param rpm 转速，单位：转/分钟
	 * @return 角速度，单位：弧度/秒
	 */
	constexpr float rpm_to_radps(float rpm)
	{
		return rpm * RPM_TO_RADPS_RATIO;
	}

	/**
	 * @brief 将角速度从rad/s(弧度/秒)转换为RPM(转/分钟)
	 * @param radps 角速度，单位：弧度/秒
	 * @return 转速，单位：转/分钟
	 */
	constexpr float radps_to_rpm(float radps)
	{
		return radps * RADPS_TO_RPM_RATIO;
	}
	
	int float_to_uint(float x_float, float x_min, float x_max, int bits);
	float uint_to_float(int x_int, float x_min, float x_max, int bits);
}
#endif
