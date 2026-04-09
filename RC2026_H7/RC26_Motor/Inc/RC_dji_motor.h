#pragma once
#include "RC_motor.h"
#include "RC_pid.h"
#include "RC_can.h"
#include "RC_tim.h"

#include <math.h>

#ifdef __cplusplus
namespace motor
{
	enum MotorPol : int8_t
	{
		POL_SYN = 1,  /*同向*/
		POL_REV = -1, /*反向*/
	};
	
	class DjiMotor : public Motor, public can::CanHandler, public tim::TimHandler
    {
    public:
		DjiMotor(can::Can &can_, tim::Tim *tim_, float gear_ratio_ = 1.f, bool is_reset_pos_angle = false);
		virtual ~DjiMotor() {}

		void Set_Current(float target_current_);//有些电机没有电流控制
		float Get_Current() const {return current;}
		void Set_Out_Angle(float target_out_angle_);// 设置输出轴角度
		float Get_Out_Angle(); // 0 ~ 2pi*/
		float Get_Angle();
		void Set_Angle(float target_angle_);//有些电机没有角度控制
		void Reset_Out_Angle(float out_angle_);// 重置输出轴角度0 ~ 2pi
		
    protected:
		virtual void Dji_Id_Init(uint8_t id_) = 0;// 初始化发送和接受帧的id
		
		void CanHandler_Register() override;
		void Tim_It_Process() override;
		void Can_Tx_Process() override;
		void Can_Rx_It_Process(uint32_t rx_id_, uint8_t *rx_data) override;

		bool can_rx_is_first = true;

		uint8_t id;
		
		float target_angle = 0;
		float current = 0;
		float target_current = 0;
		
		int16_t cycle = 0;// 转子累计旋转圈数(用于计算pos)
		int16_t last_encoder = 0;// 上一次编码器角度(0 ~ 8191)
		
		int32_t out_angle_max;
		int32_t out_angle_int = 0;
		int16_t rotor_cycle = 0;// 转子累计旋转圈数(用于计算out_angle)
		int32_t out_angle_offset = 0;
		
		int16_t encoder = 0; // (0 ~ 8191)
		bool is_gear_ratio_int; /*gear_ratio是否为整数*/
		
		float angle = 0;
		float tor_to_cur = 1.f/*力矩换为电流的系数*/;
    };

}
#endif