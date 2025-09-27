#pragma once
#include "RC_motor.h"
#include "RC_pid.h"
#include "RC_can.h"
#include "RC_tim.h"

#ifdef __cplusplus


enum GoControlMode : uint8_t
{
	GO_CONTROL_MODE_1 = 10,// 每控制一次电机CAN就返回一次电机数据
	GO_CONTROL_WRITE_K = 11,// 设置k_spd,k_pos
	GO_CONTROL_READ_K = 12,// 读取k_spd,k_pos
	GO_CONTROL_MODE_2 = 13// 只有报错时会返回电机数据
};

enum GoMotorStatus : uint8_t
{
	GO_STATUS_LOCK = 0,// 锁定(Default)
	GO_STATUS_FOC = 1,// FOC 闭环
	GO_STATUS_CALIBRATE = 3// 编码器校准
};

enum GoError : uint8_t
{
	GO_NORMAL = 0,// 正常
	GO_OVER_HEAT = 1,// 过热
	GO_OVER_CURRENT = 2,// 过流
	GO_OVER_VOLTAGE = 3,// 过压
	GO_ENCODER_ERROR = 4// 编码器故障
};


namespace motor
{
	//!!!go电机的canh和canl是反的，和大疆电机不一样!!!//
	class Go : public Motor, public can::CanHandler, public tim::TimHandler
    {
    public:
		Go(uint8_t id_, uint8_t module_id_, can::Can &can_, tim::Tim &tim_, bool torque_only_ = true, float k_spd_ = 0, float k_pos_ = 0);
		virtual ~Go() {}
		
		void Set_K_Pos(float target_k_pos_)
		{
			if (target_k_pos_ < 0) target_k_pos = 0;
			else if (target_k_pos_ > 25.599f) target_k_pos = 25.599f;
			else target_k_pos = target_k_pos_;
		}
		
		void Set_K_Spd(float target_k_spd_)
		{
			if (target_k_spd_ < 0) target_k_spd = 0;
			else if (target_k_spd_ > 25.599f) target_k_spd = 25.599f;
			else target_k_spd = target_k_spd_;
		}
		
    protected:
		void Set_Current(float target_current_) override {};// 电流模式不进行操作
		void Set_Angle(float target_angle_) override {};// 角度模式不进行操作
	
		void Can_Tx_Process() override;
		void Can_Rx_It_Process(uint32_t rx_id_, uint8_t *rx_data) override;
		void CanHandler_Register() override;
		
		void Tim_It_Process() override;
	
    private:
		pid::Pid pid_spd, pid_pos;
		float k_spd = 0, k_pos = 0;// 阻尼系数, 刚度系数
		float target_k_spd = 0, target_k_pos = 0;
	
		uint8_t air = 0;// 气压参数
		
		uint8_t id;// 电机id
		uint8_t module_id;// 模块id
		
		GoError error_code = GO_NORMAL;// 错误代号
		GoMotorStatus motor_status = GO_STATUS_FOC;
		GoControlMode control_mode = GO_CONTROL_MODE_1;

		bool torque_only = true;// 是否只使用力矩控制（速度环，位置环在stm32实现，只发送力矩）
    };
}
#endif
