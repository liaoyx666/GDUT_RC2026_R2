#pragma once
#include "RC_motor.h"
#include "RC_pid.h"
#include "RC_can.h"
#include "RC_tim.h"
#include "RC_adrc.h"
#include "RC_SMC.h"

#include <math.h>

#ifdef __cplusplus
namespace motor
{
	class DjiMotor : public Motor, public can::CanHandler, public tim::TimHandler
    {
    public:
		DjiMotor(can::Can &can_, tim::Tim &tim_, float gear_ratio_ = 1.f);
		virtual ~DjiMotor() {}
		
		pid::Pid pid_spd, pid_pos;
		//adrc::FirstADRC spd_adrc;
		//smc::SMC spd_smc;

    protected:
		virtual void Dji_Id_Init(uint8_t id_) = 0;// 初始化发送和接受帧的id
		
		void Set_Torque(float target_torque_) override {};// 不进行操作
		
		void CanHandler_Register() override;
		void Tim_It_Process() override;
		void Can_Tx_Process() override;
		void Can_Rx_It_Process(uint32_t rx_id_, uint8_t *rx_data) override;

		bool can_rx_is_first = true;
		
		uint8_t id;
		
    private:
		
    };
}
#endif