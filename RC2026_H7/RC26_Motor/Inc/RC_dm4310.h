#pragma once
#include "RC_motor.h"
#include "RC_pid.h"
#include "RC_can.h"
#include "RC_tim.h"

#ifdef __cplusplus

namespace motor
{
	class DM4310 : public Motor, public can::CanHandler, public tim::TimHandler
	{
	public:
		DM4310(uint8_t id_, can::Can &can_, tim::Tim &tim_, bool use_mit_ = false, float k_spd_ = 0, float k_pos_ = 0);
		virtual ~DM4310() {}
		
		uint8_t error_code;
		
		pid::Pid pid_spd, pid_pos;
	protected:
	
		void CanHandler_Register() override;
		void Can_Tx_Process() override;
		void Can_Rx_It_Process(uint32_t rx_id_, uint8_t *rx_data) override;
		void Tim_It_Process() override;
		
	private:
		uint16_t id;
		bool can_rx_is_first = true;
	};
}
#endif
