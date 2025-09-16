#pragma once
#include "RC_motor.h"
#include "RC_can.h"
#include "RC_tim.h"
#include "RC_pid.h"

#ifdef __cplusplus

namespace m3508
{
	
	class M3508 : public motor::Motor, public can::CanHandler, public tim::TimHandler
	{
	public:
		M3508(uint8_t id_, can::Can &can_, tim::Tim &tim_);
		virtual ~M3508() {}
		
		pid::Pid pid_spd, pid_pos;
		
	protected:
		void CanHandler_Register() override;
		void Tim_It_Process() override;
		void Can_Tx_Process() override;
		void Can_Rx_It_Process(uint8_t *rx_data) override;
	
	
	
	private:
		uint8_t id;
		bool can_rx_is_first = true;
	};


}
#endif
