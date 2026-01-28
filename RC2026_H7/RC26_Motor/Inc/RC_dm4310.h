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
		DM4310(uint8_t id_, can::Can &can_, tim::Tim &tim_, bool use_mit_ = false, float k_spd_ = 0, float k_pos_ = 0, bool is_reset_pos_ = false);
		virtual ~DM4310() {}

		void Set_K_Pos(float target_k_pos_) override;
		void Set_K_Spd(float target_k_spd_) override;
		
	protected:
		void CanHandler_Register() override;
		void Can_Tx_Process() override;
		void Can_Rx_It_Process(uint32_t rx_id_, uint8_t *rx_data) override;
		void Tim_It_Process() override;
		void Set_Current(float target_current_) override {} 

	private:
		uint16_t id;

		uint8_t error_code;
		float mos_temperature;
		bool use_mit = false;
		bool is_enable = false;

	};
}
#endif
