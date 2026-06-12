#pragma once
#include "RC_motor.h"
#include "RC_pid.h"
#include "RC_can.h"
#include "RC_tim.h"

#ifdef __cplusplus
namespace motor
{
	class DmMotor : public JointM, public can::CanHandler, public tim::TimHandler
	{
	public:
		DmMotor(
			uint8_t id_, can::Can &can_, tim::Tim *tim_, bool use_mit_, float k_spd_, float k_pos_, bool is_reset_pos_ ,
			float V_MAX_, float T_MAX_, float fc, float fs, float zeta
		);
		~DmMotor() = default;

		void Set_K_Pos(float target_k_pos_) override;
		void Set_K_Spd(float target_k_spd_) override;
		
	protected:
		void CanHandler_Register() override;
		void Can_Tx_Process() override;
		void Can_Rx_It_Process(uint32_t rx_id_, uint8_t *rx_data) override;
		void Tim_It_Process() override;

	private:
		static constexpr inline float P_MIN = -12.5f;// rad
		static constexpr inline float P_MAX = 12.5f;

		static constexpr inline float KP_MIN = 0.0f;
		static constexpr inline float KP_MAX = 500.0f;

		static constexpr inline float KD_MIN = 0.0f;
		static constexpr inline float KD_MAX = 5.0f;

		const float V_MIN;// rad/s
		const float V_MAX;
	
		const float T_MIN; // N * m
		const float T_MAX;
		
		uint16_t id;

		filter::SecondOrderLPF lp_td;
	
		uint8_t error_code;
		float mos_temperature;
		bool use_mit = false;
		bool is_enable = false;
	};
}
#endif
