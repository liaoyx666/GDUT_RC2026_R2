#pragma once
#include "RC_dji_motor.h"

#ifdef __cplusplus
namespace motor
{
	class M3508 : public DjiMotor
	{
	public:
		M3508(uint8_t id_, can::Can &can_, tim::Tim &tim_, float gear_ratio_ = 3591.f / 187.f, bool is_reset_pos_angle = false);
		virtual ~M3508() {}
		
		void Set_Torque(float target_torque_) override;


	protected:
		void Dji_Id_Init(uint8_t id_) override;
		
	private:

	};
}
#endif
