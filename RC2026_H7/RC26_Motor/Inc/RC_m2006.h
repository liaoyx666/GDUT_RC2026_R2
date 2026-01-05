#pragma once
#include "RC_dji_motor.h"

#ifdef __cplusplus
namespace motor
{
	class M2006 : public DjiMotor
	{
	public:
		M2006(uint8_t id_, can::Can &can_, tim::Tim &tim_, float gear_ratio_ = 36.f);
		virtual ~M2006() {}

	protected:
		void Dji_Id_Init(uint8_t id_) override;
		
	private:

	};
}
#endif
