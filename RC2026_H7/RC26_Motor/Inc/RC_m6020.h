#pragma once
#include "RC_dji_motor.h"

#ifdef __cplusplus
namespace motor
{
	class M6020 : public DjiMotor
	{
	public:
		M6020(uint8_t id_, can::Can &can_, tim::Tim &tim_);
		virtual ~M6020() {}

	protected:
		void Dji_Id_Init(uint8_t id_) override;
		
	private:

	};
}
#endif
