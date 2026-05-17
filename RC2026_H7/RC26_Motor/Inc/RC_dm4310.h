#pragma once
#include "RC_dm_motor.h"

#ifdef __cplusplus

namespace motor
{
	class DM4310 : public DmMotor
	{
	public:
		DM4310(uint8_t id_, can::Can &can_, tim::Tim *tim_, bool use_mit_ = false, float k_spd_ = 0, float k_pos_ = 0, bool is_reset_pos_ = true);
		~DM4310() = default;
	};
}
#endif
