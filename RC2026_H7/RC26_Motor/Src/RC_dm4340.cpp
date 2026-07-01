#include "RC_dm4340.h"

namespace motor
{
	DM4340::DM4340(uint8_t id_, can::Can &can_, tim::Tim *tim_, bool use_mit_, float k_spd_, float k_pos_, bool is_reset_pos_)
		: DmMotor(
			id_, can_, tim_, use_mit_, k_spd_, k_pos_, is_reset_pos_,
			10.0f, 28.0f, 130, 1000, 0.707
		)
	{
		// dm4310默认参数
		pid_spd.Pid_Mode_Init(true, false, 0.01);
		pid_spd.Pid_Param_Init(0, 0, 0, 0, 0.001, 0, 10, 5, 5, 5, 5);
		
		pid_pos.Pid_Mode_Init(false, false, 0.01, true);
		pid_pos.Pid_Param_Init(0, 0, 0, 0, 0.001, 0, 20, 5, 5 ,5 ,5, 10, 15);
	}
}
	