#pragma once

#include "RC_motor.h"

#ifdef __cplusplus

namespace chassis_jack
{
    class Chassis_jack
    {
	public:
		Chassis_jack(
			motor::Motor& left_front_motor_,
			motor::Motor& left_behind_motor_,
			motor::Motor& right_front_motor_, 
			motor::Motor& right_behind_motor_
		);

		virtual ~Chassis_jack() {}

		void chassis_up();
		void chassis_down();
		void chassis_test();
			
		uint8_t b = 0;

	private:
		motor::Motor& left_front_motor;
		motor::Motor& left_behind_motor;
		motor::Motor& right_front_motor;
		motor::Motor& right_behind_motor;
    };
}



#endif
