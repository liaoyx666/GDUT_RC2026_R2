#include "RC_chassis_jack.h"

namespace chassis_jack
{
    Chassis_jack::Chassis_jack( motor::Motor& left_front_motor_, 
                                motor::Motor& left_behind_motor_, 
                                motor::Motor& right_front_motor_, 
                                motor::Motor& right_behind_motor_)
							  : left_front_motor(left_front_motor_),
							    left_behind_motor(left_behind_motor_),
								right_front_motor(right_front_motor_),
								right_behind_motor(right_behind_motor_)
                                {}

	void Chassis_jack::chassis_up()
    {
        left_front_motor.Set_Out_Pos(0);
        left_behind_motor.Set_Out_Pos(0);
        right_front_motor.Set_Out_Pos(0);
        right_behind_motor.Set_Out_Pos(0);
    }

    void Chassis_jack::chassis_down()
    {
        left_front_motor.Set_Out_Pos(0);
        left_behind_motor.Set_Out_Pos(0);
        right_front_motor.Set_Out_Pos(0);
        right_behind_motor.Set_Out_Pos(0);
    }
		
	void Chassis_jack::chassis_test()
    {
		b++;
		
		if(b > 4)
		{
			b = 0;
		}
		
		switch (b)
		{
		case 0:					
				left_front_motor.Set_Out_Angle(0);
				left_behind_motor.Set_Out_Pos(0);
		
				right_front_motor.Set_Out_Angle(0);
				right_behind_motor.Set_Out_Pos(0);
				break;
		case 1:
				left_front_motor.Set_Out_Angle(256.f / 360 * TWO_PI);
				left_behind_motor.Set_Out_Pos(-104.f / 360 * TWO_PI);
		
				right_front_motor.Set_Out_Angle(104.f / 360 * TWO_PI);
				right_behind_motor.Set_Out_Pos(104.f / 360 * TWO_PI);
				break;
		case 2:
				left_front_motor.Set_Out_Angle(PI);
				left_behind_motor.Set_Out_Pos(-PI);
		
				right_front_motor.Set_Out_Angle(PI);
				right_behind_motor.Set_Out_Pos(PI);
				break;
		case 3:		
				left_front_motor.Set_Out_Angle(90.f / 360.f * TWO_PI);
				right_front_motor.Set_Out_Angle(270.f / 360.f * TWO_PI);
				break;
		case 4:
				left_behind_motor.Set_Out_Pos(0);
				right_behind_motor.Set_Out_Pos(0);
				break;
		default :
				break;
		}
    }
}