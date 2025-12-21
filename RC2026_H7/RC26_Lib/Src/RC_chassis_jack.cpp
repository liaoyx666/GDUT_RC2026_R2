#include "RC_chassis_jack.h"

namespace chassis_jack
{
    Chassis_jack::Chassis_jack( motor::Motor& left_front_motor_, 
                                motor::Motor& left_behind_motor_, 
                                motor::Motor& right_front_motor_, 
                                motor::Motor& right_behind_motor_,
								liDAR::LiDAR& LiDAR_jack_)
							  : left_front_motor(left_front_motor_),
							    left_behind_motor(left_behind_motor_),
								right_front_motor(right_front_motor_),
								right_behind_motor(right_behind_motor_),
								LiDAR_jack(LiDAR_jack_)
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
		
	void Chassis_jack::chassis_up_test()
    {
		b++;
		if(b > 4)
		{
			b = 4;
		}
		switch (b)
		{
		case 0:					
			left_front_motor.Set_Out_Pos(0);  //初始全部垂直向上
			left_behind_motor.Set_Out_Pos(0);
			right_front_motor.Set_Out_Pos(0);
			right_behind_motor.Set_Out_Pos(0);
			break;
		case 1:
			left_front_motor.Set_Out_Pos(-104.f / 360 * TWO_PI);  //向外紧贴地面
			left_behind_motor.Set_Out_Pos(-104.f / 360 * TWO_PI);
			right_front_motor.Set_Out_Pos(104.f / 360 * TWO_PI);
			right_behind_motor.Set_Out_Pos(104.f / 360 * TWO_PI);
			break;
		case 2:
			left_front_motor.Set_Out_Pos(-PI);		//垂直向下
			left_behind_motor.Set_Out_Pos(-PI);
			right_front_motor.Set_Out_Pos(PI);
			right_behind_motor.Set_Out_Pos(PI);
			break;
		case 3:		
			left_front_motor.Set_Out_Pos(-270.f / 360.f * TWO_PI);		//向内收前腿水平
			right_front_motor.Set_Out_Pos(270.f / 360.f * TWO_PI);
			break;
		case 4:
			left_behind_motor.Set_Out_Pos(-0);	//向内收后腿水平
			right_behind_motor.Set_Out_Pos(0);
			break;
		default :
			break;
		}
	}
		
	void Chassis_jack::chassis_down_test()
	{
		b++;
		if(b > 4)
		{
			b = 4;
		}
		switch (b)
		{
		case 0:					
			left_front_motor.Set_Out_Pos(0);  //初始全部垂直向上
			left_behind_motor.Set_Out_Pos(0);
			right_front_motor.Set_Out_Pos(0);
			right_behind_motor.Set_Out_Pos(0);
			break;
		case 1:
			left_front_motor.Set_Out_Pos(-PI);  //向外紧贴地面
			left_behind_motor.Set_Out_Pos(-PI);
			right_front_motor.Set_Out_Pos(104.f / 360 * TWO_PI);
			right_behind_motor.Set_Out_Pos(104.f / 360 * TWO_PI);
			break;
		case 2:
			left_front_motor.Set_Out_Pos(-PI);		//垂直向下
			left_behind_motor.Set_Out_Pos(-PI);
			right_front_motor.Set_Out_Pos(PI);
			right_behind_motor.Set_Out_Pos(PI);
			break;
		case 3:		
			left_front_motor.Set_Out_Pos(-270.f / 360.f * TWO_PI);		//向内收前腿水平
			right_front_motor.Set_Out_Pos(270.f / 360.f * TWO_PI);
			break;
		case 4:
			left_behind_motor.Set_Out_Pos(-0);	//向内收后腿水平
			right_behind_motor.Set_Out_Pos(0);
			break;
		default :
			break;
		}
	}
/*
		if(LiDAR_jack.distance <= 20)
		{
			left_front_motor.Set_Out_Pos(-PI);		//垂直向下
			left_behind_motor.Set_Out_Pos(-PI);
			right_front_motor.Set_Out_Pos(PI);
			right_behind_motor.Set_Out_Pos(PI);
		}
				
		if(HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_3) == 0)
		{
			left_front_motor.Set_Out_Pos(-270.f / 360.f * TWO_PI);		//向内收前腿水平
			right_front_motor.Set_Out_Pos(270.f / 360.f * TWO_PI);
		}
				
		if(HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_a4) == 0)
		{
			left_behind_motor.Set_Out_Pos(-0);	//向内收后腿水平
			right_behind_motor.Set_Out_Pos(0);
		}
*/
}