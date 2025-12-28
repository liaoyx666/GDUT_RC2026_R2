#include "RC_chassis_jack.h"

namespace chassis_jack
{
<<<<<<< Updated upstream
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
=======
    Chassis_jack::Chassis_jack( 
		motor::Motor& left_front_motor_, 
		motor::Motor& left_behind_motor_, 
		motor::Motor& right_front_motor_, 
		motor::Motor& right_behind_motor_,
		lidar::LiDAR& LiDAR_jack_
	) : left_front_motor(left_front_motor_),
		left_behind_motor(left_behind_motor_),
		right_front_motor(right_front_motor_),
		right_behind_motor(right_behind_motor_),
		LiDAR_jack(LiDAR_jack_)
	{
		left_front_motor.Reset_Out_Pos(0);
		left_front_motor.Reset_Out_Angle(0);
		left_front_motor.pid_pos.Pid_Mode_Init(false, false, 0.01, true);
		left_front_motor.pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.001, 0, 8000, 4000, 2000, 2000, 2000, 5000, 7000);
		
		right_front_motor.Reset_Out_Pos(0);
		right_front_motor.Reset_Out_Angle(0);
		right_front_motor.pid_pos.Pid_Mode_Init(false, false, 0.01, true);
		right_front_motor.pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.001, 0, 8000, 4000, 2000, 2000, 2000, 5000, 7000);
		
		right_behind_motor.Reset_Out_Pos(0);
		right_behind_motor.Reset_Out_Angle(0);
		right_behind_motor.pid_pos.Pid_Mode_Init(false, false, 0.01, true);
		right_behind_motor.pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.001, 0, 8000 / ((10 * 3591.f / 187.f) / 99.506f), 4000, 2000, 2000, 2000, 5000 / ((10 * 3591.f / 187.f) / 99.506f), 7000 / ((10 * 3591.f / 187.f) / 99.506f));
		
		left_behind_motor.Reset_Out_Pos(0);
		left_behind_motor.Reset_Out_Angle(0);
		left_behind_motor.pid_pos.Pid_Mode_Init(false, false, 0.01, true);
		left_behind_motor.pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.001, 0, 8000 / ((10 * 3591.f / 187.f) / 99.506f), 4000, 2000, 2000, 2000, 5000 / ((10 * 3591.f / 187.f) / 99.506f), 7000 / ((10 * 3591.f / 187.f) / 99.506f));
	}
>>>>>>> Stashed changes

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
		
<<<<<<< Updated upstream
	void Chassis_jack::chassis_up_test()
    {
		b++;
=======
	void Chassis_jack::chassis_test(bool signal, bool state)
    {
		dis = LiDAR_jack.distance;

		if(b == 0 && bool state != last_state)
		{
			up_or_down = !up_or_down;
		}
		last_state = state;

>>>>>>> Stashed changes
		if(b > 4)
		{
			b = 4;
		}
<<<<<<< Updated upstream
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
=======

		if(up_or_down == 0)
		{
			switch (b)
			{
			case 0:					
					left_front_motor.Set_Out_Angle(0);
					left_behind_motor.Set_Out_Pos(0);
			
					right_front_motor.Set_Out_Angle(0);
					right_behind_motor.Set_Out_Pos(0);
					if(signal == true)
					{
						b++;
					}
					break;
			case 1:
					left_front_motor.Set_Out_Angle(256.f / 360 * TWO_PI);
					left_behind_motor.Set_Out_Pos(-104.f / 360 * TWO_PI);
			
					right_front_motor.Set_Out_Angle(104.f / 360 * TWO_PI);
					right_behind_motor.Set_Out_Pos(104.f / 360 * TWO_PI);

					if(dis < tag)
					{
						b++;
					}
					break;
			case 2:
					left_front_motor.Set_Out_Angle(PI);
					left_behind_motor.Set_Out_Pos(-PI);
			
					right_front_motor.Set_Out_Angle(PI);
					right_behind_motor.Set_Out_Pos(PI);

					if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == 0 && fabsf(left_behind_motor.Get_Out_Pos() - (-PI)) < 0.2f && fabsf(right_behind_motor.Get_Out_Pos() - (PI)) < 0.2f)
					{
						b++;
					}

					break;
			case 3:		
					left_front_motor.Set_Out_Angle(90.f / 360.f * TWO_PI);
					right_front_motor.Set_Out_Angle(270.f / 360.f * TWO_PI);

					if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == 0)
					{
						b++;
					}
					break;
			case 4:
					left_behind_motor.Set_Out_Pos(0);
					right_behind_motor.Set_Out_Pos(0);

					if(signal == true)
					{
						b = 0;
					}
					break;
			default :
					break;
			}		
		}
		else if(up_or_down == 1)
		{
			switch(b)
			{
				case 0:
					left_front_motor.Set_Out_Angle(0);
					left_behind_motor.Set_Out_Pos(0);
					right_front_motor.Set_Out_Angle(0);
					right_behind_motor.Set_Out_Pos(0);
					if(signal == true)
					{
						b++;
					}
					break;
				case 1:
					left_front_motor.Set_Out_Angle(104.f / 360 * TWO_PI);
					right_front_motor.Set_Out_Angle(256.f / 360 * TWO_PI);
					left_behind_motor.Set_Out_Pos(-104.f / 360 * TWO_PI);
					right_behind_motor.Set_Out_Pos(104.f / 360 * TWO_PI);	//水平外展
					break;
				case 2:
					left_behind_motor.Set_Out_Pos(-PI);
					right_behind_motor.Set_Out_Pos(PI);
				case 3:
					left_front_motor.Set_Out_Angle(PI);
					right_front_motor.Set_Out_Angle(PI);
					break;
				case 4:
					left_front_motor.Set_Out_Angle(256.f / 360 * TWO_PI);
					right_front_motor.Set_Out_Angle(104.f / 360 * TWO_PI);
					left_behind_motor.Set_Out_Pos(-104.f / 360 * TWO_PI);
					right_behind_motor.Set_Out_Pos(104.f / 360 * TWO_PI);
					break;
				case 5:
					left_front_motor.Set_Out_Angle(0);
					right_front_motor.Set_Out_Angle(0);
					left_behind_motor.Set_Out_Pos(0);
					right_behind_motor.Set_Out_Pos(0);
					if(signal == true)
					{
						b++;
					}
					break;
				default:
					break;
			}
		}

				/*
				//1
				left_front_motor.Set_Out_Angle(104.f / 360 * TWO_PI);
				right_front_motor.Set_Out_Angle(256.f / 360 * TWO_PI);
				left_behind_motor.Set_Out_Pos(-104.f / 360 * TWO_PI);
				right_behind_motor.Set_Out_Pos(104.f / 360 * TWO_PI);	//水平外展
				//2
				left_behind_motor.Set_Out_Pos(-PI);
				right_behind_motor.Set_Out_Pos(PI);
				//3
				left_front_motor.Set_Out_Angle(PI);
				right_front_motor.Set_Out_Angle(PI);
				//4
				left_front_motor.Set_Out_Angle(256.f / 360 * TWO_PI);
				right_front_motor.Set_Out_Angle(104.f / 360 * TWO_PI);
				left_behind_motor.Set_Out_Pos(-104.f / 360 * TWO_PI);
				right_behind_motor.Set_Out_Pos(104.f / 360 * TWO_PI);
				//5
				left_front_motor.Set_Out_Angle(0);
				right_front_motor.Set_Out_Angle(0);
				left_behind_motor.Set_Out_Pos(0);
				right_behind_motor.Set_Out_Pos(0);
				*/
    }
>>>>>>> Stashed changes
}