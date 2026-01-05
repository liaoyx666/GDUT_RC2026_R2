#include "RC_chassis_jack.h"

namespace chassis_jack
{
    Chassis_jack::Chassis_jack( 
		motor::Motor& left_front_motor_, 
		motor::Motor& left_behind_motor_, 
		motor::Motor& right_front_motor_, 
		motor::Motor& right_behind_motor_,
		motor::Motor& left_small_wheel_,
		motor::Motor& right_small_wheel_,
		float max_linear_vel_,
		lidar::LiDAR& LiDAR_jack_,
		chassis::Chassis& v_limit_
	) : left_front_motor(left_front_motor_),
		left_behind_motor(left_behind_motor_),
		right_front_motor(right_front_motor_),
		right_behind_motor(right_behind_motor_),
		left_small_wheel(left_small_wheel_),
		right_small_wheel(right_small_wheel_),
		LiDAR_jack(LiDAR_jack_),
		v_limit(v_limit_)
	{
		
		left_front_motor.pid_pos.Pid_Mode_Init(false, false, 0.01, true);
		left_front_motor.pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.001, 0, 8000, 4000, 2000, 2000, 2000, 1000, 7000);	
		
		right_front_motor.pid_pos.Pid_Mode_Init(false, false, 0.01, true);
		right_front_motor.pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.001, 0, 8000, 4000, 2000, 2000, 2000, 1000, 7000);
		
		right_behind_motor.pid_pos.Pid_Mode_Init(false, false, 0.01, true);
		right_behind_motor.pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.001, 0, 8000 / ((10 * 3591.f / 187.f) / 99.506f), 4000, 2000, 2000, 2000, 1000 / ((10 * 3591.f / 187.f) / 99.506f), 7000 / ((10 * 3591.f / 187.f) / 99.506f));
		
		left_behind_motor.pid_pos.Pid_Mode_Init(false, false, 0.01, true);
		left_behind_motor.pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.001, 0, 8000 / ((10 * 3591.f / 187.f) / 99.506f), 4000, 2000, 2000, 2000, 1000 / ((10 * 3591.f / 187.f) / 99.506f), 7000 / ((10 * 3591.f / 187.f) / 99.506f));
		
		max_linear_vel = fabsf(max_linear_vel_);
	}

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
		
	void Chassis_jack::chassis_test(bool signal, bool state, float default_vel,    GPIO_TypeDef* GPIOx1, uint16_t GPIO_Pin_1,
												   			 float up_ready_vel,   GPIO_TypeDef* GPIOx2, uint16_t GPIO_Pin_2,
												   			 float up_close_vel,   GPIO_TypeDef* GPIOx3, uint16_t GPIO_Pin_3,
												   			 float down_close_vel, GPIO_TypeDef* GPIOx4, uint16_t GPIO_Pin_4)
    {
		dis = LiDAR_jack.distance;
		
		gd1 = HAL_GPIO_ReadPin(GPIOx1, GPIO_Pin_1);	//a8 上楼梯收前腿
		gd2 = HAL_GPIO_ReadPin(GPIOx2, GPIO_Pin_2);	//下楼梯伸前腿
		gd3 = HAL_GPIO_ReadPin(GPIOx3, GPIO_Pin_3);	//上楼梯收后腿
		gd4 = HAL_GPIO_ReadPin(GPIOx4, GPIO_Pin_4);	//下楼梯伸后腿

		if(b == 0 && state != last_state)
		{
			up_or_down = !up_or_down;
		}
		last_state = state;

		if(b > 4)
		{
			b = 0;
		}

		if(up_or_down == 0)
		{
			switch (b)
			{
			case 0:	
					left_front_motor.pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.001, 0, 8000, 4000, 2000, 2000, 2000, 5000, 7000);	
					right_front_motor.pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.001, 0, 8000, 4000, 2000, 2000, 2000, 5000, 7000);
					right_behind_motor.pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.001, 0, 8000 / ((10 * 3591.f / 187.f) / 99.506f), 4000, 2000, 2000, 2000, 5000 / ((10 * 3591.f / 187.f) / 99.506f), 7000 / ((10 * 3591.f / 187.f) / 99.506f));
					left_behind_motor.pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.001, 0, 8000 / ((10 * 3591.f / 187.f) / 99.506f), 4000, 2000, 2000, 2000, 5000 / ((10 * 3591.f / 187.f) / 99.506f), 7000 / ((10 * 3591.f / 187.f) / 99.506f));
					
					v_limit.Set_Max_Linear_Vel(default_vel);
					
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
					left_front_motor.pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.001, 0, 8000, 4000, 2000, 2000, 2000, 5000, 7000);	
					right_front_motor.pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.001, 0, 8000, 4000, 2000, 2000, 2000, 5000, 7000);
					right_behind_motor.pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.001, 0, 8000 / ((10 * 3591.f / 187.f) / 99.506f), 4000, 2000, 2000, 2000, 5000 / ((10 * 3591.f / 187.f) / 99.506f), 7000 / ((10 * 3591.f / 187.f) / 99.506f));
					left_behind_motor.pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.001, 0, 8000 / ((10 * 3591.f / 187.f) / 99.506f), 4000, 2000, 2000, 2000, 5000 / ((10 * 3591.f / 187.f) / 99.506f), 7000 / ((10 * 3591.f / 187.f) / 99.506f));

					v_limit.Set_Max_Linear_Vel(up_ready_vel);
			
					left_front_motor.Set_Out_Angle(256.f / 360 * TWO_PI);
					left_behind_motor.Set_Out_Pos(-102.f / 360 * TWO_PI);
			
					right_front_motor.Set_Out_Angle(104.f / 360 * TWO_PI);
					right_behind_motor.Set_Out_Pos(102.f / 360 * TWO_PI);

					if(dis < tag)
					{
						b++;
					}
					break;
			case 2:
					left_front_motor.pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.001, 0, 8000, 4000, 2000, 2000, 2000, 2000, 7000);	
					right_front_motor.pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.001, 0, 8000, 4000, 2000, 2000, 2000, 2000, 7000);
					right_behind_motor.pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.001, 0, 8000 / ((10 * 3591.f / 187.f) / 99.506f), 4000, 2000, 2000, 2000, 2000 / ((10 * 3591.f / 187.f) / 99.506f), 7000 / ((10 * 3591.f / 187.f) / 99.506f));
					left_behind_motor.pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.001, 0, 8000 / ((10 * 3591.f / 187.f) / 99.506f), 4000, 2000, 2000, 2000, 2000 / ((10 * 3591.f / 187.f) / 99.506f), 7000 / ((10 * 3591.f / 187.f) / 99.506f));
					
					v_limit.Set_Max_Linear_Vel(up_close_vel);
					v_limit.Set_Linear_Accel(1.5);
			
					left_front_motor.Set_Out_Angle(PI);
					left_behind_motor.Set_Out_Pos(-PI);
			
					right_front_motor.Set_Out_Angle(PI);
					right_behind_motor.Set_Out_Pos(PI);

					if(gd1 == 0 && fabsf(left_behind_motor.Get_Out_Pos() - (-PI)) < 0.2f && fabsf(right_behind_motor.Get_Out_Pos() - (PI)) < 0.2f)
					{
						b++;
					}
					break;
			case 3:	
					left_front_motor.pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.001, 0, 8000, 4000, 2000, 2000, 2000, 5000, 7000);	
					right_front_motor.pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.001, 0, 8000, 4000, 2000, 2000, 2000, 5000, 7000);
					right_behind_motor.pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.001, 0, 8000 / ((10 * 3591.f / 187.f) / 99.506f), 4000, 2000, 2000, 2000, 5000 / ((10 * 3591.f / 187.f) / 99.506f), 7000 / ((10 * 3591.f / 187.f) / 99.506f));
					left_behind_motor.pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.001, 0, 8000 / ((10 * 3591.f / 187.f) / 99.506f), 4000, 2000, 2000, 2000, 5000 / ((10 * 3591.f / 187.f) / 99.506f), 7000 / ((10 * 3591.f / 187.f) / 99.506f));
					
					v_limit.Set_Max_Linear_Vel(up_close_vel);
					v_limit.Set_Linear_Accel(5);
			
					left_front_motor.Set_Out_Angle(90.f / 360.f * TWO_PI);
					right_front_motor.Set_Out_Angle(270.f / 360.f * TWO_PI);
		
					if(gd2 == 0)
					{
						b++;
					}
					break;
			case 4:
					left_front_motor.pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.001, 0, 8000, 4000, 2000, 2000, 2000, 5000, 7000);	
					right_front_motor.pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.001, 0, 8000, 4000, 2000, 2000, 2000, 5000, 7000);
					right_behind_motor.pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.001, 0, 8000 / ((10 * 3591.f / 187.f) / 99.506f), 4000, 2000, 2000, 2000, 5000 / ((10 * 3591.f / 187.f) / 99.506f), 7000 / ((10 * 3591.f / 187.f) / 99.506f));
					left_behind_motor.pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.001, 0, 8000 / ((10 * 3591.f / 187.f) / 99.506f), 4000, 2000, 2000, 2000, 5000 / ((10 * 3591.f / 187.f) / 99.506f), 7000 / ((10 * 3591.f / 187.f) / 99.506f));
					
					v_limit.Set_Max_Linear_Vel(up_close_vel);
					
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
					left_front_motor.pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.001, 0, 8000, 4000, 2000, 2000, 2000, 5000, 7000);	
					right_front_motor.pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.001, 0, 8000, 4000, 2000, 2000, 2000, 5000, 7000);
					right_behind_motor.pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.001, 0, 8000 / ((10 * 3591.f / 187.f) / 99.506f), 4000, 2000, 2000, 2000, 5000 / ((10 * 3591.f / 187.f) / 99.506f), 7000 / ((10 * 3591.f / 187.f) / 99.506f));
					left_behind_motor.pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.001, 0, 8000 / ((10 * 3591.f / 187.f) / 99.506f), 4000, 2000, 2000, 2000, 5000 / ((10 * 3591.f / 187.f) / 99.506f), 7000 / ((10 * 3591.f / 187.f) / 99.506f));
					
					v_limit.Set_Max_Linear_Vel(default_vel);
				
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
					left_front_motor.pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.001, 0, 8000, 4000, 2000, 2000, 2000, 5000, 7000);	
					right_front_motor.pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.001, 0, 8000, 4000, 2000, 2000, 2000, 5000, 7000);
					right_behind_motor.pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.001, 0, 8000 / ((10 * 3591.f / 187.f) / 99.506f), 4000, 2000, 2000, 2000, 5000 / ((10 * 3591.f / 187.f) / 99.506f), 7000 / ((10 * 3591.f / 187.f) / 99.506f));
					left_behind_motor.pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.001, 0, 8000 / ((10 * 3591.f / 187.f) / 99.506f), 4000, 2000, 2000, 2000, 5000 / ((10 * 3591.f / 187.f) / 99.506f), 7000 / ((10 * 3591.f / 187.f) / 99.506f));
					
					v_limit.Set_Max_Linear_Vel(down_close_vel);
				
					left_front_motor.Set_Out_Angle(90.f / 360 * TWO_PI);
					right_front_motor.Set_Out_Angle(270.f / 360 * TWO_PI);
					left_behind_motor.Set_Out_Pos(-90.f / 360 * TWO_PI);
					right_behind_motor.Set_Out_Pos(90.f / 360 * TWO_PI);	//水平外展
					if(gd2 == 1)
					{
						b++;
					}
					break;
				case 2:
					left_front_motor.pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.001, 0, 8000, 4000, 2000, 2000, 2000, 5000, 7000);	
					right_front_motor.pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.001, 0, 8000, 4000, 2000, 2000, 2000, 5000, 7000);
					right_behind_motor.pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.001, 0, 8000 / ((10 * 3591.f / 187.f) / 99.506f), 4000, 2000, 2000, 2000, 5000 / ((10 * 3591.f / 187.f) / 99.506f), 7000 / ((10 * 3591.f / 187.f) / 99.506f));
					left_behind_motor.pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.001, 0, 8000 / ((10 * 3591.f / 187.f) / 99.506f), 4000, 2000, 2000, 2000, 5000 / ((10 * 3591.f / 187.f) / 99.506f), 7000 / ((10 * 3591.f / 187.f) / 99.506f));
					
					v_limit.Set_Max_Linear_Vel(down_close_vel);
				
					left_behind_motor.Set_Out_Pos(-PI);
					right_behind_motor.Set_Out_Pos(PI);
					if(gd3 == 1)
					{
						b++;
					}
					break;
				case 3:
					left_front_motor.pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.001, 0, 8000, 4000, 2000, 2000, 2000, 5000, 7000);	
					right_front_motor.pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.001, 0, 8000, 4000, 2000, 2000, 2000, 5000, 7000);
					right_behind_motor.pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.001, 0, 8000 / ((10 * 3591.f / 187.f) / 99.506f), 4000, 2000, 2000, 2000, 5000 / ((10 * 3591.f / 187.f) / 99.506f), 7000 / ((10 * 3591.f / 187.f) / 99.506f));
					left_behind_motor.pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.001, 0, 8000 / ((10 * 3591.f / 187.f) / 99.506f), 4000, 2000, 2000, 2000, 5000 / ((10 * 3591.f / 187.f) / 99.506f), 7000 / ((10 * 3591.f / 187.f) / 99.506f));
					
					v_limit.Set_Max_Linear_Vel(down_close_vel);
				
					left_front_motor.Set_Out_Angle(PI);
					right_front_motor.Set_Out_Angle(PI);
					if(signal == true)
					{
						b++;
					}
					break;
				case 4:
					left_front_motor.pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.001, 0, 8000, 4000, 2000, 2000, 2000, 300, 7000);	
					right_front_motor.pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.001, 0, 8000, 4000, 2000, 2000, 2000, 300, 7000);
					right_behind_motor.pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.001, 0, 8000 / ((10 * 3591.f / 187.f) / 99.506f), 4000, 2000, 2000, 2000, 300 / ((10 * 3591.f / 187.f) / 99.506f), 7000 / ((10 * 3591.f / 187.f) / 99.506f));
					left_behind_motor.pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.001, 0, 8000 / ((10 * 3591.f / 187.f) / 99.506f), 4000, 2000, 2000, 2000, 300 / ((10 * 3591.f / 187.f) / 99.506f), 7000 / ((10 * 3591.f / 187.f) / 99.506f));
					
					v_limit.Set_Max_Linear_Vel(down_close_vel);
				
					left_front_motor.Set_Out_Angle(256.f / 360 * TWO_PI);
					right_front_motor.Set_Out_Angle(104.f / 360 * TWO_PI);
					left_behind_motor.Set_Out_Pos(-100.f / 360 * TWO_PI);
					right_behind_motor.Set_Out_Pos(100.f / 360 * TWO_PI);
					if(signal == true)
					{
						b++;
					}
					break;
				case 5:
					left_front_motor.pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.001, 0, 8000, 4000, 2000, 2000, 2000, 5000, 7000);	
					right_front_motor.pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.001, 0, 8000, 4000, 2000, 2000, 2000, 5000, 7000);
					right_behind_motor.pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.001, 0, 8000 / ((10 * 3591.f / 187.f) / 99.506f), 4000, 2000, 2000, 2000, 5000 / ((10 * 3591.f / 187.f) / 99.506f), 7000 / ((10 * 3591.f / 187.f) / 99.506f));
					left_behind_motor.pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.001, 0, 8000 / ((10 * 3591.f / 187.f) / 99.506f), 4000, 2000, 2000, 2000, 5000 / ((10 * 3591.f / 187.f) / 99.506f), 7000 / ((10 * 3591.f / 187.f) / 99.506f));
					
					v_limit.Set_Max_Linear_Vel(default_vel);
				
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
    }
	
	
	void Chassis_jack::Set_Vel(float linear_vel_)
	{
		linear_vel_ = (linear_vel_ > max_linear_vel ? max_linear_vel : linear_vel_);

		if (left_behind_motor.Get_Out_Pos() < (-90.f / 360 * TWO_PI) && right_behind_motor.Get_Out_Pos() > (90.f / 360 * TWO_PI))
		{
			float jack_vel = (left_behind_motor.Get_Out_Rpm() - right_behind_motor.Get_Out_Rpm()) / 2.f * rpm_to_vel;
			
			linear_vel_ -= jack_vel;
		}
		else
		{
			linear_vel_ = 0;
		}
		
		left_small_wheel.Set_Out_Rpm(-linear_vel_ / SMALL_WHEEL_RADIUS * (60.f / TWO_PI));
		right_small_wheel.Set_Out_Rpm(linear_vel_ / SMALL_WHEEL_RADIUS * (60.f / TWO_PI));
	}
	
	
	
	
}