#pragma once

#include "RC_motor.h"
#include "RC_LiDAR.h"
#include "RC_chassis.h"

#ifdef __cplusplus

#define SMALL_WHEEL_RADIUS	0.03f
#define JACK_LANGHT 0.260f



namespace chassis_jack
{
    class Chassis_jack
    {
	public:
		Chassis_jack(
			motor::Motor& left_front_motor_, 
			motor::Motor& left_behind_motor_, 
			motor::Motor& right_front_motor_, 
			motor::Motor& right_behind_motor_,
			motor::Motor& left_small_wheel_,
			motor::Motor& right_small_wheel_,
			float max_linear_vel_,
			lidar::LiDAR& LiDAR_jack_,
			chassis::Chassis& v_limit_
		);

		virtual ~Chassis_jack() {}


		void chassis_test(bool signal, uint8_t state, float default_vel,    GPIO_TypeDef* GPIOx1, uint16_t GPIO_Pin_1,
												   float up_ready_vel,   GPIO_TypeDef* GPIOx2, uint16_t GPIO_Pin_2,
												   float up_close_vel,   GPIO_TypeDef* GPIOx3, uint16_t GPIO_Pin_3,
												   float down_close_vel, GPIO_TypeDef* GPIOx4, uint16_t GPIO_Pin_4);
		
		chassis::Chassis& v_limit;
		uint8_t b = 0;
		float tag = 390;
		float dis = 0;
			
		bool gd1 = 0;
		bool gd2 = 0;
		bool gd3 = 0;
		bool gd4 = 0;
			
		bool up_or_down = 0;
		
		bool last_state = 0;
		/*-------------------------------------------*/
			
		void Set_Vel(float linear_vel_);
		
	private:
		motor::Motor& left_front_motor;
		motor::Motor& left_behind_motor;
		motor::Motor& right_front_motor;
		motor::Motor& right_behind_motor;
	
		lidar::LiDAR& LiDAR_jack;
		/*-------------------------------------------*/
		
		float max_linear_vel;
		
		const float rpm_to_vel = (JACK_LANGHT) * ((2.0f * PI) / 60.0f);
		
		motor::Motor& left_small_wheel;
		motor::Motor& right_small_wheel;
		
		uint32_t last_time = 0;
    };
}



#endif
