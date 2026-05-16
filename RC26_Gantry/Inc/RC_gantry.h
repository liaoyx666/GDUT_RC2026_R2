#pragma once
#include "RC_motor.h"
#include "RC_task.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "RC_serial.h"

#ifdef __cplusplus
namespace gantry
{
	
	constexpr float GANTRY_X_M_TO_RAD = (float)(0.5 * TWO_PI / (26.0 * 0.003));
	constexpr float GANTRY_Y_M_TO_RAD = (float)(TWO_PI / (20.0 * 1.5 * PI * 0.001));
	constexpr float GANTRY_Z_M_TO_RAD = (float)(0.5 * TWO_PI / (20.0 * 0.005));
	
	constexpr float GANTRY_X_RAD_TO_M = 1.f / GANTRY_X_M_TO_RAD;
	constexpr float GANTRY_Y_RAD_TO_M = 1.f / GANTRY_Y_M_TO_RAD;
	constexpr float GANTRY_Z_RAD_TO_M = 1.f / GANTRY_Z_M_TO_RAD;
	
	constexpr float GANTRY_Y_OFFSET = 0.1;
	
	class Gantry// : public task::ManagedTask
    {
    public:
		Gantry(
			motor::Motor& m_x_,
			motor::Motor& m_y_,
			motor::Motor& m_z_,
			motor::JointM& m_p_
		);
	
		~Gantry() = default;
		
		// 设置位置
		void Set_X(float m_);
		void Set_Y(float m_);
		void Set_Z(float m_);
		void Set_P(float rad_);
		
		// 获取位置
		constexpr float Get_X() { return  motor_x.Get_Out_Pos() * GANTRY_X_RAD_TO_M; }
		constexpr float Get_Y() { return  motor_y.Get_Out_Pos() * GANTRY_Y_RAD_TO_M - GANTRY_Y_OFFSET; }
		constexpr float Get_Z() { return  motor_z.Get_Out_Pos() * GANTRY_Z_RAD_TO_M; }
		constexpr float Get_P() { return -motor_p.Get_Out_Pos(); }

		// 设置加速度，速度
		constexpr void Set_X_Td(float a, float v) { motor_x.pid_pos.Set_Td(a, v); }
		constexpr void Set_Y_Td(float a, float v) { motor_y.pid_pos.Set_Td(a, v); }
		constexpr void Set_Z_Td(float a, float v) { motor_z.pid_pos.Set_Td(a, v); }
		constexpr void Set_P_Td(float a, float v) { motor_p.pid_pos.Set_Td(a, v); }
		
		// 设置默认加速度，速度
		constexpr void Set_Defualt_Td()
		{
			Set_X_Td(2000.f, 8000.f);
			Set_Y_Td(2000.f, 8000.f);
			Set_Z_Td(1000.f, 2000.f);
			Set_P_Td(20.f  , 7.f   );
		}
		
		// 回到默认位置
		constexpr void Set_Reset_Pos()
		{
			Set_X(0.03);
			Set_Y(0);
			Set_Z(0);
			Set_P(0);
		}
		
		void Gantry_Base();
		
    private:
		SemaphoreHandle_t mutex;
		uint8_t user_id; /*0代表空闲*/
		
		float target_x;
		float target_y;
		float target_z;
		float target_p;

		float p_max = 0;
		float p_min = 0;
		
		motor::Motor& motor_x;
		motor::Motor& motor_y;
		motor::Motor& motor_z;
		motor::JointM& motor_p;// pitch
	
		friend class GantryUser;
    };
	

	class GantryUser
    {
    public:
		GantryUser(Gantry& gan_); /*id不能为0*/
		~GantryUser() = default;
		
		constexpr void Set_X(float m_)
		{
			if (gan.user_id == id)
				gan.Set_X(m_);
		}
		
	    constexpr void Set_Y(float m_)
		{
			if (gan.user_id == id)
				gan.Set_Y(m_);
		}
		
	    constexpr void Set_Z(float m_)
		{
			if (gan.user_id == id)
				gan.Set_Z(m_);
		}
		
	    constexpr void Set_P(float rad_)
		{
			if (gan.user_id == id)
				gan.Set_P(rad_);
		}
		
		constexpr void Set_Defualt_Td()
		{
			if (gan.user_id == id)
				gan.Set_Defualt_Td();
		}
		
		constexpr void Set_Reset_Pos()
		{
			if (gan.user_id == id)
				gan.Set_Reset_Pos();
		}
		
		constexpr void Set_X_Td(float a, float v)
		{ 
			if (gan.user_id == id)
				gan.Set_X_Td(a, v); 
		}
		
		constexpr void Set_Y_Td(float a, float v)
		{ 
			if (gan.user_id == id)
				gan.Set_Y_Td(a, v); 
		}
		
		constexpr void Set_Z_Td(float a, float v)
		{ 
			if (gan.user_id == id)
				gan.Set_Z_Td(a, v); 
		}
		
		constexpr void Set_P_Td(float a, float v)
		{ 
			if (gan.user_id == id)
				gan.Set_P_Td(a, v); 
		}
		
		bool Take_Control();
		void Give_Control();
    private:
		uint8_t id;
		Gantry& gan;
		
		static uint8_t user_num;
    };
}
#endif