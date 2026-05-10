#pragma once
#include "RC_motor.h"
#include "RC_task.h"

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
	
	
	class Gantry : public task::ManagedTask
    {
    public:
		Gantry(
			motor::Motor& m_x_,
			motor::Motor& m_y_,
			motor::Motor& m_z_,
			motor::JointM& m_p_
		);
	
		~Gantry() = default;
		
		void Set_X(float m_);
		void Set_Y(float m_);
		void Set_Z(float m_);
		void Set_P(float rad_);
		
		inline float Get_X() { return  motor_x.Get_Out_Pos() * GANTRY_X_RAD_TO_M; }
		inline float Get_Y() { return  motor_y.Get_Out_Pos() * GANTRY_Y_RAD_TO_M; }
		inline float Get_Z() { return  motor_z.Get_Out_Pos() * GANTRY_Z_RAD_TO_M; }
		inline float Get_P() { return -motor_p.Get_Out_Pos(); }

    private:
		void Task_Process() override;
		
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
    };
}
#endif
