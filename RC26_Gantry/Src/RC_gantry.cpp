#include "RC_gantry.h"

namespace gantry
{
	
	constexpr float GANTRY_X_MAX = 0.70f;
	constexpr float GANTRY_X_MIN = 0.0f;
	
	constexpr float GANTRY_Y_MAX = 0.2f;
	constexpr float GANTRY_Y_MIN = 0.f;
	
	constexpr float GANTRY_Z_MAX = 0.86f;
	constexpr float GANTRY_Z_MIN = 0.f;
	
	constexpr float GANTRY_P_MAX = TWO_THIRD_PI; // 270度
	constexpr float GANTRY_P_MIN = 0.f;
	

	
	
	Gantry::Gantry(
		motor::Motor& m_x_,
		motor::Motor& m_y_,
		motor::Motor& m_z_,
		motor::JointM& m_p_
	) : motor_x(m_x_), motor_y(m_y_), motor_z(m_z_), motor_p(m_p_), task::ManagedTask("GantryTask", 31, 128, task::TASK_DELAY, 1)
	{
		target_x = 0;
		target_y = 0;
		target_z = 0;
		target_p = 0;
		
		p_max = GANTRY_P_MAX;
		p_min = GANTRY_P_MIN;
	}

	void Gantry::Set_X(float m_)
	{
		if (m_ > GANTRY_X_MAX) target_x = GANTRY_X_MAX;
		else if (m_ < GANTRY_X_MIN) target_x = GANTRY_X_MIN;
		else target_x = m_;
	}
	
	void Gantry::Set_Y(float m_)
	{
		m_ += GANTRY_Y_OFFSET;
		
		if (m_ > GANTRY_Y_MAX) target_y = GANTRY_Y_MAX;
		else if (m_ < GANTRY_Y_MIN) target_y = GANTRY_Y_MIN;
		else target_y = m_;
	}
	
	void Gantry::Set_Z(float m_)
	{
		if (m_ > GANTRY_Z_MAX) target_z = GANTRY_Z_MAX;
		else if (m_ < GANTRY_Z_MIN) target_z = GANTRY_Z_MIN;
		else target_z = m_;
	}
	
	void Gantry::Set_P(float rad_)
	{
		if (rad_ > GANTRY_P_MAX) target_p = GANTRY_P_MAX;
		else if (rad_ < GANTRY_P_MIN) target_p = GANTRY_P_MIN;
		else target_p = rad_;
	}

	void Gantry::Set_Load_Mode(bool loaded_)
	{
		if (load_mode == loaded_)
		{
			return;
		}
		load_mode = loaded_;
		Set_Speed_Scale(speed_scale);
	}

	void Gantry::Set_Speed_Scale(float scale_)
	{
		if (scale_ < 0.1f) scale_ = 0.1f;
		if (scale_ > 1.0f) scale_ = 1.0f;
		speed_scale = scale_;

		// total scale = speed scale * load damping scale
		const float load_s = load_mode ? 0.5f : 1.0f;
		const float s = speed_scale * load_s;

		// Only scale the last four parameters, as requested.
		motor_x.pid_pos.Pid_Param_Init(200, 0, 3, 0, 0.002, 0, 8000, 500, 500, 500 * s, 500 * s, 2000 * s, 8000.f * s);
		motor_z.pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.002, 0, 3000, 1000, 500, 500 * s, 500 * s, 1000 * s, 2000.f * s);
		motor_y.pid_pos.Pid_Param_Init(200, 0, 3, 0, 0.002, 0, 8000, 500, 500, 500 * s, 500 * s, 2000 * s, 8000.f * s);
		motor_p.pid_pos.Pid_Param_Init(20, 0, 0.05, 0, 0.001, 0, 5, 5, 5, 5 * s, 5 * s, 20 * s, 7.f * s);
	}
	

	//P_MIN1
	constexpr float GANTRY_P_MIN_X_CONSTR_X_START_1 = 0.06;
	constexpr float GANTRY_P_MIN_X_CONSTR_X_END_1 = 0.1;
	
	constexpr float GANTRY_P_MIN_X_CONSTR_P_MIN_START_1 = 0;
	constexpr float GANTRY_P_MIN_X_CONSTR_P_MIN_END_1 = 0.2;
	
	//P_MIN2
	constexpr float GANTRY_P_MIN_X_CONSTR_X_START_2 = 0.1;
	constexpr float GANTRY_P_MIN_X_CONSTR_X_END_2 = 0.35;
	
	constexpr float GANTRY_P_MIN_X_CONSTR_P_MIN_START_2 = 0.2;
	constexpr float GANTRY_P_MIN_X_CONSTR_P_MIN_END_2 = 0.2;
	
	//P_MIN3
	constexpr float GANTRY_P_MIN_X_CONSTR_X_START_3 = 0.35;
	constexpr float GANTRY_P_MIN_X_CONSTR_X_END_3 = 0.40;
	
	constexpr float GANTRY_P_MIN_X_CONSTR_P_MIN_START_3 = 0.2;
	constexpr float GANTRY_P_MIN_X_CONSTR_P_MIN_END_3 = 0;
	
	
	
	
	
	
	
	// P_MAX1
	constexpr float GANTRY_P_MAX_X_CONSTR_X_START_1 = 0;
	constexpr float GANTRY_P_MAX_X_CONSTR_X_END_1 = 0.4;
				
	constexpr float GANTRY_P_MAX_X_CONSTR_P_MAX_START_1 = HALF_PI;
	constexpr float GANTRY_P_MAX_X_CONSTR_P_MAX_END_1 = PI;
	
	
	
	// P_MAX2
	constexpr float GANTRY_P_MAX_X_CONSTR_X_START_2 = 0.4;
	constexpr float GANTRY_P_MAX_X_CONSTR_X_END_2 = 0.55;
				
	constexpr float GANTRY_P_MAX_X_CONSTR_P_MAX_START_2 = PI;
	constexpr float GANTRY_P_MAX_X_CONSTR_P_MAX_END_2 = TWO_THIRD_PI;
	
	
	
	void Gantry::Task_Process()
	{
		float x_pos = Get_X();
        float p_pos = Get_P();
		
		if (x_pos >= GANTRY_P_MIN_X_CONSTR_X_START_1 && x_pos <= GANTRY_P_MIN_X_CONSTR_X_END_1)
		{
			p_min = (GANTRY_P_MIN_X_CONSTR_P_MIN_END_1 - GANTRY_P_MIN_X_CONSTR_P_MIN_START_1) / 
					(GANTRY_P_MIN_X_CONSTR_X_END_1 - GANTRY_P_MIN_X_CONSTR_X_START_1) * 
					(x_pos - GANTRY_P_MIN_X_CONSTR_X_START_1) + 
					GANTRY_P_MIN_X_CONSTR_P_MIN_START_1;
		}
		else if (x_pos >= GANTRY_P_MIN_X_CONSTR_X_START_2 && x_pos <= GANTRY_P_MIN_X_CONSTR_X_END_2)
		{
			p_min = (GANTRY_P_MIN_X_CONSTR_P_MIN_END_2 - GANTRY_P_MIN_X_CONSTR_P_MIN_START_2) / 
					(GANTRY_P_MIN_X_CONSTR_X_END_2 - GANTRY_P_MIN_X_CONSTR_X_START_2) * 
					(x_pos - GANTRY_P_MIN_X_CONSTR_X_START_2) + 
					GANTRY_P_MIN_X_CONSTR_P_MIN_START_2;
		}
		else if (x_pos >= GANTRY_P_MIN_X_CONSTR_X_START_3 && x_pos <= GANTRY_P_MIN_X_CONSTR_X_END_3)
		{
			p_min = (GANTRY_P_MIN_X_CONSTR_P_MIN_END_3 - GANTRY_P_MIN_X_CONSTR_P_MIN_START_3) / 
					(GANTRY_P_MIN_X_CONSTR_X_END_3 - GANTRY_P_MIN_X_CONSTR_X_START_3) * 
					(x_pos - GANTRY_P_MIN_X_CONSTR_X_START_3) + 
					GANTRY_P_MIN_X_CONSTR_P_MIN_START_3;
		}
		
		
		
		if (x_pos >= GANTRY_P_MAX_X_CONSTR_X_START_1 && x_pos <= GANTRY_P_MAX_X_CONSTR_X_END_1)
		{
			p_max = (GANTRY_P_MAX_X_CONSTR_P_MAX_END_1 - GANTRY_P_MAX_X_CONSTR_P_MAX_START_1) / 
					(GANTRY_P_MAX_X_CONSTR_X_END_1 - GANTRY_P_MAX_X_CONSTR_X_START_1) * 
					(x_pos - GANTRY_P_MAX_X_CONSTR_X_START_1) + 
					GANTRY_P_MAX_X_CONSTR_P_MAX_START_1;
		}
		else if (x_pos >= GANTRY_P_MAX_X_CONSTR_X_START_2 && x_pos <= GANTRY_P_MAX_X_CONSTR_X_END_2)
		{
			p_max = (GANTRY_P_MAX_X_CONSTR_P_MAX_END_2 - GANTRY_P_MAX_X_CONSTR_P_MAX_START_2) / 
					(GANTRY_P_MAX_X_CONSTR_X_END_2 - GANTRY_P_MAX_X_CONSTR_X_START_2) * 
					(x_pos - GANTRY_P_MAX_X_CONSTR_X_START_2) + 
					GANTRY_P_MAX_X_CONSTR_P_MAX_START_2;
		}
		
		/*-------------------------------p---------------------------------------*/
		
		float constr_p = target_p;
		
		if (target_p > p_max)
		{
			constr_p = p_max ;
		}
		else if (target_p < p_min)
		{
			constr_p = p_min ;
		}
		
		motor_p.Set_Out_Mit_Pos(-constr_p);
		
		/*---------------------------------x-------------------------------------*/
		
		float constr_x = target_x;
		
		if (p_pos > p_max + 0.05f || p_pos < p_min - 0.05f)
		{
			constr_x = x_pos;
		}
		
		motor_x.Set_Out_Pos( constr_x * GANTRY_X_M_TO_RAD);
		
		/*---------------------------------y-------------------------------------*/
		
		motor_y.Set_Out_Pos( target_y * GANTRY_Y_M_TO_RAD);
		
		/*---------------------------------z-------------------------------------*/

		motor_z.Set_Out_Pos( target_z * GANTRY_Z_M_TO_RAD);
	}
}
