#pragma once
#ifdef __cplusplus

#include "RC_dji_motor.h"
#include "RC_data_pool.h"
#include "RC_vector2d.h"
#include "RC_tim.h"
#include "RC_radar.h"

namespace fusion
{
	constexpr uint8_t QEO_DELAY_FRAME = 65; // 30帧  60ms
	
	enum QEOMode : uint8_t
	{
		FUSION_MODE = 0,
		RADAR_MODE,
		QEO_MODE,
	};
	
	
	
	class QEO : public tim::TimHandler
    {
    public:
		QEO(motor::DjiMotor& m1_, motor::DjiMotor& m2_, motor::DjiMotor& m3_, motor::DjiMotor& m4_, data::RobotPose& pose_, tim::Tim& tim_, ros::Radar& radar_);
		~QEO() = default;
		
		void Iteration()
		{
			float yaw = pose.Yaw();
			
			vector2d::Vector2D delta;
			
			delta.x() = (m1.Get_Pos() - m3.Get_Pos()) * QEO_POS_TO_M;// 45
			
			m1.Set_Pos_Zero();
			m3.Set_Pos_Zero();
			
			delta.y() = (m2.Get_Pos() - m4.Get_Pos()) * QEO_POS_TO_M;// 315
			
			m2.Set_Pos_Zero();
			m4.Set_Pos_Zero();
		
			delta = delta.rotate(yaw - RAD45);
			
			vel = delta * (1.f / 0.002f);
			
			pos += delta;
			
			delay_pos[now_dx] = pos;
			now_dx = (now_dx + 1) % QEO_DELAY_FRAME;
		}
		
		constexpr float Delay_X() const { return delay_pos[(now_dx + 1) % QEO_DELAY_FRAME].x(); }
		constexpr float Delay_Y() const { return delay_pos[(now_dx + 1) % QEO_DELAY_FRAME].y(); }
		
		constexpr float X() const { return pos.x(); }
		constexpr float Y() const { return pos.y(); }
		
		constexpr void Set_X(float x_) { pos.x() = x_; }
		constexpr void Set_Y(float y_) { pos.y() = y_; }
		
		constexpr float Vel_X() const { return vel.x(); }
		constexpr float Vel_Y() const { return vel.y(); }
		
		void Fusion()
		{
			if (mode == QEO_MODE) return;
			
			float radar_x = radar.X();
			float radar_y = radar.Y();
			
			if (mode == RADAR_MODE)
			{
				Set_X(radar_x);
				Set_Y(radar_y);
				is_init = false;
				return;
			}
			
			/*--------------------------------------------*/
			if (is_init)
			{
				if (last_radar_x != radar_x)
				{
					last_radar_x = radar_x;
					
					float pos_x = pos.x();
					float pos_y = pos.y();
					
					float error_x = radar_x - Delay_X();
					float error_y = radar_y - Delay_Y();
					
					pos_x += kp * error_x;
					pos_y += kp * error_y;
					
					Set_X(pos_x);
					Set_Y(pos_y);
					/*-------------------------------------*/
					if (fabsf(error_x) > 0.2)
					{
						Set_X(radar_x);
					}
					
					if (!reset_flag_x && fabsf(error_x) > 0.08 && fabsf(Vel_X()) < 0.24f)
					{
						reset_flag_x = true;
						last_time_x = timer::Timer::Get_TimeStamp();
					}
					
					if (reset_flag_x && timer::Timer::Get_DeltaTime(last_time_x) > 1000000) // 100ms
					{
						if (fabsf(error_x) > 0.08 && fabsf(Vel_X()) < 0.24f)
						{
							Set_X(radar_x);
						}
						
						reset_flag_x = false;
					}
					/*-------------------------------------*/
					if (fabsf(error_y) > 0.2)
					{
						Set_Y(radar_y);
					}
					
					if (!reset_flag_y && fabsf(error_y) > 0.08 && fabsf(Vel_Y()) < 0.24f)
					{
						reset_flag_y = true;
						last_time_y = timer::Timer::Get_TimeStamp();
					}
					
					if (reset_flag_y && timer::Timer::Get_DeltaTime(last_time_y) > 1000000) // 100ms
					{
						if (fabsf(error_y) > 0.08 && fabsf(Vel_X()) < 0.24f)
						{
							Set_Y(radar_y);
						}
						
						reset_flag_y = false;
					}
					/*-------------------------------------*/
				}
			}
			else
			{
				Set_X(radar_x);
				Set_Y(radar_y);
				is_init = true;
			}
		}
		
    private:
		void Tim_It_Process() override 
		{
			Iteration();
		}
		
		static constexpr float RAD45 = 0.785398;
		static constexpr float QEO_POS_TO_M = 187.0 / 3591.0 * 0.064 / 2.0 * 0.9721559268098647573587907716786;
		
		motor::DjiMotor& m1;
		motor::DjiMotor& m2;
		motor::DjiMotor& m3;
		motor::DjiMotor& m4;
		
		data::RobotPose& pose;
		
		vector2d::Vector2D pos;
		vector2d::Vector2D delay_pos[QEO_DELAY_FRAME];
		uint8_t now_dx;
		
		ros::Radar& radar;
		float last_radar_x;
		
		uint32_t last_time_x;
		uint32_t last_time_y;
		
		const float kp = 0.003f;
		
		vector2d::Vector2D vel;
		
		bool reset_flag_x;
		bool reset_flag_y;
		
		bool is_init;
		
		QEOMode mode;
		friend class FusionCtrl;
    };
}
#endif
