#pragma once
#ifdef __cplusplus

#include "RC_dji_motor.h"
#include "RC_data_pool.h"
#include "RC_vector2d.h"
#include "RC_tim.h"
#include "RC_radar.h"

namespace qeo
{
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
			
			pos += delta;
		}
		
		void Fusion()
		{
			
		}
		
		
		
		
		constexpr float X() const { return pos.x(); }
		constexpr float Y() const { return pos.y(); }
		
    private:
		void Tim_It_Process() override 
		{
			Iteration();
		}
	
		static constexpr float RAD45 = 0.785398;
		static constexpr float COS45 = 0.70710678118654752440084436210485;
		static constexpr float QEO_POS_TO_M = 187.0 / 3591.0 * 0.064 / 2.0 * 0.9721559268098647573587907716786;
		
		motor::DjiMotor& m1;
		motor::DjiMotor& m2;
		motor::DjiMotor& m3;
		motor::DjiMotor& m4;
	
		vector2d::Vector2D pos;
		
		ros::Radar& radar;
		float last_x;
		
		data::RobotPose& pose;
    };
}
#endif
