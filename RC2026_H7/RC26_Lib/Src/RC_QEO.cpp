#include "RC_QEO.h"

namespace fusion
{
	QEO::QEO(motor::DjiMotor& m1_, motor::DjiMotor& m2_, motor::DjiMotor& m3_, motor::DjiMotor& m4_, data::RobotPose& pose_, tim::Tim& tim_, ros::Radar& radar_)
	: m1(m1_),
	  m2(m2_),
	  m3(m3_),
	  m4(m4_),
	  pose(pose_), 
	  tim::TimHandler(&tim_),
	  radar(radar_)
	{
		now_dx = 0;
		is_init = false;
		
		reset_flag_x = false;
		reset_flag_y = false;
		
		last_radar_x = 0;
		last_time_x = 0;
		last_time_y = 0;
		
		mode = FUSION_MODE;
	}
}