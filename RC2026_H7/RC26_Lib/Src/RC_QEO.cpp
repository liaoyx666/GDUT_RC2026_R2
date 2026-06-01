#include "RC_QEO.h"

namespace qeo
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
		
	}
}