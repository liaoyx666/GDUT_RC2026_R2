#include "RC_omni_chassis.h"

namespace chassis
{
    Omni4Chassis::Omni4Chassis(
        motor::Motor& m1, motor::Motor& m2,
        motor::Motor& m3, motor::Motor& m4,
        float max_lv, float la, float ld,
        float max_av, float aa, float ad,
		data::RobotPose& pose_
    ) : Chassis(max_lv, la, ld, max_av, aa, ad, pose_)
    {
        drive_motor[0] = &m1;
        drive_motor[1] = &m2;
        drive_motor[2] = &m3;
        drive_motor[3] = &m4;

        Set_Is_Init_True();
    }

    void Omni4Chassis::Chassis_Init()
	{
		Set_Is_Init_True();
    }

    void Omni4Chassis::Chassis_Re_Init()
	{
		Set_Is_Init_True();
    }
    
    void Omni4Chassis::Kinematics_calc(vector2d::Vector2D v_, float vw_)
    {
        float y = -v_.y();
        float x =  v_.x();

		float vel[4] = {0};
		
        vel[0] = x * (-COS_45) - y * ( SIN_45) + OMNI4_CHASSIS_L * vw_;
        vel[1] = x * (-COS_45) - y * (-SIN_45) + OMNI4_CHASSIS_L * vw_;
        vel[2] = x * ( COS_45) - y * (-SIN_45) + OMNI4_CHASSIS_L * vw_;
        vel[3] = x * ( COS_45) - y * ( SIN_45) + OMNI4_CHASSIS_L * vw_;

        for (int i = 0; i < 4; i++)
		{
            drive_motor[i]->Set_Out_Rpm(-vel[i] * vel_to_rpm);
		}
    }
}