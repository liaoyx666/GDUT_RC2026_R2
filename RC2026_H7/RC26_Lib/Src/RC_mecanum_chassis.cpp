#include "RC_mecanum_chassis.h"

/*

2    1
               ----->    轮子顺序     
3    4

*/

namespace chassis
{
	Mecanum4Chassis::Mecanum4Chassis(
		motor::Motor& drive_motor_1_, motor::Motor& drive_motor_2_, motor::Motor& drive_motor_3_, motor::Motor& drive_motor_4_,
        motor::Motor& auxiliary_motor_1_, motor::Motor& auxiliary_motor_2_, 
        motor::Motor& lift_lower_motor_,
        float max_linear_vel_, float linear_accel_, float linear_decel_,
		float max_angular_vel_, float angular_accel_, float angular_decel_
	) : 
	Chassis(
		max_linear_vel_, linear_accel_, linear_decel_,
		max_angular_vel_, angular_accel_, angular_decel_
	)
	{

		drive_motor[0] = &drive_motor_1_;
		drive_motor[1] = &drive_motor_2_;
		drive_motor[2] = &drive_motor_3_;
		drive_motor[3] = &drive_motor_4_;

        auxiliary_motor[0] = &auxiliary_motor_1_;
        auxiliary_motor[1] = &auxiliary_motor_2_; 

        lift_lower_motor = lift_lower_motor_;

	}
	
	void Mecanum4Chassis::Kinematics_calc(vector2d::Vector2D v_, float vw_)
	{
		/*************************底盘解算************************/

        float vx = v_.x();
        float vy = v_.y();
        float w = vw_;

        float a = MECANUM4_CHASSIS_TRACK_WIDTH / 2;
        float b = MECANUM4_CHASSIS_WHEELBASE / 2;

        vel[0] = vy - vx + w * ( a + b );
        vel[1] = vy + vx - w * ( a + b );
        vel[2] = vy - vx - w * ( a + b );
        vel[3] = vy + vx + w * ( a + b );
        

		/***********************设置电机**************************/
		for (uint8_t i = 0; i < 4; i++)
		{
			// 设置航向电机速度
			drive_motor[i]->Set_Out_Rpm(vel[i] * vel_to_rpm * ((float)drive_motor_sign[i]));
		}
		
		/*************************************************/
	}

    void up_stair()
    {
        
        if()// 靠近台阶
        {
            lift_body();
        }

    }

    void down_stair()
    {
        
    }
    
    void lift_body()
    {
        
    }

    void lower_body()
    {
        
    }


}