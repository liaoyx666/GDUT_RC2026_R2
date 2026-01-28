#include "RC_data_pool.h"

namespace data
{
	RobotPose::RobotPose() : ManagedTask("ChassisTask", 10, 64, task::TASK_DELAY, 80)
	{
		x = 0;
		y = 0;
		z = 0;
		
		yaw = 0;
		roll = 0;
		pitch = 0;
	
		position_is_valid = false;
		position_last_time = 0;
			
		orientation_is_valid = false;
		orientation_last_time = 0;
	}

	void RobotPose::Update_Position(float * x_, float * y_, float * z_)
	{
		if (x_ != NULL)
		{
			x = *x_;
		}
		
		if (y_ != NULL)
		{
			y = *y_;
		}
		
		if (z_ != NULL)
		{
			z = *z_;
		}

		position_last_time = timer::Timer::Get_TimeStamp();
		position_is_valid = true;
	}
	
	
	void RobotPose::Update_Orientation(float * yaw_, float * roll_, float * pitch_)
	{
		if (yaw_ != NULL)
		{
			yaw = *yaw_;
		}
		
		if (roll_ != NULL)
		{
			roll = *roll_;
		}
		
		if (pitch_ != NULL)
		{
			pitch = *pitch_;
		}

		orientation_last_time = timer::Timer::Get_TimeStamp();
		orientation_is_valid = true;
	}
	
	#define POSITION_TIME_OUT 1000000// us
	#define ORIENTATION_TIME_OUT 1000000// us

	void RobotPose::Task_Process()
	{
		uint32_t delta_time = timer::Timer::Get_DeltaTime(position_last_time);
		
		if (delta_time > POSITION_TIME_OUT)
		{
			position_is_valid = false;
		}

		delta_time = timer::Timer::Get_DeltaTime(orientation_last_time);
		
		if (delta_time > ORIENTATION_TIME_OUT)
		{
			orientation_is_valid = false;
		}
	}

}