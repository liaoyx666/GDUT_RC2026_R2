#include "RC_data_pool.h"




namespace data
{
	/*==============左右半场=====================*/
	static bool blue_left_side = true;  /*场地位置*/
	static bool is_side_init = false;

	void Init_Side(bool blue_left_side_)
	{
		if (!is_side_init)
		{
			blue_left_side = blue_left_side_;
			is_side_init = true;
		}
	}
	bool Is_Side_Init() { return is_side_init; }
	bool Is_Blue_Left_Side() { return blue_left_side; }
	/*============左右半场=======================*/
	
	
	/*==============携带KFS的数量=====================*/
	static uint8_t KFS_num = 0;
	
	uint8_t KFS_Num() { return KFS_num; }
	void Set_KFS_Num(uint8_t kfs_num) { KFS_num = kfs_num; }
	void KFS_Add_One() { KFS_num++; }
	void KFS_Sub_One() { KFS_num--; }
	/*==============携带KFS的数量=====================*/
	
	
	
	RobotPose::RobotPose()// : ManagedTask("ChassisTask", 10, 64, task::TASK_DELAY, 80)
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
	
	

	

}