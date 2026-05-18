#pragma once
#include "RC_task.h"
#include "RC_timer.h"

#ifdef __cplusplus
namespace data
{
	/*================左右半场===================*/
	void Init_Side(bool blue_left_side_);
	bool Is_Side_Init();
	bool Is_Blue_Left_Side();
	/*================左右半场===================*/
	
	
	
	
	/*==============携带KFS的数量=====================*/
	uint8_t KFS_Num();
	void KFS_Add_One();
	void KFS_Sub_One();
	/*==============携带KFS的数量=====================*/
	
	
	
	
	
	#define POSITION_TIME_OUT 1000000// us
	#define ORIENTATION_TIME_OUT 1000000// us

	class RobotPose// : public task::ManagedTask
    {
    public:
		RobotPose();
		~RobotPose() = default;
		
		void Update_Position(float * x_, float * y_, float * z_);
		void Update_Orientation(float * yaw_, float * roll_, float * pitch_);
		
		float X() const {return x;}
		float Y() const {return y;}
		float Z() const {return z;}
		
		float Yaw() const {return yaw;}
		float Roll() const {return roll;}
		float Pitch() const {return pitch;}
		
		bool Is_Position_Valid() const {return position_is_valid;}
		bool Is_Orientation_Valid() const {return orientation_is_valid;}
		
		inline void Robot_Pose_Check()
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

    private:
		//void Task_Process() override;
	
		float x = 0;
		float y = 0;
		float z = 0;
			
		float yaw = 0;
		float roll = 0;
		float pitch = 0;
	
		bool position_is_valid = false;
		uint32_t position_last_time = 0;
			
		bool orientation_is_valid = false;
		uint32_t orientation_last_time = 0;
    };
	













}
#endif
