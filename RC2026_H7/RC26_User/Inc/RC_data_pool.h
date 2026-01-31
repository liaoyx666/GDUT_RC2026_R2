#pragma once
#include "RC_task.h"
#include "RC_timer.h"

#ifdef __cplusplus
namespace data
{
	class RobotPose : public task::ManagedTask
    {
    public:
		RobotPose();
		virtual ~RobotPose() {}
		
		void Update_Position(float * x_, float * y_, float * z_);
		void Update_Orientation(float * yaw_, float * roll_, float * pitch_);
		
		float const * Get_pX() const {return &x;}
		float const * Get_pY() const {return &y;}
		float const * Get_pZ() const {return &z;}
		
		float const * Get_pYaw() const {return &yaw;}
		float const * Get_pRoll() const {return &roll;}
		float const * Get_pPitch() const {return &pitch;}
		
		bool Is_Position_Valid() const {return position_is_valid;}
		bool Is_Orientation_Valid() const {return orientation_is_valid;}
		
    protected:
		
    
    private:
		void Task_Process() override;
	
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
