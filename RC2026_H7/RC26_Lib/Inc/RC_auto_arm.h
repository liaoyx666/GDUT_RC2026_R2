#pragma once
#include "RC_arm.h"
#include "RC_path2.h"

#ifdef __cplusplus
namespace arm
{
	class AutoArm
    {
    public:
		AutoArm(Arm_task& arm_task_, path::PathPlan2& path_plan_, uint8_t event_arm_up_id_, uint8_t event_arm_down_id_);
		virtual ~AutoArm() {}
		
		void Auto_Arm();
    protected:
    
    private:
		uint8_t kfs_count = 0;
	
		path::PathEvent2 event_arm_up;
		path::PathEvent2 event_arm_down;
		uint8_t arm_state = 0;
	
		Arm_task* arm_task;
    };
}
#endif

