#pragma once
#include "RC_event3.h"
#include "RC_task.h"
#include "RC_traj_track3.h"
#include "RC_data_pool.h"
#include "RC_lift_chassis.h"


#ifdef __cplusplus
namespace chassis
{
	class AutoLift : public task::ManagedTask
    {
    public:
		AutoLift(LiftChassis& lift_, path::TrajTrack3& track_, data::RobotPose& pose_);
		~AutoLift() = default;

	
    private:
		void Task_Process() override;
	
		path::Event3 lift_event[8];
		path::Event3 check_event[4];
	
		path::TrajTrack3& track;
		data::RobotPose& pose;
		bool check_flag;
		uint8_t check_dx;
		float yaw;
	
		LiftChassis& lift;
		bool lift_trig;
		LiftAction la;
		LiftHeigth lh;
		LiftDir ld;
    };
}
#endif
