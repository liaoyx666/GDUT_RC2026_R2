#pragma once
#include "RC_traj_track3.h"
#include "RC_event3.h"
#include "RC_data_pool.h"

#ifdef __cplusplus
namespace path
{
	constexpr float HEAD_CHEAK_THRESHOLD = 4.f * PI / 180.f; /*4度*/
	
	class HeadCheck : public Event3
    {
    public:
		HeadCheck(uint8_t id_, float yaw, TrajTrack3& t_, data::RobotPose& pose_);
		
		void Cheak_Head();
    protected:
		
    private:
		TrajTrack3& track;
		data::RobotPose& pose;
		bool flag;
		float yaw;
    };
}
#endif
