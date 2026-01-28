#pragma once
#include "RC_cdc.h"
#include "RC_data_pool.h"

#ifdef __cplusplus
namespace ros
{
	class Radar : cdc::CDCHandler
    {
    public:
		Radar(cdc::CDC &cdc_, uint8_t rx_id_, data::RobotPose& robot_pose_);
		virtual ~Radar() {}
		
		
    protected:
		void CDC_Receive_Process(uint8_t *buf, uint16_t len) override;
		
    private:
		float x = 0, y = 0, z = 0, yaw = 0;
		data::RobotPose* robot_pose;
    };

	
}
#endif
