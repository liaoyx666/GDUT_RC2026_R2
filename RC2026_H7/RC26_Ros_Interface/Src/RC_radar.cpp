#include "RC_radar.h"


namespace ros
{
	Radar::Radar(cdc::CDC &cdc_, uint8_t rx_id_, data::RobotPose& robot_pose_) : cdc::CDCHandler(cdc_, rx_id_), robot_pose(&robot_pose_)
	{
		
	}
	
	void Radar::CDC_Receive_Process(uint8_t *buf, uint16_t len)
	{
		if (len == 16)
		{
			x   = *(float*)(&buf[0]);
			y   = *(float*)(&buf[4]);
			z   = *(float*)(&buf[8]);
			yaw = *(float*)(&buf[12]);
			
			robot_pose->Update_Position(&x, &y, &z);
				
			robot_pose->Update_Orientation(&yaw, NULL, NULL);

		}
	}

	

}