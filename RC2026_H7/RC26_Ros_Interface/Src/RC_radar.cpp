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
			float x_ = *(float*)(&buf[0]);
			float y_ = *(float*)(&buf[4]);
			
			if (fabsf(x_) < 30) x = x_;
			if (fabsf(y_) < 30) y = y_;
			
			z = *(float*)(&buf[8]);
			
			float yaw_ = *(float*)(&buf[12]);
			
			if (fabsf(yaw_) < 4) yaw = yaw_;
			
			robot_pose->Update_Position(&x, &y, &z);
			//robot_pose->Update_Orientation(&yaw, NULL, NULL);
		}
	}
	
	void Radar::Reposition()
	{
		uint8_t ack = 1;
		cdc->CDC_Send_Pkg(4, &ack, 1, 100);
	}

	

}