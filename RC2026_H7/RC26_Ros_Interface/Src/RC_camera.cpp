#include "RC_camera.h"

namespace ros
{
	Camera::Camera(cdc::CDC &cdc_, uint8_t rx_id_, data::RobotPose& robot_pose_) : cdc::CDCHandler(cdc_, rx_id_), robot_pose(&robot_pose_)
	{

	}

	void Camera::CDC_Receive_Process(uint8_t *buf, uint16_t len)
	{
		if (len == 16)
		{
			x   = *(float*)(&buf[0]);
			y   = *(float*)(&buf[4]);
			z   = *(float*)(&buf[8]);
			yaw = *(float*)(&buf[12]);
			event = 1;
			new_data = true;                    // 通知有新帧
		}
	}

	void Camera::Send_QR_Req()
	{
		uint8_t data = 1;
		cdc->CDC_Send_Pkg(0x06, &data, 1, 1000);   // id=c → 二维码识别
	}

	void Camera::Send_QR_Close()
	{
		uint8_t data = 1;
		cdc->CDC_Send_Pkg(0x07, &data, 1, 1000);   // id=d → 相机关闭
	}
}
