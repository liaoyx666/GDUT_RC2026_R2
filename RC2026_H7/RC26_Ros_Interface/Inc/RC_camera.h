#pragma once
#include "RC_cdc.h"
#include "RC_data_pool.h"

#ifdef __cplusplus
namespace ros
{
	class Camera : cdc::CDCHandler
	{
	public:
		Camera(cdc::CDC &cdc_, uint8_t rx_id_, data::RobotPose& robot_pose_);
		~Camera() = default;

		float X() const {return x;}
		float Y() const {return y;}
		float Z() const {return z;}
		float Yaw() const {return yaw;}
		uint8_t Event() const {return event;}

		void Send_ICP_Front();      // id=a → 正面，相机开启
		void Send_ICP_Top();        // id=b → 顶面，相机开启
		void Send_QR_Req();         // id=c → 二维码识别，相机开启
		void Send_Cam_Close();      // id=d → 相机通道关闭

	protected:
		void CDC_Receive_Process(uint8_t *buf, uint16_t len) override;

	private:
		float x = 0, y = 0, z = 0, yaw = 0;
		uint8_t event = 0;
		data::RobotPose* robot_pose;
	};
}
#endif
