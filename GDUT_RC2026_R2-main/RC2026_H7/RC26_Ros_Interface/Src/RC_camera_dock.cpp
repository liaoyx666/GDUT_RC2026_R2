#include "RC_camera_dock.h"

namespace ros
{
	Camera::Camera(cdc::CDC &cdc_, uint8_t rx_id_) : cdc::CDCHandler(cdc_, rx_id_), id(rx_id_)
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
			new_data = true;
			is_qr_enabled = true;
		}
		else if (len == 1 && buf[0] == 0)
		{
			is_qr_enabled = false;
		}
	}

	void Camera::Send_QR_Req()
	{
		uint8_t data = 1;
		cdc->CDC_Send_Pkg(id, &data, 1, 0); 
	}

	void Camera::Send_QR_Close()
	{
		uint8_t data = 0;
		cdc->CDC_Send_Pkg(id, &data, 1, 0);   
	}
	bool Camera::QR_Enable()
	{
		Send_QR_Req();
		
		return is_qr_enabled;
	}

	bool Camera::QR_Disable()
	{
		Send_QR_Close();
		return !is_qr_enabled;
	}
}
