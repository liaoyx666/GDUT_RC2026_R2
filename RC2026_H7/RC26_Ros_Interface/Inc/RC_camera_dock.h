#pragma once
#include "RC_cdc.h"

#ifdef __cplusplus
namespace ros
{
	class Camera : cdc::CDCHandler
	{
	public:
		Camera(cdc::CDC &cdc_, uint8_t rx_id_);
		~Camera() = default;

		float X() const { return x; }
		float Y() const { return y; }
		float Z() const { return z; }
		float Yaw() const { return yaw; }
		uint8_t Event() const { return event; }

		// 检查是否有新帧到达（调用后自动清零）
		bool Check_New_Data()
		{
			if (new_data) { new_data = false; return true; }
			return false;
		}

		bool QR_Enable();                     // 发送 QR 请求，返回 Is_QR_Enabled()
		bool QR_Disable();                    // 发送 QR 关闭，返回 !Is_QR_Enabled()
		bool Is_QR_Enabled() const { return is_qr_enabled; }

		void Send_QR_Req();         // id=c → 二维码识别，相机开启
		void Send_QR_Close();       // id=d → 相机通道关闭

	protected:
		void CDC_Receive_Process(uint8_t *buf, uint16_t len) override;

	private:
		uint8_t id;
		volatile float x = 0, y = 0, z = 0, yaw = 0;
		volatile uint8_t event = 0;
		volatile bool new_data = false;
		volatile bool is_qr_enabled = false;
	};
}
#endif
