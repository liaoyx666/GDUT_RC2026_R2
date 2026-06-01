#pragma once
#include "RC_serial.h"
#include "RC_filter.h"
#include "RC_timer.h"

#ifdef __cplusplus

constexpr uint8_t HWT101CT_RX_BUFFER_SIZE = 32;
constexpr uint8_t HWT101CT_DELAY_FRAME = 20; // 15帧  40ms


class HWT101CT : public serial::UartRx
{
public:
	HWT101CT(UART_HandleTypeDef &huart_, uint8_t* buf_);
	~HWT101CT() = default;
	constexpr float Yaw() const {return yaw;}
	
	constexpr float W() const {return w;}
	
	constexpr void Set_Yaw(const float yaw_)
	{
		float e = raw - yaw_;
		
		if (e > PI)
			e -= TWO_PI;
		else if (e < -PI)
			e += TWO_PI;

		offset = e;
	}
	
	constexpr float Delay_Yaw()
	{
		return delay_yaw[(now_dx + 1) % HWT101CT_DELAY_FRAME];
	}
	
	
private:
	void Uart_Rx_It_Process(uint8_t *buf_, uint16_t len_) override;

	float yaw;
	float raw;
	float w;
	float offset;

	float delay_yaw[HWT101CT_DELAY_FRAME]{};
	uint8_t now_dx;

	bool is_init;
};
#endif
