#pragma once
#include "RC_serial.h"
#include "RC_filter.h"
#ifdef __cplusplus

constexpr uint8_t HWT101CT_RX_BUFFER_SIZE = 32;

//enum HWT101CT_STATE : uint8_t
//{
//	HWT101CT_WAIT_HEAD = 0,
//	HWT101CT_WAIT_TYPE,
//	HWT101CT_WAIT_
//};

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
private:
	void Uart_Rx_It_Process(uint8_t *buf_, uint16_t len_) override;
	float yaw;
	float raw;
	float w;
	float offset;
	bool is_init;
};
#endif
