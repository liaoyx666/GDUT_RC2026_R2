#include "RC_HWT101CT.h"

HWT101CT::HWT101CT(UART_HandleTypeDef &huart_, uint8_t* buf_) : serial::UartRx(huart_, buf_, HWT101CT_RX_BUFFER_SIZE, true, true)
{
	now_dx = 0;
	yaw = 0;
	raw = 0;
	w = 0;
	offset = 0;
	is_init = false;
}

void HWT101CT::Uart_Rx_It_Process(uint8_t *buf_, uint16_t len_)
{
	if (len_ == 22)
	{
		if (buf_[0] == 0x55 && buf_[1] == 0x52 && buf_[10] == (uint8_t)(0x55 + 0x52 + buf_[4] + buf_[5] + buf_[6] + buf_[7]))
		{
			w = (float)( (int16_t)( (buf_[7] << 8) | buf_[6]) ) * (float)(1.0 / 32768.0 * 34.906585039886591538473815369772);
		}
		
		if (buf_[0 + 11] == 0x55 && buf_[1 + 11] == 0x53 && buf_[10 + 11] == (uint8_t)(0x55 + 0x53 + buf_[6 + 11] + buf_[7 + 11] + buf_[8 + 11] + buf_[9 + 11]))
		{
			raw = (float)( (int16_t)( (buf_[7 + 11] << 8) | buf_[6 + 11]) ) * (float)(1.0 / 32768.0 * PI);
			
			if (is_init)
			{
				float y = raw - offset;
				
				if (y > PI)
					y -= TWO_PI;
				else if (y < -PI)
					y += TWO_PI;
				
				yaw = y;

				delay_yaw[now_dx] = yaw;
				now_dx = (now_dx + 1) % HWT101CT_DELAY_FRAME;
			}
			else
			{
				offset = raw;
				is_init = true;
			}
		}
	}
	
	
}
