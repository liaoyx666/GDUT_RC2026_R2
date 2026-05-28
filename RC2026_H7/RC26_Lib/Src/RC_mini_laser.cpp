#include "RC_mini_laser.h"

namespace mini_laser
{
    MiniLaser::MiniLaser(UART_HandleTypeDef &huart_, uint8_t* buf_)
        : serial::UartRx(huart_, buf_, MINI_LASER_RX_BUFFER_SIZE, true, true), filter(150, 1000, 0.707)
    {
		dis_filter = 0;
    }

    void MiniLaser::Uart_Rx_It_Process(uint8_t *buf_, uint16_t len_)
    {
		if (len_ != 4) return;

		if (buf_[0] == 0xEE && buf_[3] == (uint8_t)(buf_[0] + buf_[1] + buf_[2]))
		{
			distance = (uint16_t)((buf_[1] << 8) | buf_[2]);
			dis_filter = filter.filter((float)distance);
		}
		
    }
}
